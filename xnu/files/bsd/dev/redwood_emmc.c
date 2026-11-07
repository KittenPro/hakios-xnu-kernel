/*
 * The X6826B's eMMC as a block device: the "xnusys" partition, where the system image lives, read
 * when the kernel asks for it rather than held in memory. Boot-arg emmc_root=1 makes it the root
 * (iokit/bsddev/IOKitBSDInit.cpp asks redwood_emmc_root for it); /dev/emmc0 and /dev/remmc0 are its
 * nodes once devfs is up.
 *
 * Until now the loader read the whole image -- 2.4 GB of it -- into DRAM as the RAM disk md0,
 * in the 4 GB the kernel has: 2.5 GB wired before the first program ran, the rest too little for
 * iOS with its wallpaper (run 320: 10-26 MB free, the system "unhealthy", jetsam at work), and
 * three and a half minutes of every boot spent reading it (11 MB/s).
 *
 * Nothing is set up here: LK has powered the card, set its bus up and left it in the transfer
 * state, and mtkloader has read the GPT through it the same way (mtkloader/msdc.h) -- MSDC0 in PIO
 * mode, plain block reads (CMD17, or CMD18 then CMD12). Registers as Linux's
 * drivers/mmc/host/mtk-sd.c has them.
 *
 * rosemary's storage is not on the MSDC at all: it is a UFS 2.2 device behind the host controller
 * at 0x11270000 (Linux drives it as ufshcd-mtk, its three LUNs are sda, sdb and the 119 GiB sdc
 * whose GPT was carved), and every command to the MSDC times out there. When no card is found on
 * the MSDC, the UFS transport below takes over instead: LK has brought the controller and the link
 * up to read the boot image and left them running, so this code only points the transfer list at
 * its own descriptors and rings the doorbell -- a host reset with a DME_LINK_STARTUP is the
 * fallback. The register addresses and the command sequences were read out of lk-mt6765's ufs_aio
 * driver (lk/platform/common/storage/ufs/, as LK runs it on this family of SoCs) and Linux's
 * drivers/scsi/ufs; none of that code is used.
 *
 * Read only, by construction: the strategy routine refuses writes and opening for writing is
 * refused. Reads queue deep: thirty-one slots of the transport's own, as many commands in
 * flight at once as there are askers of the pager, in pieces of a slot's UFS_SLOT_BLOCKS.
 */
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <miscfs/devfs/devfs.h>
#include <pexpert/pexpert.h>
#include <sys/buf.h>
#include <sys/conf.h>
#include <sys/disk.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/redwood_emmc.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern uint64_t kvtophys(vm_offset_t va);    /* as redwood_mali.c declares it */
extern void delay(int usec);                 /* as redwood_connsys.c declares it */

#define MSDC0_PHYS       0x11230000u
#define EMMC_PART        "xnusys"
#define EMMC_MAX_BLOCKS  256                /* 128 KB a command */

#define MSDC_CFG         0x00
#define MSDC_CFG_PIO     (1u << 3)
#define MSDC_INT         0x0c
#define MSDC_FIFOCS      0x14
#define MSDC_FIFOCS_CLR  (1u << 31)
#define MSDC_RXDATA      0x1c
#define SDC_CMD          0x34
#define SDC_ARG          0x38
#define SDC_STS          0x3c
#define SDC_STS_BUSY     3u                 /* SDCBUSY, CMDBUSY */
#define SDC_BLK_NUM      0x50

#define INT_CMDRDY       (1u << 8)
#define INT_CMDTMO       (1u << 9)
#define INT_RSPCRCERR    (1u << 10)
#define INT_XFER_COMPL   (1u << 12)
#define INT_DATTMO       (1u << 14)
#define INT_DATCRCERR    (1u << 15)

#define CMD_RSP_R1       (1u << 7)
#define CMD_RSP_R1B      (6u << 7)
#define CMD_DTYPE_SINGLE (1u << 11)
#define CMD_DTYPE_MULTI  (2u << 11)
#define CMD_STOP         (1u << 14)
#define CMD_BLKLEN(n)    (((n) & 0xfffu) << 16)

static vm_offset_t msdc;
static uint64_t emmc_lba, emmc_blocks;      /* the partition: its first block, and the image's length in blocks */
static int emmc_bmajor = -1, emmc_cmajor = -1;
static uint32_t emmc_int_seen;              /* MSDC_INT bits read (reading clears them) but not yet used */
static const char *emmc_failed;
static uint32_t emmc_failed_status;
static uint64_t emmc_reads, emmc_read_blocks, emmc_errors;
static LCK_GRP_DECLARE(emmc_lck_grp, "redwood-emmc");
static LCK_MTX_DECLARE(emmc_lock, &emmc_lck_grp);

static inline uint32_t
rd(uint32_t reg)
{
	return *(volatile uint32_t *)(msdc + reg);
}

static inline void
wr(uint32_t reg, uint32_t value)
{
	*(volatile uint32_t *)(msdc + reg) = value;
}

static bool
fail(const char *what, uint32_t status)
{
	emmc_failed = what;
	emmc_failed_status = status;
	return false;
}

/* (spins are counted in register reads, as the loader does: about a microsecond each, give or take) */
static bool
wait_idle(void)
{
	for (uint32_t i = 0; i < 2000000; i++) {
		if (!(rd(SDC_STS) & SDC_STS_BUSY)) {
			return true;
		}
	}
	return fail("controller busy", rd(SDC_STS));
}

static bool
wait_int(uint32_t want, uint32_t bad, const char *what)
{
	for (uint32_t i = 0; i < 20000000; i++) {
		uint32_t status = rd(MSDC_INT);

		if (status) {
			wr(MSDC_INT, status);
			emmc_int_seen |= status;
		}
		if (emmc_int_seen & bad) {
			status = emmc_int_seen;
			emmc_int_seen = 0;
			return fail(what, status);
		}
		if (emmc_int_seen & want) {
			emmc_int_seen &= ~want;
			return true;
		}
	}
	return fail(what, emmc_int_seen);
}

static void
pio_mode(void)
{
	wr(MSDC_CFG, rd(MSDC_CFG) | MSDC_CFG_PIO);
	wr(MSDC_INT, rd(MSDC_INT));
	emmc_int_seen = 0;
	wr(MSDC_FIFOCS, rd(MSDC_FIFOCS) | MSDC_FIFOCS_CLR);
	for (uint32_t i = 0; i < 100000 && (rd(MSDC_FIFOCS) & MSDC_FIFOCS_CLR); i++) {
	}
}

static bool
stop(void)
{
	wr(SDC_ARG, 0);
	wr(SDC_CMD, 12 | CMD_RSP_R1B | CMD_STOP);
	return wait_int(INT_CMDRDY, INT_CMDTMO | INT_RSPCRCERR, "stop") && wait_idle();
}

/* blocks of 512 bytes from the card's block lba into dst (4-aligned) */
static bool
read_blocks(uint64_t lba, uint32_t blocks, uint32_t *dst)
{
	if (!wait_idle()) {
		return false;
	}
	pio_mode();
	wr(SDC_BLK_NUM, blocks);
	wr(SDC_ARG, (uint32_t)lba);                 /* high capacity: the unit is a block */
	wr(SDC_CMD, (blocks == 1 ? 17 : 18) | CMD_RSP_R1 | (blocks == 1 ? CMD_DTYPE_SINGLE : CMD_DTYPE_MULTI) | CMD_BLKLEN(512));
	if (!wait_int(INT_CMDRDY, INT_CMDTMO | INT_RSPCRCERR, "command")) {
		return false;
	}
	for (uint32_t left = blocks * 512 / 4; left;) {
		uint32_t fifo = rd(MSDC_FIFOCS) & 0xff, spin = 0, words;

		while (fifo < 4) {
			uint32_t status = rd(MSDC_INT);

			if (status & (INT_DATTMO | INT_DATCRCERR)) {
				wr(MSDC_INT, status);
				if (blocks > 1) {
					(void)stop();
				}
				return fail("data", status);
			}
			if (++spin > 20000000) {
				return fail("data stalled", rd(MSDC_FIFOCS));
			}
			fifo = rd(MSDC_FIFOCS) & 0xff;
		}
		words = fifo / 4 < left ? fifo / 4 : left;
		for (uint32_t i = 0; i < words; i++) {
			*dst++ = rd(MSDC_RXDATA);
		}
		left -= words;
	}
	if (!wait_int(INT_XFER_COMPL, INT_DATTMO | INT_DATCRCERR, "transfer end")) {
		return false;
	}
	return blocks == 1 || stop();
}

/* ---- the UFS transport (rosemary: see the note in the file's head) ----
 *
 * ufs19 takes the upstream tree's own driver for its model: a transfer list of this kernel's
 * own with room for thirty-two commands at once.  A demand-paged boot is many askers of one
 * page each, so the reads take slots 1..31 as they are free, many in flight together -- and
 * the line that dies at ~4 ms of idle (ufs3: a line reset at the PHY adapter) never sees an
 * idle while a page is being read: the queue is the keepalive.  Slot 0 is the keepalive's
 * own: while nothing else is in flight it reads one block of LUN 0 every millisecond, as
 * ufs1-18 always did, but in a slot and a descriptor of its own, sharing nothing with the
 * reads.  A waiter spins forty microseconds, then sleeps; one poller thread watches every
 * sleeping waiter's doorbell bits (nothing in XNU serves this controller's interrupts, so IE
 * stays zero throughout and a command is finished when its doorbell bit clears -- the IS is
 * read only to say what went wrong).  Nothing fights the controller on a command's failure
 * any more: the read is simply tried again over whatever stands, and the ladder (HIBERN8_EXIT
 * with the list rebuilt; the controller from cold as the last rung) is walked once per
 * episode under the recovery lock, while other askers wait the episode out and try again. */

#define UFS_PHYS          0x11270000u    /* as lk-mt6765's ufs_aio has it */
#define UFS_CAP           0x00
#define UFS_VER           0x08
#define UFS_IS            0x20           /* write one to clear */
#define UFS_IE            0x24
#define UFS_HCS           0x30           /* DP 1, UTRLRDY 2, UCRDY 8 */
#define UFS_HCE           0x34
#define UFS_UECPA          0x38           /* the UIC error codes: one per layer, together they name the killer */
#define UFS_UECDL          0x3c
#define UFS_UECN           0x40
#define UFS_UECT           0x44
#define UFS_AHIT          0x18           /* auto-hibernate: after this much idle the link is put away */
#define UFS_UTRLBA_L      0x50
#define UFS_UTRLBA_H      0x54
#define UFS_UTRLDBR       0x58           /* a bit rings its slot; it clears when the command is done */
#define UFS_UTRLCLR       0x5c
#define UFS_UTRLRSR       0x60
#define UFS_UIC_CMD       0x90
#define UFS_UIC_ARG1      0x94
#define UFS_UIC_ARG2      0x98           /* a DME command's result: its low byte */
#define UFS_UIC_ARG3      0x9c
#define UFS_REFCLK        0x144          /* MediaTek: the reference clock's handshake */
#define UFS_MPHY          0x2200         /* MediaTek: the MPHY block (the deep stall and its kin) */
#define UFS_STALL         0x2210         /* MediaTek: automatic deep stall */
#define UFS_OCS_ERR       0x2244         /* MediaTek's own: what the last command died of */
#define UFS_DBG_SEL       0x22c0         /* MediaTek: which window PROBE looks through */
#define UFS_PROBE         0x22c8         /* 0x20 there: the link's own state machine (1 idle, 8..0xd entering hibernation) */

#define UFS_IS_UTRCS      (1u << 0)
#define UFS_IS_UICERR     (1u << 2)
#define UFS_IS_UICCOMPL   (1u << 10)
#define UFS_HCS_DP        (1u << 0)
#define UFS_HCS_UTRLRDY   (1u << 1)
#define UFS_HCS_UCRDY     (1u << 3)
#define UFS_UIC_DME_GET    0x01           /* a MIB read: the value comes back in ARG3 */
#define UFS_UIC_DME_SET    0x02           /* a MIB write: the value rides in ARG3 */
#define UFS_UIC_LINK_STARTUP 0x16
#define UFS_UIC_HIBER_ENTER  0x17
#define UFS_UIC_HIBER_EXIT   0x18
#define UFS_VS_DBGCLK      0xd0a1         /* unipro: bit 5 forces the TX symbol clock request on */
#define UFS_VS_SAVEPOWER   0xd0a6         /* unipro: bits 0/2/3 gate the RX/SYS/TX clocks, bit 6 is the deep stall */
#define UFS_VS_UNIPRO_PD   0xd0a8         /* unipro: nonzero is its low-power mode */

/* an area of this kernel's own: the thirty-two UTRDs at its start (the list base wants
 * 1 KB), every slot's command descriptor a kilobyte in -- its command UPIU, its response,
 * and one PRD entry, for a slot's data is one physical piece and one entry covers any read */
#define UFS_SLOTS         32u
#define UFS_UCD_OFF       0x400
#define UFS_UCD_SIZE      2048u
#define UFS_CMD_OFF       0              /* within a slot's command descriptor */
#define UFS_RSP_OFF       512
#define UFS_PRD_OFF       1024
#define UFS_AREA_SIZE     (UFS_UCD_OFF + UFS_SLOTS * UFS_UCD_SIZE)
#define UFS_SLOT_BLOCKS   64u            /* a read slot's data: 256 KB of 4 KB blocks, one READ10 */
#define UFS_SPIN_US       40u            /* a waiter spins this long, then sleeps */
#define UFS_TIMEOUT_US    10000000u      /* ten seconds: the slot's command is given up for lost */

/* what a command came to */
#define UFS_OK       0                        /* the data is in the slot's buffer */
#define UFS_REFUSED  1                        /* the device answered, and refused the command */
#define UFS_SILENT   2                        /* no answer: the transport failed */

struct ufs_slot {
	uint32_t number;                     /* in the list; 0 is the keepalive's, 1..31 the reads' */
	volatile uint32_t *utrd;             /* its descriptor and its command descriptor: this */
	volatile uint32_t *ucd;              /* kernel's memory, kept clean for the controller by hand */
	uint64_t ucd_pa;
	uint8_t *data;                       /* its own landing place: slot 0 one block, a read's 256 KB */
	uint64_t data_pa;
};

static vm_offset_t ufs;
static bool ufs_on;                          /* reads go through the transport */
static uint8_t ufs_lun;                      /* the LUN the partition was found on */
static vm_offset_t ufs_area;                 /* the UTRDs and the command descriptors */
static uint64_t ufs_area_pa;
static struct ufs_slot ufs_slots[UFS_SLOTS];
static uint32_t ufs_free_bits;               /* the reads' slots 1..31 that are free (ufs_slot_lock) */
static uint32_t ufs_in_flight, ufs_most_in_flight;
static volatile uint32_t ufs_waiting, ufs_done_bits;   /* the poller's (ufs_slot_lock) */
static bool ufs_poller_asleep;
static bool ufs_answered;                    /* the device answered, even to refuse: the link is fine */
static bool ufs_attention;                   /* the device answered "I was reset, ask again": a unit attention */
static volatile bool ufs_recovering;         /* an episode is walking: the keepalive stands aside */
static uint64_t ufs_quick_fail_last;           /* the keepalive complains of a failure at most once a second */
static uint64_t ufs_reads, ufs_read_bytes_total, ufs_errors;
static uint64_t ufs_keepalives, ufs_keepalive_fails, ufs_episodes, ufs_cold_starts;
static uint64_t ufs_last_cold;               /* mach_absolute_time of the last cold start: not two an hour */
static LCK_GRP_DECLARE(ufs_lck_grp, "redwood-ufs");
static LCK_MTX_DECLARE(ufs_slot_lock, &ufs_lck_grp);      /* the slots and their bits */
static LCK_MTX_DECLARE(ufs_recover_lock, &ufs_lck_grp);   /* one episode at a time */

static inline uint32_t
ufs_rd(uint32_t reg)
{
	return *(volatile uint32_t *)(ufs + reg);
}

static inline void
ufs_wr(uint32_t reg, uint32_t value)
{
	*(volatile uint32_t *)(ufs + reg) = value;
}

/* the path to the controller is not IO-coherent: as redwood_lx.c does for its radio memory,
 * what the controller is to read is written out, and what it is to write is dropped first */
static void
ufs_clean(vm_offset_t address, vm_size_t size)
{
	for (vm_offset_t line = address; line < address + size; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__asm__ volatile ("dsb sy");
}

/* one DME command, polled like the rest */
static bool
ufs_uic(uint32_t opcode, uint32_t arg1, uint32_t arg3, const char *what)
{
	uint32_t is = 0, i;

	for (i = 0; i < 100000 && !(ufs_rd(UFS_HCS) & UFS_HCS_UCRDY); i++) {
	}
	if (!(ufs_rd(UFS_HCS) & UFS_HCS_UCRDY)) {
		return fail(what, ufs_rd(UFS_HCS));
	}
	is = ufs_rd(UFS_IS);
	if (is) {
		ufs_wr(UFS_IS, is);                  /* write one to clear */
	}
	ufs_wr(UFS_UIC_ARG1, arg1);
	ufs_wr(UFS_UIC_ARG2, 0);
	ufs_wr(UFS_UIC_ARG3, arg3);
	ufs_wr(UFS_UIC_CMD, opcode & 0xff);
	for (i = 0; i < 20000000; i++) {
		is = ufs_rd(UFS_IS);
		if (is) {
			ufs_wr(UFS_IS, is);
		}
		if (is & (UFS_IS_UICCOMPL | UFS_IS_UICERR)) {
			break;
		}
	}
	if (!(is & (UFS_IS_UICCOMPL | UFS_IS_UICERR))) {
		return fail(what, is);
	}
	if ((is & UFS_IS_UICERR) || (ufs_rd(UFS_UIC_ARG2) & 0xff) != 0) {
		printf("ufs: %s: IS %#x, result %#x\n", what, is, ufs_rd(UFS_UIC_ARG2) & 0xff);
		return fail(what, ufs_rd(UFS_UIC_ARG2));
	}
	return true;
}

/* the two DME commands the unipro's own settings are reached through */
static bool
ufs_dme_get(uint32_t attr, uint32_t *val, const char *what)
{
	if (!ufs_uic(UFS_UIC_DME_GET, attr << 16, 0, what)) {
		return false;
	}
	*val = ufs_rd(UFS_UIC_ARG3);
	return true;
}

static bool
ufs_dme_set(uint32_t attr, uint32_t val, const char *what)
{
	return ufs_uic(UFS_UIC_DME_SET, attr << 16, val, what);
}

static bool ufs_fill(struct ufs_slot *slot, uint8_t lun, uint64_t lba, uint32_t blocks);
static bool ufs_ring(struct ufs_slot *slot, bool episode);
static bool ufs_await(struct ufs_slot *slot, uint64_t deadline, bool quick);
static int ufs_status(struct ufs_slot *slot, bool quick);

/* at most one ufs complaint a second: under the storm a thousand commands fail a second,
 * and ufs18 lost its hour's whole middle to the log of it -- the counters keep the entire
 * story for the sysctl, the log keeps one line a second */
static bool
ufs_log_ok(void)
{
	uint64_t now = mach_absolute_time(), ns = 0;

	absolutetime_to_nanoseconds(now - ufs_quick_fail_last, &ns);
	if (ns < NSEC_PER_SEC) {
		return false;
	}
	ufs_quick_fail_last = now;
	return true;
}

/* the millisecond keepalive: the line dies at ~4 ms of idle (ufs3: a line reset at the PHY
 * adapter) and nothing brings it back whole (ufs5, ufs6: a cold start buys one answered
 * command, then silence) -- a DME peer read every millisecond did not stop it (ufs7: the
 * killer does not count UIC traffic), so this sends a real READ10 over the line, one block
 * of a boot LUN.  It owns slot 0 outright -- its own descriptor, its own buffer, no lock
 * shared with the reads -- and rings only when no episode walks and the queue itself is
 * not keeping the line busy: with reads in flight the queue is the keepalive, and this
 * thread is what holds the line through the true idles between them.  A thread of its own
 * -- and the wait between reads a true spin on the time counter: ufs8's callout muted
 * itself over its own transient refusals, ufs9's starved for six whole seconds while the
 * failing exec walked its quiet path, and ufs10's thread fared no better, for delay() is
 * not busy: it sleeps (xnu's clock.c parks the thread with timer leeway), and ufs10's
 * slept straight through the launchd fight -- six seconds without one complaint while the
 * controller held no device at all (HCS 0x8), waking only to find the device freshly
 * reset.  A spinning thread asks no scheduler when the line is worth keeping */
static void
ufs_keepalive_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint64_t period;

	clock_interval_to_absolutetime_interval(1, NSEC_PER_MSEC, &period);
	for (;;) {
		uint64_t until = mach_absolute_time() + period;
		uint64_t deadline;

		if (ufs_on && !ufs_recovering && (ufs_rd(UFS_UTRLDBR) & ~1u) == 0) {
			/* the reads' slots all quiet: the line is this thread's to hold */
			clock_interval_to_absolutetime_interval(2000, NSEC_PER_USEC, &deadline);
			deadline += mach_absolute_time();
			ufs_keepalives++;
			if (!ufs_fill(&ufs_slots[0], 0, 0, 1) || !ufs_ring(&ufs_slots[0], false) ||
			    !ufs_await(&ufs_slots[0], deadline, true) || ufs_status(&ufs_slots[0], true) != UFS_OK) {
				ufs_keepalive_fails++;
			}
		}
		while (mach_absolute_time() < until) {
			/* a true spin: no sleep, no leeway, no scheduler; one core buys the line */
		}
	}
}

/* the link's own state machine, through MediaTek's window: 1 is idle, 8 through 0xd are the
 * states of entering hibernation (Linux waits the same window out before it touches the timer) */
static uint32_t
ufs_sm(void)
{
	ufs_wr(UFS_DBG_SEL, 0x20);
	return ufs_rd(UFS_PROBE) & 0x1f;
}

/* the transfer list pointed at this side's page (the loader's descriptors may be anywhere by
 * now): base addresses are only to be written while the list is stopped */
static bool
ufs_list(void)
{
	ufs_wr(UFS_UTRLRSR, 0);
	for (uint32_t i = 0; i < 100000 && (ufs_rd(UFS_UTRLRSR) & 1); i++) {
	}
	ufs_wr(UFS_UTRLCLR, 0xffffffffu);        /* every doorbell bit: none is in use */
	ufs_wr(UFS_UTRLBA_L, (uint32_t)ufs_area_pa);
	ufs_wr(UFS_UTRLBA_H, (uint32_t)(ufs_area_pa >> 32));
	ufs_wr(UFS_UTRLRSR, 1);
	for (uint32_t i = 0; i < 100000 && !(ufs_rd(UFS_HCS) & UFS_HCS_UTRLRDY); i++) {
	}
	if (!(ufs_rd(UFS_HCS) & UFS_HCS_UTRLRDY)) {
		return fail("transfer list", ufs_rd(UFS_HCS));
	}
	return true;
}

/* the fallback: the controller from cold, as the loader brings it up -- HCE off and on, the
 * link started again with a DME command -- then the list; the pair is retried the way LK
 * retries it, for the first startup after a toggle often fails on its own (ufs5: IS 0x4) */
static bool
ufs_reset(void)
{
	bool up = false;

	for (uint32_t try = 0; try < 5 && !up; try++) {
		ufs_wr(UFS_HCE, 0);
		for (uint32_t i = 0; i < 2000000 && (ufs_rd(UFS_HCE) & 1); i++) {
		}
		delay(5000);
		ufs_wr(UFS_HCE, 1);
		for (uint32_t i = 0; i < 20000000 && !(ufs_rd(UFS_HCE) & 1); i++) {
		}
		ufs_wr(UFS_IE, 0);                       /* again: nothing in XNU would serve it */
		ufs_wr(UFS_AHIT, 0);                     /* a cold start re-arms it: the same death would wait again */
		if (!(ufs_rd(UFS_HCE) & 1)) {
			printf("ufs: the controller would not enable (HCE %#x); toggling again\n", ufs_rd(UFS_HCE));
			continue;
		}
		if (ufs_uic(UFS_UIC_LINK_STARTUP, 0, 0, "link startup") && (ufs_rd(UFS_HCS) & UFS_HCS_DP)) {
			up = true;
			break;
		}
		printf("ufs: the link would not start (HCS %#x), attempt %u; toggling the controller again\n", ufs_rd(UFS_HCS), try);
	}
	if (!up) {
		return fail("link startup", ufs_rd(UFS_HCS));
	}
	return ufs_list();
}

/* what the controller wrote back is not IO-coherent either: its lines are dropped before
 * they are read, or a stale line of ours would answer for it */
static void
ufs_inval(vm_offset_t address, vm_size_t size)
{
	for (vm_offset_t line = address; line < address + size; line += 64) {
		__asm__ volatile ("dc ivac, %0" : : "r" (line));
	}
	__asm__ volatile ("dsb sy");
}

/* one READ10 built into a slot of its own: the UTRD and the UPIU in the byte order each is
 * read in (the UTRD little endian, the UPIU big endian, byte by byte), the slot's buffer as
 * the one PRD entry -- no other slot's descriptor or data is touched, so a fill needs no
 * lock; the task tag is the slot's own number, and the controller's answer comes back to
 * the slot that asked */
static bool
ufs_fill(struct ufs_slot *slot, uint8_t lun, uint64_t lba, uint32_t blocks)
{
	uint32_t bytes = blocks * 4096;
	volatile uint32_t *d = slot->utrd;
	uint32_t *p = (uint32_t *)(void *)((vm_offset_t)slot->ucd + UFS_PRD_OFF);
	uint8_t *c = (uint8_t *)(void *)slot->ucd + UFS_CMD_OFF;

	if (blocks == 0 || blocks > UFS_SLOT_BLOCKS) {
		return fail("too many blocks", blocks);
	}
	bzero((void *)slot->utrd, 32);
	bzero((void *)slot->ucd, UFS_PRD_OFF);
	d[0] = 0x15000000u;                      /* device to host, the command-type bit the loader's driver sets, interrupt command */
	d[2] = 0x0000000fu;                      /* OCS: invalid until the controller writes its zero */
	d[4] = (uint32_t)slot->ucd_pa;           /* the command descriptor, both halves little endian */
	d[5] = (uint32_t)(slot->ucd_pa >> 32);
	d[6] = (128u << 16) | 128u;              /* its response UPIU: 128 words long, at word 128 */
	d[7] = (256u << 16) | 1u;                /* one PRD entry, at word 256 */
	c[0] = 0x01;                             /* transaction: command */
	c[1] = 0x40;                             /* flags: read */
	c[2] = lun;
	c[3] = (uint8_t)slot->number;            /* task tag: the slot's own number */
	c[12] = (uint8_t)(bytes >> 24);          /* the data expected, big endian */
	c[13] = (uint8_t)(bytes >> 16);
	c[14] = (uint8_t)(bytes >> 8);
	c[15] = (uint8_t)bytes;
	c[16] = 0x28;                            /* READ10 */
	c[17] = 0x08;                            /* FUA, as the loader's driver reads */
	c[18] = (uint8_t)(lba >> 24);            /* the block, big endian */
	c[19] = (uint8_t)(lba >> 16);
	c[20] = (uint8_t)(lba >> 8);
	c[21] = (uint8_t)lba;
	c[23] = (uint8_t)(blocks >> 8);          /* how many blocks, big endian */
	c[24] = (uint8_t)blocks;
	p[0] = (uint32_t)slot->data_pa;          /* where the data goes: one piece, so one entry */
	p[1] = (uint32_t)(slot->data_pa >> 32);
	p[3] = bytes - 1;                        /* PRD entries count one less, as they do */

	ufs_clean((vm_offset_t)slot->utrd, 32);  /* what the controller reads */
	ufs_clean((vm_offset_t)slot->ucd, UFS_PRD_OFF + 16);
	ufs_clean((vm_offset_t)slot->data, bytes);   /* where it writes: no stale line of ours */
	return true;
}

/* the slot rung, under the slot lock -- for the lock is what an episode holds around its
 * list rebuild, and a command rung against a list being stopped is swallowed: an episode's
 * own commands (episode true) pass, everyone else's wait the episode out */
static bool
ufs_ring(struct ufs_slot *slot, bool episode)
{
	uint32_t is;

	lck_mtx_lock(&ufs_slot_lock);
	if (!episode && ufs_recovering) {
		lck_mtx_unlock(&ufs_slot_lock);
		return false;                    /* an episode owns the list: this command is not rung */
	}
	is = ufs_rd(UFS_IS);
	if (is) {
		ufs_wr(UFS_IS, is);
	}
	ufs_wr(UFS_UTRLDBR, 1u << slot->number);
	lck_mtx_unlock(&ufs_slot_lock);
	return true;
}

/* the wait: forty microseconds spun, then asleep with the poller watching the bit -- no
 * core burned on a read that takes its milliseconds; the keepalone's quick wait spins it
 * whole (sleeping waits starve: ufs8-10).  A command that outlives its deadline is aborted
 * with the list-clear bit and its slot is free to take another */
static bool
ufs_await(struct ufs_slot *slot, uint64_t deadline, bool quick)
{
	const uint32_t bit = 1u << slot->number;
	uint64_t spin_until, tick;

	clock_interval_to_absolutetime_interval(UFS_SPIN_US, NSEC_PER_USEC, &spin_until);
	spin_until += mach_absolute_time();
	clock_interval_to_absolutetime_interval(1, NSEC_PER_MSEC, &tick);
	while (ufs_rd(UFS_UTRLDBR) & bit) {
		uint64_t now = mach_absolute_time();

		if ((int64_t)(now - deadline) >= 0) {
			ufs_wr(UFS_UTRLCLR, bit);    /* the slot's run bit cleared, the slot reusable */
			return false;
		}
		if (quick || (int64_t)(now - spin_until) < 0) {
			continue;
		}
		lck_mtx_lock(&ufs_slot_lock);
		if (!(ufs_rd(UFS_UTRLDBR) & bit)) {   /* done while the lock was taken */
			lck_mtx_unlock(&ufs_slot_lock);
			return true;
		}
		ufs_waiting |= bit;                    /* the poller watches the bit from here */
		if (ufs_poller_asleep) {
			wakeup((caddr_t)&ufs_waiting);
		}
		(void)lck_mtx_sleep_deadline(&ufs_slot_lock, LCK_SLEEP_DEFAULT, (event_t)&ufs_done_bits, THREAD_UNINT,
		    now + tick);                       /* a millisecond at a time: a stray wake costs nothing */
		ufs_done_bits &= ~bit;                 /* ours, taken; another waiter's bits stay */
		lck_mtx_unlock(&ufs_slot_lock);
	}
	return true;
}

/* the command's answer read back: the OCS the controller wrote into the UTRD, the response
 * and the sense the device wrote into its UPIU; one line a second at most, whatever the
 * storm, for the counters keep the story */
static int
ufs_status(struct ufs_slot *slot, bool quick)
{
	volatile uint32_t *d = slot->utrd;
	volatile uint8_t *r = (volatile uint8_t *)slot->ucd + UFS_RSP_OFF;
	uint32_t ocs, resp;

	ufs_inval((vm_offset_t)slot->utrd, 32);
	ufs_inval((vm_offset_t)slot->ucd, UFS_PRD_OFF);
	ocs = d[2] & 0xf;
	resp = (*(volatile uint32_t *)(void *)(r + 4) >> 16) & 0xffff;   /* response and status, big endian where they sit */
	if (ocs == 0 && resp == 0) {
		return UFS_OK;
	}
	ufs_answered = ocs == 0;                 /* the device took the command, even to refuse it */
	ufs_attention = ufs_answered && r[36] == 6;   /* sense key 6: the device was reset and wants the command again */
	if (ufs_log_ok()) {
		if (quick) {
			printf("ufs: the keepalive's read: OCS %u, response %#x, sense %#x, IS %#x, HCS %#x, UECPA %#x, sm %u\n",
			    ocs, resp, r[36], ufs_rd(UFS_IS), ufs_rd(UFS_HCS), ufs_rd(UFS_UECPA), ufs_sm());
		} else {
			printf("ufs: slot %u: OCS %u, response %#x, sense %#x %#x %#x, IS %#x, HCS %#x, RSR %#x, UECPA %#x, sm %u\n",
			    slot->number, ocs, resp, r[36], r[46], r[47], ufs_rd(UFS_IS), ufs_rd(UFS_HCS), ufs_rd(UFS_UTRLRSR),
			    ufs_rd(UFS_UECPA), ufs_sm());
		}
	}
	return ocs == 0 ? UFS_REFUSED : UFS_SILENT;
}

/* the reads' slots: one taken from the free bits, one given back; slot 0 is never in them
 * (the keepalive's own), so the engine and the keepalive never meet */
static struct ufs_slot *
ufs_slot_take(void)
{
	struct ufs_slot *slot = NULL;

	lck_mtx_lock(&ufs_slot_lock);
	if (ufs_free_bits != 0) {
		uint32_t low = ufs_free_bits & (0u - ufs_free_bits);
		uint32_t number = (uint32_t)__builtin_ctz(low);

		ufs_free_bits &= ~low;
		ufs_in_flight++;
		if (ufs_in_flight > ufs_most_in_flight) {
			ufs_most_in_flight = ufs_in_flight;
		}
		slot = &ufs_slots[number];
	}
	lck_mtx_unlock(&ufs_slot_lock);
	return slot;
}

static void
ufs_slot_give(struct ufs_slot *slot)
{
	lck_mtx_lock(&ufs_slot_lock);
	ufs_free_bits |= 1u << slot->number;
	if (ufs_in_flight > 0) {
		ufs_in_flight--;
	}
	lck_mtx_unlock(&ufs_slot_lock);
}

/* the poller: one thread for every sleeping waiter -- it watches the doorbell bits of those
 * who have spun their forty microseconds and sleep, wakes each as its bit clears, and
 * otherwise parks itself a hundred milliseconds at a time (a waiter waking it by the flag) */
static void
ufs_poller_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint64_t tick;

	clock_interval_to_absolutetime_interval(100, NSEC_PER_MSEC, &tick);
	for (;;) {
		uint32_t waiting = ufs_waiting;

		if (waiting != 0) {
			uint32_t done = ~ufs_rd(UFS_UTRLDBR) & waiting;

			if (done != 0) {
				lck_mtx_lock(&ufs_slot_lock);
				ufs_done_bits |= done;
				ufs_waiting &= ~done;
				lck_mtx_unlock(&ufs_slot_lock);
				wakeup((caddr_t)&ufs_done_bits);
			} else {
				delay(100);              /* a hundred microseconds: waiters are waiting */
			}
			continue;
		}
		lck_mtx_lock(&ufs_slot_lock);
		if (ufs_waiting == 0) {
			ufs_poller_asleep = true;
			(void)lck_mtx_sleep_deadline(&ufs_slot_lock, LCK_SLEEP_DEFAULT, (event_t)&ufs_waiting, THREAD_UNINT,
			    mach_absolute_time() + tick);
			ufs_poller_asleep = false;
		}
		lck_mtx_unlock(&ufs_slot_lock);
	}
}

/* one read placed across every free slot: the pieces fill and ring together, and the
 * collection waits each in turn -- this is the depth the line needs, for the pager asks one
 * page of every thread of it, and thirty-one answers in flight is what keeps the ~4 ms death
 * from ever seeing an idle.  `drop` bytes of the first piece and `take` bytes in all are
 * what the caller wants of the read (its 512-byte blocks sit anywhere across the LUN's
 * 4096-byte ones); the pieces' data is copied out of the slots as each is collected */
static int
ufs_read_once(uint8_t lun, uint64_t lba, uint32_t blocks, uint8_t *dst, uint32_t drop, uint32_t take, bool episode)
{
	struct ufs_slot *placed[UFS_SLOTS];
	uint32_t counts[UFS_SLOTS], starts[UFS_SLOTS];
	bool rung[UFS_SLOTS];
	uint32_t placed_blocks = 0, copied = 0, want = take;
	int rc = UFS_OK;

	while (placed_blocks < blocks) {
		uint32_t np = 0, i;

		if (!episode && ufs_recovering) {
			delay(100);                /* an episode walks: its owner retries, this asker waits */
			continue;
		}
		while (placed_blocks < blocks && np < UFS_SLOTS) {
			struct ufs_slot *slot = ufs_slot_take();

			if (slot == NULL) {
				break;             /* every read slot busy: what is placed is collected first */
			}
			counts[np] = blocks - placed_blocks;
			if (counts[np] > UFS_SLOT_BLOCKS) {
				counts[np] = UFS_SLOT_BLOCKS;
			}
			starts[np] = placed_blocks;
			if (!ufs_fill(slot, lun, lba + placed_blocks, counts[np])) {
				ufs_slot_give(slot);
				return UFS_REFUSED;
			}
			placed[np] = slot;
			placed_blocks += counts[np];
			np++;
		}
		if (np == 0) {
			delay(100);                /* nothing placed, nothing to collect: a hundred microseconds */
			continue;
		}
		for (i = 0; i < np; i++) {
			rung[i] = ufs_ring(placed[i], episode);
		}
		for (i = 0; i < np; i++) {
			uint64_t deadline;
			int st = UFS_SILENT;

			clock_interval_to_absolutetime_interval(UFS_TIMEOUT_US, NSEC_PER_USEC, &deadline);
			deadline += mach_absolute_time();
			if (rung[i] && ufs_await(placed[i], deadline, false)) {
				st = ufs_status(placed[i], false);
			}                            /* not rung (an episode took the list), or its ten seconds ran out */
			if (st == UFS_OK) {
				uint32_t skip = starts[i] == 0 ? drop : 0;
				uint32_t have = counts[i] * 4096 - skip;
				uint32_t n = want < have ? want : have;

				ufs_inval((vm_offset_t)placed[i]->data, (vm_size_t)counts[i] * 4096);
				memcpy(dst + copied, placed[i]->data + skip, n);
				copied += n;
				want -= n;
			} else if (rc == UFS_OK) {
				rc = st;             /* the first failure is the read's own story */
			}
			ufs_slot_give(placed[i]);
		}
		if (rc != UFS_OK) {
			return rc;               /* the whole read is the caller's to try over what stands */
		}
	}
	return rc;
}

/* the episode, and the read that walks it: one episode at a time under the recovery lock,
 * every other asker waiting it out (their reads are not rung while it walks).  The rungs
 * are the old ladder's -- the link woken and the list rebuilt, the link started warm, and
 * the controller from cold as the last rung of last resort, kept to one an hour, for
 * ufs18's last half hour was thirty-six cold resets, one collapse every six seconds, each
 * poisoning the line it meant to fix.  Nothing else fights the controller any more */
static bool
ufs_read_or_wait(uint8_t lun, uint64_t lba, uint32_t blocks, uint8_t *dst, uint32_t drop, uint32_t take)
{
	int rc = ufs_read_once(lun, lba, blocks, dst, drop, take, false);

	if (rc == UFS_REFUSED) {
		rc = ufs_read_once(lun, lba, blocks, dst, drop, take, false);   /* the device was reset meanwhile: ask again */
		if (rc == UFS_REFUSED) {
			return false;              /* the link is fine: the device refuses this command */
		}
	}
	if (rc == UFS_OK) {
		return true;
	}
	lck_mtx_lock(&ufs_recover_lock);
	lck_mtx_lock(&ufs_slot_lock);
	ufs_recovering = true;
	lck_mtx_unlock(&ufs_slot_lock);
	ufs_episodes++;
	if (ufs_log_ok()) {
		printf("ufs: episode %llu: the link went quiet (read at %llu); waking it\n", ufs_episodes, lba);
	}
	(void)ufs_uic(UFS_UIC_HIBER_EXIT, 0, 0, "waking the link");
	rc = ufs_list() ? ufs_read_once(lun, lba, blocks, dst, drop, take, true) : UFS_SILENT;
	if (rc == UFS_REFUSED) {
		rc = ufs_read_once(lun, lba, blocks, dst, drop, take, true);
	}
	if (rc != UFS_OK && ufs_uic(UFS_UIC_LINK_STARTUP, 0, 0, "starting the link warm") && (ufs_rd(UFS_HCS) & UFS_HCS_DP)) {
		rc = ufs_list() ? ufs_read_once(lun, lba, blocks, dst, drop, take, true) : UFS_SILENT;
		if (rc == UFS_REFUSED) {
			rc = ufs_read_once(lun, lba, blocks, dst, drop, take, true);
		}
	}
	if (rc != UFS_OK) {
		uint64_t now = mach_absolute_time(), ns = 0;

		absolutetime_to_nanoseconds(now - ufs_last_cold, &ns);
		if (ufs_last_cold == 0 || ns > (uint64_t)3600 * NSEC_PER_SEC) {
			ufs_last_cold = now;
			ufs_cold_starts++;
			printf("ufs: episode %llu: the controller from cold (run %llu)\n", ufs_episodes, ufs_cold_starts);
			(void)ufs_reset();
			for (uint32_t i = 0; i < 5 && rc != UFS_OK; i++) {
				rc = ufs_read_once(lun, lba, blocks, dst, drop, take, true);
				if (rc == UFS_REFUSED) {
					rc = ufs_read_once(lun, lba, blocks, dst, drop, take, true);
				}
				if (rc != UFS_OK && ufs_attention) {
					(void)ufs_uic(UFS_UIC_LINK_STARTUP, 0, 0, "starting the link over the awake device");
					(void)ufs_list();
				}
			}
		}
	}
	lck_mtx_lock(&ufs_slot_lock);
	ufs_recovering = false;
	lck_mtx_unlock(&ufs_slot_lock);
	lck_mtx_unlock(&ufs_recover_lock);
	if (rc == UFS_OK) {
		return true;
	}
	ufs_errors++;
	emmc_errors++;
	return false;
}

/* the caller's 512-byte blocks out of any LUN's 4096-byte ones: each piece is read whole
 * into its slots and the asked-for bytes copied out of them */
static bool
ufs_read_lun(uint8_t lun, uint64_t lba, uint32_t blocks, uint32_t *dst)
{
	uint8_t *out = (uint8_t *)dst;

	while (blocks > 0) {
		uint64_t at = lba >> 3;              /* the 4096-byte block these bytes start in */
		uint32_t skip = (uint32_t)(lba & 7) * 512;
		uint32_t need = blocks * 512;
		uint32_t n4k = (skip + need + 4095) / 4096;
		uint32_t take;

		if (n4k > UFS_SLOT_BLOCKS) {
			n4k = UFS_SLOT_BLOCKS;
		}
		take = need < n4k * 4096 - skip ? need : n4k * 4096 - skip;
		if (!ufs_read_or_wait(lun, at, n4k, out, skip, take)) {
			return false;
		}
		ufs_reads++;
		ufs_read_bytes_total += take;
		out += take;
		lba += take / 512;
		blocks -= take / 512;
	}
	return true;
}

/* the system partition's own: the same, through the LUN it was found on */
static bool
ufs_read_bytes_r(uint64_t lba, uint32_t blocks, uint32_t *dst)
{
	bool ok = ufs_read_lun(ufs_lun, lba, blocks, dst);

	if (ok) {
		emmc_reads++;
		emmc_read_blocks += blocks;
	}
	return ok;
}

/* brings the transport up: LK has left the controller and the link running, it read the boot
 * image through them moments before, so this only takes the interrupts off, points the
 * transfer list at this side's area and reads once; the reset is the fallback */
static bool
ufs_start(void)
{
	vm_offset_t at = 0;
	uint32_t savepower = 0, dbgclk = 0;
	thread_t th;

	/* the transfer list of this kernel's own: thirty-two UTRDs at the start (the list base
	 * wants 1 KB), every slot's command descriptor a kilobyte in -- and every slot its own
	 * landing place, the keepalive's slot 0 one block, the reads' slots 256 KB apiece */
	if (kmem_alloc_contig(kernel_map, &at, UFS_AREA_SIZE, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("ufs: no %u bytes for the transfer list\n", (uint32_t)UFS_AREA_SIZE);
		return false;
	}
	ufs_area = at;
	ufs_area_pa = kvtophys(ufs_area);
	for (uint32_t n = 0; n < UFS_SLOTS; n++) {
		struct ufs_slot *slot = &ufs_slots[n];
		vm_size_t bytes = n == 0 ? PAGE_SIZE : (vm_size_t)UFS_SLOT_BLOCKS * 4096;

		slot->number = n;
		slot->utrd = (volatile uint32_t *)(ufs_area + (uint64_t)n * 32);
		slot->ucd = (volatile uint32_t *)(ufs_area + UFS_UCD_OFF + (uint64_t)n * UFS_UCD_SIZE);
		slot->ucd_pa = ufs_area_pa + UFS_UCD_OFF + (uint64_t)n * UFS_UCD_SIZE;
		at = 0;
		if (kmem_alloc_contig(kernel_map, &at, bytes, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
		    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
			printf("ufs: no %u bytes in one piece for slot %u\n", (uint32_t)bytes, n);
			return false;
		}
		slot->data = (uint8_t *)at;
		slot->data_pa = kvtophys(at);
	}
	ufs_free_bits = 0xfffffffeu;              /* the reads' slots 1..31; slot 0 is the keepalive's own */
	ufs = ml_io_map(UFS_PHYS, 0x3000);
	if (ufs == 0) {
		return false;
	}
	printf("ufs: HCE %#x, HCS %#x, CAP %#x, VER %#x\n", ufs_rd(UFS_HCE), ufs_rd(UFS_HCS), ufs_rd(UFS_CAP), ufs_rd(UFS_VER));
	ufs_wr(UFS_IE, 0);
	/* the preloader leaves the auto-hibernate idle timer armed; Linux on this family (the
	 * mt8183-ufshci quirk) marks it broken and zeroes it: once it fires the controller drops
	 * commands and refuses to wake -- exactly the death ufs1 took at its first ~4 ms idle */
	printf("ufs: AHIT %#x, refclk %#x, mphy %#x, stall %#x; clearing the idle timer\n", ufs_rd(UFS_AHIT), ufs_rd(UFS_REFCLK),
	    ufs_rd(UFS_MPHY), ufs_rd(UFS_STALL));
	ufs_wr(UFS_AHIT, 0);
	delay(5000);
	if ((ufs_rd(UFS_HCE) & 1) == 0) {
		printf("ufs: the controller is off; bringing it up from cold\n");
		if (!ufs_reset()) {
			return false;
		}
	} else if (!ufs_list()) {
		printf("ufs: the list would not come up; resetting\n");
		if (!ufs_reset()) {
			return false;
		}
	}
	/* Linux's pre-link recipe, run warm (ufs2: AHIT was already 0, yet the death at the first
	 * ~4 ms idle was the same; ufs3 caught the death itself: UECPA 0x80000010, a line reset,
	 * with the clock gates and the deep stall already off -- so SAVEPOWERCONTROL goes to zero
	 * whole, its unnamed bits included) */
	(void)ufs_dme_get(UFS_VS_SAVEPOWER, &savepower, "reading SAVEPOWERCONTROL");
	(void)ufs_dme_get(UFS_VS_DBGCLK, &dbgclk, "reading DEBUGCLOCKENABLE");
	printf("ufs: unipro: savepower %#x, dbgclk %#x, sm %u; clearing every gate and stall\n", savepower, dbgclk, ufs_sm());
	(void)ufs_dme_set(UFS_VS_UNIPRO_PD, 0, "clearing the unipro low-power mode");
	(void)ufs_dme_set(UFS_VS_SAVEPOWER, 0, "clearing the whole of SAVEPOWERCONTROL");
	(void)ufs_dme_set(UFS_VS_DBGCLK, dbgclk | 0x20u, "forcing the TX symbol clock on");
	(void)ufs_dme_get(UFS_VS_SAVEPOWER, &savepower, "re-reading SAVEPOWERCONTROL");
	(void)ufs_dme_get(UFS_VS_DBGCLK, &dbgclk, "re-reading DEBUGCLOCKENABLE");
	printf("ufs: unipro now: savepower %#x, dbgclk %#x\n", savepower, dbgclk);
	/* the poller before the first sleeping waiter, the keepalive before the very first read
	 * (ufs10's run died in the tail of this very function -- an idle the line did not
	 * survive): better two threads too early than one read too late */
	if (kernel_thread_start(ufs_poller_thread, NULL, &th) != KERN_SUCCESS) {
		printf("ufs: no poller thread: the waiters would spin\n");
		return false;
	}
	thread_deallocate(th);
	if (kernel_thread_start(ufs_keepalive_thread, NULL, &th) != KERN_SUCCESS) {
		printf("ufs: no keepalive thread: the line will idle\n");
		return false;
	}
	thread_deallocate(th);
	ufs_on = true;
	/* one read of LUN 0 (a boot LUN, small but alive) says the link answers -- through the
	 * whole of the new path: a slot of its own, a ring, the poller behind it, an episode if
	 * it must; slot 1's buffer is the scratch, for the keepalive holds slot 0 */
	if (!ufs_read_or_wait(0, 0, 1, ufs_slots[1].data, 0, 4096)) {
		ufs_on = false;
		printf("ufs: the first read failed; the transport is not up\n");
		return false;
	}
	printf("ufs: the transport is up: %u read slots of %u KB, a poller, a keepalive\n", UFS_SLOTS - 1,
	    (uint32_t)(UFS_SLOT_BLOCKS * 4096 >> 10));
	return true;
}

/* the card's own path (the MSDC), tried again once after a failure (the controller cleared
 * and the card told to stop) -- the UFS path needs none of this: its retries and its
 * episodes are its own */
static bool
read_retrying(uint64_t lba, uint32_t blocks, uint32_t *dst)
{
	if (read_blocks(lba, blocks, dst)) {
		return true;
	}
	emmc_errors++;
	printf("emmc: reading %u blocks at %llu: %s (%#x); trying again\n", blocks, lba, emmc_failed, emmc_failed_status);
	pio_mode();
	(void)stop();
	return read_blocks(lba, blocks, dst);
}

/* whichever transport the system came up on */
static bool
read_root(uint64_t lba, uint32_t blocks, uint32_t *dst)
{
	return ufs_on ? ufs_read_bytes_r(lba, blocks, dst) : read_retrying(lba, blocks, dst);
}

/* ---- the partition ---- */

static uint32_t sector[128] __attribute__((aligned(16)));
static uint32_t entries[128 * 128 / 4] __attribute__((aligned(16)));

static bool
find_partition(void)
{
	const uint8_t *s = (const uint8_t *)sector;
	uint64_t entries_lba, at;
	uint32_t count, size;

	/* The GPT is laid out on the card's own sectors: 512 bytes on the X6826B, 4096 on rosemary
	 * (its header sits at byte 4096, the entry array at 8192). Both answer reads in 512-byte
	 * blocks, so a GPT sector is gpt512 of them and every LBA below comes in those.
	 * rosemary's boot LUNs have GPTs of their own, without the partition: each LUN is read
	 * until one has it, quietly -- only the last failure speaks on the card. */
	for (uint8_t lun = 0; lun < (ufs_on ? 4 : 1); lun++) {
		uint32_t gpt512 = 0;

		for (uint32_t gs = 512; gs <= 4096 && gpt512 == 0; gs *= 2) {
			if ((ufs_on ? ufs_read_lun(lun, gs / 512, 1, sector) : read_retrying(gs / 512, 1, sector)) &&
			    memcmp(s, "EFI PART", 8) == 0) {
				gpt512 = gs / 512;
			}
		}
		if (gpt512 == 0) {
			if (!ufs_on) {
				printf("emmc: no GPT (%s)\n", emmc_failed ? emmc_failed : "no signature");
				return false;
			}
			continue;                       /* a boot LUN's own GPT is elsewhere, or none: the next */
		}
		memcpy(&entries_lba, s + 72, 8);
		memcpy(&count, s + 80, 4);
		memcpy(&size, s + 84, 4);
		if (count > 128 || size != 128 ||
		    !(ufs_on ? ufs_read_lun(lun, entries_lba * gpt512, count * size / 512, entries)
		             : read_retrying(entries_lba * gpt512, count * size / 512, entries))) {
			printf("emmc: the GPT's entries: %u of %u bytes\n", count, size);
			return false;
		}
		for (uint32_t i = 0; i < count; i++) {
			const uint8_t *e = (const uint8_t *)entries + i * size;
			size_t k;
			uint64_t first, last;

			for (k = 0; EMMC_PART[k] && e[56 + 2 * k] == (uint8_t)EMMC_PART[k] && e[57 + 2 * k] == 0; k++) {
			}
			if (EMMC_PART[k] != 0 || e[56 + 2 * k] != 0 || e[57 + 2 * k] != 0) {
				continue;
			}
			memcpy(&first, e + 32, 8);
			memcpy(&last, e + 40, 8);
			if (first == 0 || last < first) {
				return false;
			}
			emmc_lba = first * gpt512;
			emmc_blocks = (last - first + 1) * gpt512;
			ufs_lun = lun;                  /* on the card it stays zero, unused */
			break;
		}
		if (emmc_lba != 0) {
			break;
		}
		if (ufs_on) {
			continue;                       /* this LUN's GPT has no xnusys: the next */
		}
		printf("emmc: no " EMMC_PART " partition\n");
		return false;
	}
	if (emmc_lba == 0) {
		printf("emmc: no " EMMC_PART " partition%s\n", ufs_on ? " on any LUN" : "");
		return false;
	}
	/* as long as its APFS containers, back to back, as the loader measured it for md0 */
	for (at = 0; at < emmc_blocks * 512;) {
		uint32_t bs;
		uint64_t n;

		if (!read_root(emmc_lba + at / 512, 1, sector) || sector[8] != 0x4253584eu) {   /* "NXSB" */
			break;
		}
		bs = sector[9];
		memcpy(&n, s + 40, 8);
		if (bs < 512 || bs > 65536 || n == 0 || n * bs > emmc_blocks * 512 - at) {
			break;
		}
		at += n * bs;
	}
	if (at == 0) {
		printf("emmc: no APFS container in " EMMC_PART " (tools/system-flash.sh writes it)\n");
		return false;
	}
	printf("emmc: " EMMC_PART " at block %llu, %llu MB of %llu MB in its APFS containers\n", emmc_lba, at >> 20,
	    (emmc_blocks * 512) >> 20);
	emmc_blocks = at / 512;
	return true;
}

/* ---- the devices ---- */

static int
emmc_open(__unused dev_t dev, int flags, __unused int devtype, __unused proc_t p)
{
	return (flags & FWRITE) ? EROFS : 0;
}

static int
emmc_close(__unused dev_t dev, __unused int flags, __unused int devtype, __unused proc_t p)
{
	return 0;
}

static void
emmc_strategy(buf_t bp)
{
	uint64_t block = (uint64_t)buf_blkno(bp);
	uint32_t count = buf_count(bp), done = 0;
	uint8_t *data = (uint8_t *)buf_dataptr(bp);
	int error = 0;

	if (!(buf_flags(bp) & B_READ)) {
		error = EROFS;                      /* this device never writes */
		goto out;
	}
	if (block >= emmc_blocks) {
		buf_setresid(bp, count);
		error = block == emmc_blocks ? 0 : EINVAL;
		goto out;
	}
	if (count > (emmc_blocks - block) * 512) {
		count = (uint32_t)((emmc_blocks - block) * 512);
	}
	if (ufs_on) {
		/* no lock around the UFS path: its slots are its own, and the pager's askers --
		 * many threads of one page each -- are exactly the depth the line needs; the
		 * episode under the recovery lock is the only serialization a failure costs */
		uint32_t blocks = count / 512;

		if (blocks > 0 && !ufs_read_bytes_r(emmc_lba + block, blocks, (uint32_t *)(void *)data)) {
			error = EIO;
		} else {
			done = blocks * 512;
			if (done < count) {         /* a part block at the end: one whole sector read, its tail taken */
				uint8_t *one = kalloc_data(512, Z_WAITOK);

				if (one == NULL) {
					error = ENOMEM;
				} else if (ufs_read_bytes_r(emmc_lba + block + blocks, 1, (uint32_t *)(void *)one)) {
					memcpy(data + done, one, count - done);
					done = count;
					kfree_data(one, 512);
				} else {
					kfree_data(one, 512);
					error = EIO;
				}
			}
		}
	} else {
		lck_mtx_lock(&emmc_lock);
		while (done < count && error == 0) {
			uint32_t blocks = (count - done) / 512;

			if (blocks == 0 || ((uintptr_t)(data + done) & 3)) {
				/* a part block, or a buffer not 4-aligned: one block at a time through the sector buffer */
				uint32_t n = count - done < 512 ? count - done : 512;

				if (!read_retrying(emmc_lba + block + done / 512, 1, sector)) {
					error = EIO;
					break;
				}
				memcpy(data + done, sector, n);
				done += n;
				continue;
			}
			if (blocks > EMMC_MAX_BLOCKS) {
				blocks = EMMC_MAX_BLOCKS;
			}
			if (!read_retrying(emmc_lba + block + done / 512, blocks, (uint32_t *)(void *)(data + done))) {
				error = EIO;
				break;
			}
			emmc_reads++;
			emmc_read_blocks += blocks;
			done += blocks * 512;
		}
		lck_mtx_unlock(&emmc_lock);
	}
	buf_setresid(bp, buf_count(bp) - done);
	if (error) {
		printf("emmc: block %llu, %u bytes: %s (%#x)\n", block, count, emmc_failed, emmc_failed_status);
	}
out:
	if (error) {
		buf_seterror(bp, error);
	}
	buf_biodone(bp);
}

static int
emmc_ioctl(__unused dev_t dev, u_long cmd, caddr_t data, __unused int fflag, __unused proc_t p)
{
	switch (cmd) {
	case DKIOCGETBLOCKSIZE:
		*(uint32_t *)data = 512;
		return 0;
	case DKIOCSETBLOCKSIZE:
		/* (spec_open sets 512 and back again, and takes a refusal for a broken device) */
		return *(uint32_t *)data == 512 ? 0 : EINVAL;
	case DKIOCGETBLOCKCOUNT:
		*(uint64_t *)data = emmc_blocks;
		return 0;
	case DKIOCISWRITABLE:
		*(uint32_t *)data = 0;
		return 0;
	case DKIOCGETMAXBLOCKCOUNTREAD:
		*(uint64_t *)data = EMMC_MAX_BLOCKS;
		return 0;
	default:
		return ENOTTY;
	}
}

static int
emmc_size(__unused dev_t dev)
{
	return 512;
}

static int
emmc_read(dev_t dev, struct uio *uio, __unused int ioflag)
{
	return physio(emmc_strategy, NULL, dev, B_READ, (u_int (*)(buf_t))emmc_size, uio, 512);
}

static const struct bdevsw emmc_bdevsw = {
	.d_open     = emmc_open,
	.d_close    = emmc_close,
	.d_strategy = emmc_strategy,
	.d_ioctl    = emmc_ioctl,
	.d_dump     = eno_dump,
	.d_psize    = emmc_size,
	.d_type     = D_DISK,
};

static const struct cdevsw emmc_cdevsw = {
	.d_open       = emmc_open,
	.d_close      = emmc_close,
	.d_read       = emmc_read,
	.d_write      = eno_rdwrt,
	.d_ioctl      = emmc_ioctl,
	.d_stop       = eno_stop,
	.d_reset      = eno_reset,
	.d_ttys       = NULL,
	.d_select     = eno_select,
	.d_mmap       = eno_mmap,
	.d_strategy   = eno_strat,
	.d_reserved_1 = eno_getc,
	.d_reserved_2 = eno_putc,
	.d_type       = D_DISK,
};

int
redwood_emmc_root(dev_t *root)
{
	uint64_t t0 = mach_absolute_time(), ns;
	uint32_t *probe;

	if (emmc_bmajor >= 0) {
		*root = makedev(emmc_bmajor, 0);
		return 0;
	}
	msdc = ml_io_map(MSDC0_PHYS, 0x1000);
	if (!find_partition()) {
		/* no card answered, and on rosemary every MSDC command times out instead: its storage
		 * is the UFS device behind the controller the loader left running */
		if (!ufs_start() || !find_partition()) {
			return ENXIO;
		}
	}
	/* how fast: the first 4 MB, which the root's mount reads anyway */
	probe = kalloc_data(EMMC_MAX_BLOCKS * 512, Z_WAITOK);
	if (probe != NULL) {
		for (uint32_t at = 0; at < 4 * 2048 && at + EMMC_MAX_BLOCKS <= emmc_blocks; at += EMMC_MAX_BLOCKS) {
			if (!read_root(emmc_lba + at, EMMC_MAX_BLOCKS, probe)) {
				break;
			}
		}
		absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
		printf("emmc: 4 MB in %llu ms\n", ns / 1000000);
		kfree_data(probe, EMMC_MAX_BLOCKS * 512);
	}
	emmc_bmajor = bdevsw_add(-1, &emmc_bdevsw);
	if (emmc_bmajor < 0 || (emmc_cmajor = cdevsw_add_with_bdev(-1, &emmc_cdevsw, emmc_bmajor)) < 0) {
		printf("emmc: no room among the devices (%d, %d)\n", emmc_bmajor, emmc_cmajor);
		return ENXIO;
	}
	*root = makedev(emmc_bmajor, 0);
	return 0;
}

void
redwood_emmc_devfs(void)
{
	if (emmc_bmajor < 0 || emmc_cmajor < 0) {
		return;
	}
	if (devfs_make_node(makedev(emmc_bmajor, 0), DEVFS_BLOCK, UID_ROOT, GID_OPERATOR, 0640, "emmc0") == NULL ||
	    devfs_make_node(makedev(emmc_cmajor, 0), DEVFS_CHAR, UID_ROOT, GID_OPERATOR, 0640, "remmc0") == NULL) {
		printf("emmc: no /dev nodes\n");
	}
}

/* sysctl kern.redwood.emmc: how much it has read, and its errors */
static int
emmc_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char line[320];

	snprintf(line, sizeof(line),
	    "emmc: %llu reads, %llu MB, %llu errors%s%s; ufs: %llu MB in %llu commands, %u most in flight, "
	    "%llu keepalives (%llu failed), %llu episodes, %llu cold starts",
	    emmc_reads, emmc_read_blocks >> 11, emmc_errors, emmc_failed ? "; last: " : "",
	    emmc_failed ? emmc_failed : "", ufs_read_bytes_total >> 20, ufs_reads, ufs_most_in_flight,
	    ufs_keepalives, ufs_keepalive_fails, ufs_episodes, ufs_cold_starts);
	return SYSCTL_OUT(req, line, strlen(line) + 1);
}

SYSCTL_DECL(_kern_redwood);
SYSCTL_PROC(_kern_redwood, OID_AUTO, emmc, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0, emmc_sysctl, "A",
    "the eMMC's reads so far");
