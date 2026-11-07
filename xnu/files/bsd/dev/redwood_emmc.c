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
 * refused. One request at a time, under a mutex, in pieces of EMMC_MAX_BLOCKS.
 */
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/locks.h>
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
 * One command at a time, slot 0, polled: nothing in XNU serves this controller's interrupts,
 * so IE stays zero throughout and a command is finished when its doorbell bit clears -- the IS
 * is read only to say what went wrong. */

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

/* one page of descriptors: the UTRD at its start (the list base wants 1 KB), the command
 * descriptor a kilobyte in -- its command UPIU, its response, and one PRD entry, for the
 * bounce is one physical piece and one entry covers any read */
#define UFS_UCD_OFF       0x400
#define UFS_CMD_OFF       (UFS_UCD_OFF + 0)
#define UFS_RSP_OFF       (UFS_UCD_OFF + 512)
#define UFS_PRD_OFF       (UFS_UCD_OFF + 1024)

static vm_offset_t ufs;
static bool ufs_on;                          /* reads go through the transport */
static uint8_t ufs_lun;                      /* the LUN the partition was found on */
static vm_offset_t ufs_desc, ufs_bounce;     /* that page, and EMMC_MAX_BLOCKS * 512 to read through */
static uint64_t ufs_desc_pa, ufs_bounce_pa;
static bool ufs_stuck;                       /* the doorbell never cleared: the list needs rebuilding */
static bool ufs_answered;                    /* the device answered, even to refuse: the link is fine */
static bool ufs_parked;                      /* the link sits in hibernation between reads: never idle awake */

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

/* the link's own state machine, through MediaTek's window: 1 is idle, 8 through 0xd are the
 * states of entering hibernation (Linux waits the same window out before it touches the timer) */
static uint32_t
ufs_sm(void)
{
	ufs_wr(UFS_DBG_SEL, 0x20);
	return ufs_rd(UFS_PROBE) & 0x1f;
}

/* the link back into hibernation the moment a read is done: whatever fires at ~4 ms of idle
 * (ufs3: UECPA 0x80000010, a line reset, with every named power saving already off) must
 * never find the link awake and unattended */
static void
ufs_park(void)
{
	if (ufs_uic(UFS_UIC_HIBER_ENTER, 0, 0, "parking the link")) {
		ufs_parked = true;
	}
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
	ufs_wr(UFS_UTRLBA_L, (uint32_t)ufs_desc_pa);
	ufs_wr(UFS_UTRLBA_H, (uint32_t)(ufs_desc_pa >> 32));
	ufs_wr(UFS_UTRLRSR, 1);
	for (uint32_t i = 0; i < 100000 && !(ufs_rd(UFS_HCS) & UFS_HCS_UTRLRDY); i++) {
	}
	if (!(ufs_rd(UFS_HCS) & UFS_HCS_UTRLRDY)) {
		return fail("transfer list", ufs_rd(UFS_HCS));
	}
	return true;
}

/* the fallback: the controller from cold, as the loader brings it up -- HCE off and on, the
 * link started again with a DME command -- then the list */
static bool
ufs_reset(void)
{
	ufs_wr(UFS_HCE, 0);
	for (uint32_t i = 0; i < 2000000 && (ufs_rd(UFS_HCE) & 1); i++) {
	}
	delay(5000);
	ufs_wr(UFS_HCE, 1);
	for (uint32_t i = 0; i < 20000000 && !(ufs_rd(UFS_HCE) & 1); i++) {
	}
	ufs_wr(UFS_IE, 0);                       /* again: nothing in XNU would serve it */
	if (!(ufs_rd(UFS_HCE) & 1)) {
		printf("ufs: the controller would not enable (HCE %#x)\n", ufs_rd(UFS_HCE));
		return fail("controller enable", ufs_rd(UFS_HCE));
	}
	ufs_wr(UFS_AHIT, 0);                     /* a cold start re-arms it: the same death would wait again */
	if (!ufs_uic(UFS_UIC_LINK_STARTUP, 0, 0, "link startup")) {
		return false;
	}
	if (!(ufs_rd(UFS_HCS) & UFS_HCS_DP)) {
		printf("ufs: no device after the startup (HCS %#x)\n", ufs_rd(UFS_HCS));
		return fail("no device", ufs_rd(UFS_HCS));
	}
	return ufs_list();
}

/* one READ10, in the LUN's 4096-byte blocks, into the bounce: the UTRD and the UPIU are built
 * where the page has them, both in the byte order each is read in (the UTRD little endian, the
 * UPIU big endian, byte by byte) */
static bool
ufs_read4k(uint8_t lun, uint64_t lba, uint32_t blocks)
{
	uint64_t ucd_pa = ufs_desc_pa + UFS_UCD_OFF;
	uint32_t bytes = blocks * 4096;
	uint32_t *d = (uint32_t *)ufs_desc;
	uint32_t *p = (uint32_t *)(ufs_desc + UFS_PRD_OFF);
	uint8_t *c = (uint8_t *)ufs_desc + UFS_CMD_OFF;
	uint8_t *r = (uint8_t *)ufs_desc + UFS_RSP_OFF;
	uint32_t is, ocs, resp, i;

	if (blocks == 0 || blocks > EMMC_MAX_BLOCKS * 512 / 4096) {
		return fail("too many blocks", blocks);
	}
	bzero((void *)ufs_desc, UFS_PRD_OFF + 16);
	d[0] = 0x15000000u;                      /* device to host, the command-type bit the loader's driver sets, interrupt command */
	d[2] = 0x0000000fu;                      /* OCS: invalid until the controller writes its zero */
	d[4] = (uint32_t)ucd_pa;                 /* the command descriptor, both halves little endian */
	d[5] = (uint32_t)(ucd_pa >> 32);
	d[6] = (128u << 16) | 128u;              /* its response UPIU: 128 words long, at word 128 */
	d[7] = (256u << 16) | 1u;                /* one PRD entry, at word 256 */
	c[0] = 0x01;                             /* transaction: command */
	c[1] = 0x40;                             /* flags: read */
	c[2] = lun;
	c[3] = 0;                                /* task tag 0, the slot the doorbell rings */
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
	p[0] = (uint32_t)ufs_bounce_pa;          /* where the data goes: one piece, so one entry */
	p[1] = (uint32_t)(ufs_bounce_pa >> 32);
	p[3] = bytes - 1;                        /* PRD entries count one less, as they do */

	ufs_clean(ufs_desc, UFS_PRD_OFF + 16);   /* what the controller reads */
	ufs_clean(ufs_bounce, bytes);            /* where it writes: no stale line of ours */
	is = ufs_rd(UFS_IS);
	if (is) {
		ufs_wr(UFS_IS, is);
	}
	ufs_wr(UFS_UTRLDBR, 1);                  /* slot 0: run */
	for (i = 0; i < 20000000; i++) {         /* the loader's driver gives a command five seconds */
		if (!(ufs_rd(UFS_UTRLDBR) & 1)) {
			break;
		}
	}
	if (ufs_rd(UFS_UTRLDBR) & 1) {
		ufs_stuck = true;
		ufs_answered = false;
		printf("ufs: LUN %u, %u blocks at %llu: the doorbell never cleared (IS %#x, MTK %#x)\n", lun, blocks, lba,
		    ufs_rd(UFS_IS), ufs_rd(UFS_OCS_ERR));
		return fail("doorbell stuck", ufs_rd(UFS_IS));
	}
	ufs_stuck = false;
	ocs = d[2] & 0xf;
	resp = (*(uint32_t *)(void *)(r + 4) >> 16) & 0xffff;    /* response and status, big endian where they sit */
	ufs_answered = ocs == 0;                 /* the device took the command, even to refuse it */
	if (ufs_answered) {
		ufs_park();                      /* straight back into hibernation: never idle, always parked */
	}
	if (ocs != 0 || resp != 0) {
		printf("ufs: LUN %u, %u blocks at %llu: OCS %u, response %#x, sense %#x %#x %#x, MTK %#x, HCS %#x, RSR %#x\n", lun, blocks, lba,
		    ocs, resp, r[36], r[46], r[47], ufs_rd(UFS_OCS_ERR), ufs_rd(UFS_HCS), ufs_rd(UFS_UTRLRSR));
		printf("ufs: the aftermath: IS %#x, UECPA %#x, UECDL %#x, UECN %#x, UECT %#x, sm %u, refclk %#x, mphy %#x, stall %#x\n",
		    ufs_rd(UFS_IS), ufs_rd(UFS_UECPA), ufs_rd(UFS_UECDL), ufs_rd(UFS_UECN), ufs_rd(UFS_UECT), ufs_sm(),
		    ufs_rd(UFS_REFCLK), ufs_rd(UFS_MPHY), ufs_rd(UFS_STALL));
		return fail("command status", (ocs << 16) | resp);
	}
	return true;
}

/* one READ10 with the ladder behind it: plainly again if the command merely failed, then the
 * link woken and the list rebuilt (a link left hibernated by the loader looks exactly like a
 * dead one), then hibernation in and out with the link started warm (the rung the cold reset
 * cannot replace, for it loses the mphy setup the preloader left), then the controller from cold */
static bool
ufs_read4k_retry(uint8_t lun, uint64_t lba, uint32_t blocks)
{
	if (ufs_parked) {
		ufs_parked = false;               /* woken or not, every rung of the ladder below stands ready */
		if (!ufs_uic(UFS_UIC_HIBER_EXIT, 0, 0, "waking the parked link")) {
			printf("ufs: the parked link would not wake (sm %u); the ladder starts\n", ufs_sm());
		}
	}
	if (ufs_read4k(lun, lba, blocks)) {
		return true;
	}
	if (ufs_answered) {
		return false;                       /* the link is fine: the device refused this command */
	}
	if (!ufs_stuck) {
		emmc_errors++;
		if (ufs_read4k(lun, lba, blocks)) {
			return true;
		}
		if (ufs_answered) {
			return false;
		}
	}
	emmc_errors++;
	printf("ufs: %s (%#x); waking the link and rebuilding the list (sm %u)\n", emmc_failed, emmc_failed_status, ufs_sm());
	(void)ufs_uic(UFS_UIC_HIBER_EXIT, 0, 0, "waking the link");
	if (ufs_list() && ufs_read4k(lun, lba, blocks)) {
		return true;
	}
	printf("ufs: hibernation in and out, then the link started warm\n");
	(void)ufs_uic(UFS_UIC_HIBER_ENTER, 0, 0, "entering hibernation");
	(void)ufs_uic(UFS_UIC_HIBER_EXIT, 0, 0, "leaving hibernation");
	if (ufs_uic(UFS_UIC_LINK_STARTUP, 0, 0, "starting the link warm") && (ufs_rd(UFS_HCS) & UFS_HCS_DP) &&
	    ufs_list() && ufs_read4k(lun, lba, blocks)) {
		return true;
	}
	printf("ufs: resetting the controller\n");
	if (ufs_reset() && ufs_read4k(lun, lba, blocks)) {
		return true;
	}
	return false;
}

/* the caller's 512-byte blocks out of the LUN's 4096-byte ones: each piece is read whole into
 * the bounce and the asked-for bytes copied out of it */
static bool
ufs_read_512(uint8_t lun, uint64_t lba, uint32_t blocks, uint32_t *dst)
{
	while (blocks) {
		uint64_t at = lba >> 3;              /* the 4096-byte block these bytes start in */
		uint32_t skip = (uint32_t)(lba & 7) * 512;
		uint32_t n4k = (skip + blocks * 512 + 4095) / 4096;
		uint32_t take = blocks * 512 < n4k * 4096 - skip ? blocks * 512 : n4k * 4096 - skip;

		if (n4k > EMMC_MAX_BLOCKS * 512 / 4096) {
			n4k = EMMC_MAX_BLOCKS * 512 / 4096;
			take = n4k * 4096 - skip;
		}
		if (!ufs_read4k_retry(lun, at, n4k)) {
			return false;
		}
		memcpy(dst, (void *)((uint8_t *)ufs_bounce + skip), take);
		dst = (uint32_t *)(void *)((uint8_t *)dst + take);
		lba += take / 512;
		blocks -= take / 512;
	}
	return true;
}

/* brings the transport up: LK has left the controller and the link running, it read the boot
 * image through them moments before, so this only takes the interrupts off, points the
 * transfer list at this side's page and reads once; the reset is the fallback */
static bool
ufs_start(void)
{
	vm_offset_t at = 0;
	uint32_t savepower = 0, dbgclk = 0;

	if (kmem_alloc_contig(kernel_map, &at, PAGE_SIZE, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) !=
	    KERN_SUCCESS) {
		printf("ufs: no page for the descriptors\n");
		return false;
	}
	ufs_desc = at;
	ufs_desc_pa = kvtophys(ufs_desc);
	at = 0;
	if (kmem_alloc_contig(kernel_map, &at, EMMC_MAX_BLOCKS * 512, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("ufs: no %u bytes in one piece for the reads\n", EMMC_MAX_BLOCKS * 512);
		return false;
	}
	ufs_bounce = at;
	ufs_bounce_pa = kvtophys(ufs_bounce);
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
	 * whole, its unnamed bits included, and every read parks the link in hibernation after) */
	(void)ufs_dme_get(UFS_VS_SAVEPOWER, &savepower, "reading SAVEPOWERCONTROL");
	(void)ufs_dme_get(UFS_VS_DBGCLK, &dbgclk, "reading DEBUGCLOCKENABLE");
	printf("ufs: unipro: savepower %#x, dbgclk %#x, sm %u; clearing every gate and stall\n", savepower, dbgclk, ufs_sm());
	(void)ufs_dme_set(UFS_VS_UNIPRO_PD, 0, "clearing the unipro low-power mode");
	(void)ufs_dme_set(UFS_VS_SAVEPOWER, 0, "clearing the whole of SAVEPOWERCONTROL");
	(void)ufs_dme_set(UFS_VS_DBGCLK, dbgclk | 0x20u, "forcing the TX symbol clock on");
	(void)ufs_dme_get(UFS_VS_SAVEPOWER, &savepower, "re-reading SAVEPOWERCONTROL");
	(void)ufs_dme_get(UFS_VS_DBGCLK, &dbgclk, "re-reading DEBUGCLOCKENABLE");
	printf("ufs: unipro now: savepower %#x, dbgclk %#x; reads park the link between them\n", savepower, dbgclk);
	/* the link LK left may be hibernated: the read below says so, and the ladder wakes it */
	/* one read of LUN 0 (a boot LUN, small but alive) says the link answers */
	if (!ufs_read4k_retry(0, 0, 1)) {
		printf("ufs: the first read failed; the transport is not up\n");
		return false;
	}
	ufs_on = true;
	printf("ufs: the transport is up\n");
	return true;
}

/* the same, tried again once after a failure (the controller cleared and the card told to stop) */
static bool
read_retrying(uint64_t lba, uint32_t blocks, uint32_t *dst)
{
	if (ufs_on) {
		return ufs_read_512(ufs_lun, lba, blocks, dst);     /* its retries and its ladder are its own */
	}
	if (read_blocks(lba, blocks, dst)) {
		return true;
	}
	emmc_errors++;
	printf("emmc: reading %u blocks at %llu: %s (%#x); trying again\n", blocks, lba, emmc_failed, emmc_failed_status);
	pio_mode();
	(void)stop();
	return read_blocks(lba, blocks, dst);
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
			if ((ufs_on ? ufs_read_512(lun, gs / 512, 1, sector) : read_retrying(gs / 512, 1, sector)) &&
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
		    !(ufs_on ? ufs_read_512(lun, entries_lba * gpt512, count * size / 512, entries)
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

		if (!read_retrying(emmc_lba + at / 512, 1, sector) || sector[8] != 0x4253584eu) {   /* "NXSB" */
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
			if (!read_retrying(emmc_lba + at, EMMC_MAX_BLOCKS, probe)) {
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
	char line[160];

	snprintf(line, sizeof(line), "emmc: %llu reads, %llu MB, %llu errors%s%s", emmc_reads, emmc_read_blocks >> 11,
	    emmc_errors, emmc_failed ? "; last: " : "", emmc_failed ? emmc_failed : "");
	return SYSCTL_OUT(req, line, strlen(line) + 1);
}

SYSCTL_DECL(_kern_redwood);
SYSCTL_PROC(_kern_redwood, OID_AUTO, emmc, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0, emmc_sysctl, "A",
    "the eMMC's reads so far");
