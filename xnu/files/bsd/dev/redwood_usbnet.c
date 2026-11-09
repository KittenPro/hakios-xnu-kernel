/*
 * The phone's own USB cable as the link behind en0 (see sys/redwood_net.h for what a link is).
 *
 * MT6785's SSUSB controller in device mode -- the same hardware Linux drives with the mtu3
 * driver, and the same recipe is followed here (registers, the endpoint FIFO map, the QMU DMA
 * engine and its 16-byte GPD rings with checksums, the chapter 9 state machine of EP0).
 * What rides on it is RNDIS, the way Project Sandcastle does it: the frames of en0 wrapped in
 * a 44-byte header, control messages encapsulated over EP0, the whole thing seen by a host
 * (Linux's rndis_host, macOS, Windows) as a CDC-ACM-ish ethernet adapter. One cable, live
 * logs over IP, nothing else to invent.
 *
 * Only with boot-arg usb_step=N:
 *   1 -- clocks, the PHY, the device powered on and SOFT_CONN raised: enough to see the speed
 *        the PHY settles at in the log, and nothing else;
 *   2 -- all of that, and chapter 9 answered, RNDIS handled, bulk endpoints running over the
 *        QMU, and the link attached to en0 (which stays down until the host sets RNDIS's
 *        packet filter: the cable is not a carrier until the host says so).
 * Boot-arg usb_trace=1 prints every EP0 setup packet and every RNDIS message.
 *
 * No interrupt is used: a thread of our own polls the bus, EP0 and the receive ring once a
 * millisecond (the controller is a phone peripheral on a boot experiment; one ms is nothing).
 * The QMU's DMA is not cache-coherent, so everything it reads is flushed and everything it
 * wrote is flushed again before it is read (the dance redwood_ufshc.c does).
 *
 * Who owns what: the thread owns EP0, the RX ring and the notification endpoint; transmit()
 * (called by the interface's output path, one thread at a time) owns the TX ring, reaping
 * what the hardware finished and arming the next GPD; usbnet_lock keeps transmit() and the
 * thread's configure/teardown (SET_CONFIG, bus reset) from stepping on each other. Nothing
 * of redwood_net.c is ever called with usbnet_lock held.
 */

#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <pexpert/pexpert.h>
#include <sys/kpi_mbuf.h>
#include <sys/redwood_net.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern uint64_t kvtophys(vm_offset_t va);       /* pmap_paddr_t */
extern void FlushPoC_DcacheRegion(vm_offset_t va, size_t length);
extern void delay_for_interval(uint32_t interval, uint32_t scale_factor);

/*
 * Where the hardware is. The MAC registers, the IP block's own power/clock control (IPPC),
 * the PHY's register banks (the "SIF" window; the phone's phy is the V2 layout: the U2
 * common registers at +0x300, the U3 PHYA bank at +0xb00), and INFRACFG-AO for the gates.
 */
#define USB_MAC_PHYS    0x11201000u
#define USB_MAC_SIZE    0x3000
#define USB_IPPC_PHYS   0x11203E00u
#define USB_SIF_PHYS    0x11E40000u
#define INFRA_PHYS      0x10001000u
#define TOPCLK_PHYS     0x10000000u     /* the top clocks: usb_top_sel, [9:8] of CK_CFG_8 */
#define APMIXED_PHYS    0x1000C000u     /* the PLLs and the 26 MHz gates: SSUSB26M is bit 4 of +0x20 */
#define REG_CKCFG8      0x0a0           /* usb_top_sel here: 0 clk26m / 1..3 univpll divisions, all alive */
#define REG_APMIXED_CG  0x020           /* write-one-to-set, read for status */
#define APMIXED_SSUSB26M (1u << 4)      /* the device's "ref_ck": 26 MHz for the SSUSB */

/* the two SSUSB clock gates of INFRA2: a power-down bank (clk-mt6785.c's INFRA_PDN_SET2/CLR2/STA2,
 * +0xa4/+0xa8/+0xac) -- a bit SET is the clock OFF; mtk_clk_gate_ops_setclr's enable writes the
 * bit to the CLR bank. The connsys driver ungates CCIF4 the same way (INFRA3_CLR, +0xc4). */
#define INFRA2_CG_CLR   0x0a8
#define INFRA2_CG_STA   0x0ac
#define INFRA_USB       (1u << 1)
#define INFRA_SSUSB_XHCI (1u << 31)

/* the PHY, SIF-relative */
#define PHY_PHYA_REG1   0xb04            /* VUSB10 on, bit 29 */
#define PHY_ACR6        0x318            /* OTG_VBUSCMP_EN, bit 20 */
#define PHY_ACR4        0x320            /* RG_USB20_* bits 18..16, 9, 8 */
#define PHY_DTM0        0x368            /* RG_UART_MODE 30, FORCE_UART_EN 26, FORCE_SUSPENDM 18 */
#define PHY_DTM1        0x36c            /* UART_EN 16; FORCE bits 13/12/10 enable the RG overrides:
                                         * RG_VBUSVALID 5, RG_SESSEND 4, RG_AVALID 2 are the values */

/* IPPC, the IP block's own power and reset */
#define IPPC_PW_CTRL0   0x00             /* bit 0 SSUSB_IP_SW_RST: pulse to reset the whole IP */
#define IPPC_PW_CTRL2   0x08             /* bit 0 DEV_PDN: clear to power on */
#define IPPC_PW_STS1    0x10             /* bit 0x400: the 125M clock stable */
#define IPPC_PW_STS2    0x14             /* bit 0x1 */
#define IPPC_U2_CTRL    0x50             /* DIS 0, PDN 1, HOST 2, OTG_SEL 7 */
#define IPPC_DEV_RST    0x98             /* bit 0 DEV_SW_RST: pulse, after power */

/*
 * The MAC registers (mtu3's mtu3_hw_regs.h, SSUSB_DEV_BASE-relative). EP0 and the endpoint
 * CSRs, the QMU, and the device-side registers at 0x0800+.
 */
#define REG_EP0CSR      0x0100           /* W1C below; the rest are commands */
#define EP0_MAXPKTSZ    0x03ff
#define EP0_RXPKTRDY    (1u << 16)       /* these three: write 1 to clear */
#define EP0_SETUPPKTRDY (1u << 17)
#define EP0_TXPKTRDY    (1u << 18)
#define EP0_DATAEND     (1u << 19)
#define EP0_DPHTX       (1u << 20)
#define EP0_SENTSTALL   (1u << 22)
#define EP0_FIFOFULL    (1u << 23)
#define EP0_SENDSTALL   (1u << 25)
#define EP0_W1C_BITS    ~(EP0_SETUPPKTRDY | EP0_RXPKTRDY | EP0_SENTSTALL)
#define REG_RXCOUNT0    0x0108
#define REG_FIFO0       0x0300
#define REG_TX1CSR0     0x0110           /* +0x10 per endpoint number */
#define REG_TX1CSR1     0x0114
#define REG_TX1CSR2     0x0118
#define REG_RX1CSR0     0x0210
#define REG_RX1CSR1     0x0214
#define REG_RX1CSR2     0x0218
#define CSR_DMAREQEN    (1u << 29)
#define CSR_MAXPKTSZ    0x07ff
#define CSR_TYPE        (3u << 4)        /* bulk 0, interrupt 1 */
#define CSR_SLOT        (0x3fu << 8)     /* bulk 1, interrupt 0 */
#define CSR_FIFOADDR    0x1fff           /* the address, >>4 */
#define CSR_FIFOSEG     (0xfu << 16)     /* ilog2 of the segment: 512 bytes -> 9 */

#define REG_QCR0        0x0400           /* CS16B 0, TX checksum-enable bit n, RX bit 16+n */
#define REG_QCR1        0x0404           /* TX ZLP-enable, bit n */
#define REG_QCR3        0x040c           /* RX "move on on a ZLP", bit 16+n */
#define REG_TXQCSR1     0x0510           /* +0x10 per endpoint number */
#define REG_TXQSAR1     0x0514
#define REG_RXQCSR1     0x0610
#define REG_RXQSAR1     0x0614
#define Q_START         (1u << 0)
#define Q_RESUME        (1u << 1)
#define Q_STOP          (1u << 2)
#define Q_ACTIVE        (1u << 15)

#define REG_DEVICE_CONF 0x0800
#define DEV_ADDR_MSK    0x7f000000u      /* GENMASK(30,24), the address <<24 */
#define DEV_SPEED       0x7              /* 0 inactive, 1 full, 3 high */
#define REG_EP_RST      0x0804           /* ((in? 1<<16 : 1) << n): set, then clear */
#define REG_DEV_LINK_INTR 0x0854         /* W1C */
#define REG_CAP_TXFFSZ  0x0c08
#define REG_CAP_RXFFSZ  0x0c0c
#define REG_CAP_EPINFO  0x0c10
#define REG_MISC_CTRL   0x0c84           /* VBUS_FRC_EN 0, VBUS_ON 1 */
#define REG_USB3_CONFIG 0x141c           /* USB3_EN 0: this is a high-speed-only device */
#define REG_LTSSM_INTR  0x1540           /* W1C */
#define REG_POWER_MGMT  0x2404           /* SUSPENDM 0, HS_ENABLE 5, SOFT_CONN 6 */
#define REG_USB_INTR_EN 0x2418
#define REG_USB_INTR    0x241c           /* W1C: RESET 2, DISCONN 5 */
#define INTR_RESET      (1u << 2)
#define INTR_DISCONN    (1u << 5)
#define REG_LV1IECR     0x000c           /* write 1 to disable the level-1 interrupts */
#define REG_EPISR       0x0080
#define REG_EPIECR      0x008c

/* the endpoints this function uses */
#define EP_BULK_IN      1                /* frames to the host, TX endpoint 1 */
#define EP_BULK_OUT     1                /* frames from the host, RX endpoint 1 */
#define EP_NOTIFY       2                /* "a response is available", TX endpoint 2 */
#define REG_TX2CSR0     (REG_TX1CSR0 + 0x10)
#define REG_TX2CSR1     (REG_TX1CSR1 + 0x10)
#define REG_TX2CSR2     (REG_TX1CSR2 + 0x10)
#define REG_TXQCSR2     (REG_TXQCSR1 + 0x10)
#define REG_TXQSAR2     (REG_TXQSAR1 + 0x10)

/*
 * The QMU's GPD: 16 bytes, little-endian, with a checksum such that all sixteen bytes add up
 * to 0xff (hardware owns it once HWO is set; completion is HWO cleared).
 */
struct usbnet_gpd {
	uint8_t         flag;           /* HWO 1, IOC 0x80 */
	uint8_t         chksum;
	uint16_t        data_buf_len;   /* receive: the capacity offered */
	uint32_t        next_gpd;       /* physical, the ring is a ring */
	uint32_t        buffer;         /* physical */
	uint16_t        buf_len;        /* transmit: to send; receive: what came (hardware) */
	uint8_t         ext_len;
	uint8_t         ext_flag;       /* ZLP 0x20 */
} __attribute__((packed));

#define GPD_HWO         0x01
#define GPD_IOC         0x80
#define GPD_ZLP         0x20

#define RING_DEPTH      32              /* GPDs per ring; one stays spare (never more than 31 owned by us) */
#define BUFFER_SIZE     2048            /* 44 + a frame, and never crosses a page */

/*
 * Chapter 9 and RNDIS, as f_rndis.c has them: VID:PID of the canonical gadget, a CDC-ACM
 * shaped pair of interfaces under an IAD, RNDIS's messages encapsulated on EP0. The host
 * picks the configuration apart from these bytes; the endpoints are where they say.
 */
static const uint8_t device_descriptor[18] = {
	0x12, 0x01, 0x00, 0x02, 0xef, 0x02, 0x01, 0x40,
	0x25, 0x05, 0xa2, 0xa4, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01
};
static const uint8_t qualifier_descriptor[10] = {
	0x0a, 0x06, 0x00, 0x02, 0xef, 0x02, 0x01, 0x40, 0x01, 0x00
};
static const uint8_t langid_descriptor[4] = { 0x04, 0x03, 0x09, 0x04 };

static const uint8_t config_fs[75] = {
	0x09, 0x02, 0x4b, 0x00, 0x02, 0x01, 0x00, 0x80, 0xfa,
	0x08, 0x0b, 0x00, 0x02, 0x02, 0x06, 0x00, 0x00,
	0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0xff, 0x00,
	0x05, 0x24, 0x00, 0x10, 0x11,
	0x05, 0x24, 0x01, 0x00, 0x01,
	0x04, 0x24, 0x02, 0x00,
	0x05, 0x24, 0x06, 0x00, 0x01,
	0x07, 0x05, 0x82, 0x03, 0x08, 0x00, 0x20,
	0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00,
	0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,
	0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00
};
static const uint8_t config_hs[75] = {
	0x09, 0x02, 0x4b, 0x00, 0x02, 0x01, 0x00, 0x80, 0xfa,
	0x08, 0x0b, 0x00, 0x02, 0x02, 0x06, 0x00, 0x00,
	0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0xff, 0x00,
	0x05, 0x24, 0x00, 0x10, 0x11,
	0x05, 0x24, 0x01, 0x00, 0x01,
	0x04, 0x24, 0x02, 0x00,
	0x05, 0x24, 0x06, 0x00, 0x01,
	0x07, 0x05, 0x82, 0x03, 0x08, 0x00, 0x06,
	0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00,
	0x07, 0x05, 0x81, 0x02, 0x00, 0x02, 0x00,
	0x07, 0x05, 0x01, 0x02, 0x00, 0x02, 0x00
};

/* RNDIS message types (rndis.h); the OIDs rndis_host asks while binding */
#define RNDIS_MSG_PACKET       0x00000001u
#define RNDIS_MSG_INIT         0x00000002u
#define RNDIS_MSG_INIT_C       0x80000002u
#define RNDIS_MSG_HALT         0x00000003u
#define RNDIS_MSG_QUERY        0x00000004u
#define RNDIS_MSG_QUERY_C      0x80000004u
#define RNDIS_MSG_SET          0x00000005u
#define RNDIS_MSG_SET_C        0x80000005u
#define RNDIS_MSG_RESET        0x00000006u
#define RNDIS_MSG_RESET_C      0x80000006u
#define RNDIS_MSG_KEEPALIVE    0x00000008u
#define RNDIS_MSG_KEEPALIVE_C  0x80000008u
#define RNDIS_STATUS_SUCCESS   0x00000000u
#define RNDIS_STATUS_NOT_SUPPORTED 0xc00000bbu
#define OID_GEN_PHYSICAL_MEDIUM      0x00010202u
#define OID_802_3_PERMANENT_ADDRESS  0x01010101u
#define OID_802_3_CURRENT_ADDRESS    0x01010102u
#define OID_GEN_MAXIMUM_FRAME_SIZE   0x00010106u
#define OID_GEN_MEDIA_CONNECT_STATUS 0x00010114u
#define OID_GEN_PACKET_FILTER        0x0001010eu
#define RNDIS_HEADER_SIZE     44

static const uint8_t usbnet_address[6] = { 0x02, 0x72, 0x6f, 0x73, 0x65, 0x01 };
static const uint8_t notification_message[8] = { 0xa1, 0x01, 0, 0, 0, 0, 0, 0 };

/* the state of it all */
static uint32_t usb_step, usb_trace;
static vm_offset_t usb_mac, usb_ippc, usb_sif, usb_infra, usb_top, usb_apmixed;
static bool usbnet_on;                   /* step 2 got as far as starting the thread */
static bool usbnet_configured;           /* SET_CONFIG(1) done, the endpoints are the gadget's */
static uint8_t usbnet_address_set;       /* the last SET_ADDRESS value */
static bool usbnet_highspeed;            /* the bus speed, at configuration time */
static uint32_t usbnet_bulk_maxp;        /* 64 or 512 */
static uint32_t usbnet_rx_fifo_addr;     /* in bytes, for RX1CSR2 */

static LCK_GRP_DECLARE(usbnet_lock_group, "redwood-usbnet");
static LCK_MTX_DECLARE(usbnet_lock, &usbnet_lock_group);

/* the DMA memory: one page of GPD rings, then the buffers */
static struct usbnet_gpd *tx_ring, *rx_ring, *notify_ring;
static uint8_t *tx_buffers, *rx_buffers;
static vm_offset_t dma_rings, dma_buffers;

/* ring cursors: [head, enq) is what we have armed; the hardware parks on the first GPD without HWO */
static unsigned tx_head, tx_enq;
static unsigned rx_head, rx_enq;
static unsigned notify_pos;              /* the two slots alternate */

/* EP0's state machine (the thread's own; nothing else touches it) */
enum { EP0_SETUP, EP0_TX, EP0_RX };
static uint8_t ep0_state;
static uint8_t ep0_in[256];
static uint32_t ep0_in_len, ep0_in_pos;
static uint8_t ep0_out[512];
static uint32_t ep0_out_want, ep0_out_got;

/* the encapsulated response waiting for the host to read */
static uint8_t rndis_response[64];
static uint32_t rndis_response_len;

/* a frame on its way from the RX ring to the interface, clear of every lock */
static uint8_t rx_stage[BUFFER_SIZE];

/* which buffer each armed GPD owns, and the buffers not owned right now */
static unsigned tx_gpd_buf[RING_DEPTH], rx_gpd_buf[RING_DEPTH];
static unsigned tx_free[RING_DEPTH], rx_free[RING_DEPTH];
static unsigned tx_free_count, rx_free_count;

/* buffers' physical addresses, computed once and checked against the DMA window */
static uint32_t tx_buf_pa[RING_DEPTH], rx_buf_pa[RING_DEPTH];
static uint32_t notify_pa;
static vm_offset_t notify_va;            /* the 8-byte message, in the rings' page */
static uint32_t usbnet_fifo_limit;       /* in 512-byte units: TX's half of the FIFO */

static void usbnet_start(void);
static void usbnet_stop(void);

static inline uint32_t
mac_read(uint32_t offset)
{
	return *(volatile uint32_t *)(usb_mac + offset);
}

static inline void
mac_write(uint32_t offset, uint32_t value)
{
	*(volatile uint32_t *)(usb_mac + offset) = value;
}

static inline uint32_t
mac_set(uint32_t offset, uint32_t bits)  /* read, set, write */
{
	uint32_t value = mac_read(offset) | bits;

	mac_write(offset, value);
	return value;
}

static inline uint32_t
mac_clr(uint32_t offset, uint32_t bits)  /* read, clear, write */
{
	uint32_t value = mac_read(offset) & ~bits;

	mac_write(offset, value);
	return value;
}

static inline void
ippc_write(uint32_t offset, uint32_t value)
{
	*(volatile uint32_t *)(usb_ippc + offset) = value;
}

static inline uint32_t
ippc_read(uint32_t offset)
{
	return *(volatile uint32_t *)(usb_ippc + offset);
}

static inline void
sif_write(uint32_t offset, uint32_t value)
{
	*(volatile uint32_t *)(usb_sif + offset) = value;
}

static inline uint32_t
sif_read(uint32_t offset)
{
	return *(volatile uint32_t *)(usb_sif + offset);
}

static inline uint32_t
top_read(uint32_t offset)
{
	return *(volatile uint32_t *)(usb_top + offset);
}

static inline void
top_write(uint32_t offset, uint32_t value)
{
	*(volatile uint32_t *)(usb_top + offset) = value;
}

static inline uint32_t
apm_read(uint32_t offset)
{
	return *(volatile uint32_t *)(usb_apmixed + offset);
}

static inline void
apm_write(uint32_t offset, uint32_t value)
{
	*(volatile uint32_t *)(usb_apmixed + offset) = value;
}

/* everything the DMA engine is to read must leave the cache first (redwood_ufshc.c's dance) */
static inline void
dma_flush(vm_offset_t address, size_t length)
{
	FlushPoC_DcacheRegion(address, length);
	__asm__ volatile ("dsb sy" ::: "memory");
}

static void
ep0_fifo_write(const uint8_t *source, uint32_t length)
{
	vm_offset_t fifo = usb_mac + REG_FIFO0;
	uint32_t i = 0;

	if (length >= 4) {
		for (; i + 4 <= length; i += 4) {
			*(volatile uint32_t *)fifo = *(const uint32_t *)(source + i);
		}
	}
	if (length & 2) {
		*(volatile uint16_t *)fifo = *(const uint16_t *)(source + i);
		i += 2;
	}
	if (length & 1) {
		*(volatile uint8_t *)fifo = source[i];
	}
}

static void
ep0_fifo_read(uint8_t *destination, uint32_t length)
{
	vm_offset_t fifo = usb_mac + REG_FIFO0;
	uint32_t i = 0;

	if (length >= 4) {
		for (; i + 4 <= length; i += 4) {
			*(uint32_t *)(destination + i) = *(volatile uint32_t *)fifo;
		}
	}
	if (length & 2) {
		*(uint16_t *)(destination + i) = *(volatile uint16_t *)fifo;
		i += 2;
	}
	if (length & 1) {
		destination[i] = *(volatile uint8_t *)fifo;
	}
}

static uint8_t
gpd_checksum(struct usbnet_gpd *gpd)     /* all sixteen bytes add up to 0xff */
{
	uint8_t sum = 0;

	gpd->chksum = 0;
	for (unsigned i = 0; i < sizeof(*gpd); i++) {
		sum += ((uint8_t *)gpd)[i];
	}
	return 0xff - sum;
}

static void
gpd_arm(struct usbnet_gpd *gpd, uint32_t buffer_pa, uint16_t send_len, uint16_t capacity, bool zlp)
{
	gpd->buffer = buffer_pa;
	gpd->buf_len = send_len;
	gpd->data_buf_len = capacity;
	gpd->ext_len = 0;
	gpd->ext_flag = zlp ? GPD_ZLP : 0;
	gpd->flag = GPD_IOC | GPD_HWO;
	gpd->chksum = gpd_checksum(gpd);
}

/* n = 0: TX ring; n = 1: RX ring; the notify ring is its own two slots */
static uint32_t
ring_pa(int n)
{
	return (uint32_t)kvtophys(dma_rings + (n == 0 ? 0 : RING_DEPTH * sizeof(struct usbnet_gpd)));
}

static uint32_t
buffer_pa(const uint8_t *buffers, unsigned index)
{
	return (uint32_t)kvtophys((vm_offset_t)buffers + (size_t)index * BUFFER_SIZE);
}

static void
queue_resume(uint32_t qcsr)              /* nudge the QMU to look at the GPD just armed */
{
	if (mac_read(qcsr) & Q_ACTIVE) {
		mac_write(qcsr, Q_RESUME);
	} else {
		mac_write(qcsr, Q_START);
	}
}

/* the 8-byte RESPONSE_AVAILABLE on the interrupt endpoint; the host polls EP0 anyway */
static void
usbnet_notify(void)
{
	struct usbnet_gpd *gpd = &notify_ring[notify_pos];

	if (!usbnet_configured) {
		return;
	}
	dma_flush((vm_offset_t)gpd, sizeof(*gpd));
	if (gpd->flag & GPD_HWO) {
		return;                     /* the last one still in flight: let EP0 polling do it */
	}
	dma_flush(notify_va, sizeof(notification_message));
	gpd_arm(gpd, notify_pa, sizeof(notification_message), 0, false);
	dma_flush((vm_offset_t)gpd, sizeof(*gpd));
	queue_resume(REG_TXQCSR2);
	notify_pos ^= 1;
}

static void
put32(uint8_t *out, uint32_t value)
{
	out[0] = (uint8_t)value;
	out[1] = (uint8_t)(value >> 8);
	out[2] = (uint8_t)(value >> 16);
	out[3] = (uint8_t)(value >> 24);
}

static uint32_t
get32(const uint8_t *in)
{
	return (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
}

/*
 * One encapsulated RNDIS message, arrived over EP0. The reply (if any) is put where the next
 * GET_ENCAPSULATED will find it, and the host is nudged. Runs on the thread, no lock held.
 */
static void
rndis_process(uint8_t *message, uint32_t length)
{
	uint32_t type, request_id = 0, oid, n = 0, status = RNDIS_STATUS_SUCCESS;
	const uint8_t *data = NULL;
	uint8_t *out = rndis_response;
	static const uint8_t zero[4] = { 0, 0, 0, 0 };
	static const uint8_t frame_size[4] = { 0xea, 0x05, 0, 0 };   /* 1514 */

	if (length < 8) {
		return;
	}
	type = get32(message);
	if (type != RNDIS_MSG_RESET) {
		request_id = get32(message + 8);
	}
	if (usb_trace) {
		printf("usb: rndis message %u (%u bytes)\n", type, length);
	}

	switch (type) {
	case RNDIS_MSG_INIT:
		put32(out + 0, RNDIS_MSG_INIT_C);
		put32(out + 4, 52);         /* request id, status, major, minor, flags, medium, */
		put32(out + 8, request_id); /* packets per transfer, max transfer, alignment, */
		put32(out + 12, RNDIS_STATUS_SUCCESS);
		put32(out + 16, 1);
		put32(out + 20, 0);
		put32(out + 24, 1);         /* connectionless */
		put32(out + 28, 0);         /* 802.3 */
		put32(out + 32, 1);
		put32(out + 36, 1580);      /* what f_rndis answers: a frame and its headers, and room to spare */
		put32(out + 40, 0);
		put32(out + 44, 0);
		put32(out + 48, 0);
		rndis_response_len = 52;
		break;
	case RNDIS_MSG_QUERY:
		if (length < 24) {
			return;
		}
		oid = get32(message + 12);
		switch (oid) {
		case OID_GEN_PHYSICAL_MEDIUM:
		case OID_GEN_MAXIMUM_FRAME_SIZE:
		case OID_GEN_MEDIA_CONNECT_STATUS:
			n = 4;
			data = oid == OID_GEN_MAXIMUM_FRAME_SIZE ? frame_size : zero;
			break;
		case OID_802_3_PERMANENT_ADDRESS:
		case OID_802_3_CURRENT_ADDRESS:
			n = 6;
			data = usbnet_address;
			break;
		default:
			n = 0;
			data = NULL;
			status = RNDIS_STATUS_NOT_SUPPORTED;
			break;
		}
		put32(out + 0, RNDIS_MSG_QUERY_C);
		put32(out + 4, 24 + n);     /* exactly what was asked, no more */
		put32(out + 8, request_id);
		put32(out + 12, status);
		put32(out + 16, n);
		put32(out + 20, 16);        /* the payload, from request_id + 16 */
		if (n > 0) {
			memcpy(out + 24, data, n);
		}
		rndis_response_len = 24 + n;
		break;
	case RNDIS_MSG_SET:
		if (length < 24) {
			return;
		}
		oid = get32(message + 12);
		if (oid == OID_GEN_PACKET_FILTER) {
			uint32_t offset = 8 + get32(message + 20);       /* from request_id, the host says 16 */
			uint32_t in_len = get32(message + 16);

			if (offset + 4 <= length && in_len >= 4) {
				uint32_t filter = get32(message + offset);

				printf("usb: the host set the packet filter 0x%x: the cable %s a carrier\n",
				    filter, filter ? "is" : "is no longer");
				redwood_net_link_state(filter != 0);
			}
		}
		put32(out + 0, RNDIS_MSG_SET_C);
		put32(out + 4, 16);
		put32(out + 8, request_id);
		put32(out + 12, RNDIS_STATUS_SUCCESS);
		rndis_response_len = 16;
		break;
	case RNDIS_MSG_RESET:
		put32(out + 0, RNDIS_MSG_RESET_C);
		put32(out + 4, 16);
		put32(out + 8, RNDIS_STATUS_SUCCESS);   /* no request id in this one */
		put32(out + 12, 1);                     /* addressing reset needed */
		rndis_response_len = 16;
		break;
	case RNDIS_MSG_KEEPALIVE:
		put32(out + 0, RNDIS_MSG_KEEPALIVE_C);
		put32(out + 4, 16);
		put32(out + 8, request_id);
		put32(out + 12, RNDIS_STATUS_SUCCESS);
		rndis_response_len = 16;
		break;
	case RNDIS_MSG_HALT:
		rndis_response_len = 0;
		redwood_net_link_state(false);
		return;                     /* no reply to HALT */
	default:
		return;
	}
	usbnet_notify();
}

/*
 * EP0's chapter 9 and the two class requests RNDIS runs on. Returns what the data stage is:
 * -1 stall, 0 none (the status stage is a zero-length packet), 1 in (ep0_in), 2 out (ep0_out).
 */
static int
ep0_dispatch(const uint8_t *setup)
{
	uint8_t type = setup[0], request = setup[1];
	uint16_t value = (uint16_t)(setup[2] | (setup[3] << 8));
	uint16_t index = (uint16_t)(setup[4] | (setup[5] << 8));
	uint16_t length = (uint16_t)(setup[6] | (setup[7] << 8));

	if (usb_trace) {
		printf("usb: setup %02x %02x %04x %04x %04x\n", type, request, value, index, length);
	}

	if ((type & 0x60) == 0x20 && (type & 0x1f) == 1) {
		/* the class requests of RNDIS's control interface */
		if (request == 0x01 && (type & 0x80)) {
			/* SEND_ENCAPSULATED_RESPONSE: what rndis_process() left, when it left it */
			ep0_in_len = length < rndis_response_len ? length : rndis_response_len;
			if (ep0_in_len > sizeof(ep0_in)) {
				ep0_in_len = sizeof(ep0_in);
			}
			memcpy(ep0_in, rndis_response, ep0_in_len);
			return 1;
		}
		if (request == 0x00 && !(type & 0x80)) {
			/* SEND_ENCAPSULATED_MESSAGE: one RNDIS message */
			ep0_out_want = length;
			if (ep0_out_want > sizeof(ep0_out)) {
				return -1;
			}
			ep0_out_got = 0;
			return 2;
		}
		return -1;
	}
	if ((type & 0x60) != 0x00) {
		return -1;
	}

	switch (request) {
	case 0x00:                      /* GET_STATUS */
		ep0_in[0] = ep0_in[1] = 0;
		ep0_in_len = length < 2 ? length : 2;
		return (type & 0x80) ? 1 : -1;
	case 0x01:                      /* CLEAR_FEATURE: ENDPOINT_HALT, with the endpoint reset */
		if (value == 0 && (type & 0x1f) == 2) {
			uint32_t bit = ((index & 0x80) ? 1u << 16 : 1u) << (index & 0x0f);

			mac_set(REG_EP_RST, bit);
			mac_clr(REG_EP_RST, bit);
			return 0;
		}
		return value == 0 ? 0 : -1;
	case 0x03:                      /* SET_FEATURE (ENDPOINT_HALT would stall it) */
		return 0;
	case 0x05:                      /* SET_ADDRESS: the host starting over counts as a reset */
		usbnet_address_set = (uint8_t)(value & 0x7f);
		if (usbnet_configured && value != 0) {
			usbnet_stop();
		}
		mac_clr(REG_DEVICE_CONF, DEV_ADDR_MSK);
		mac_set(REG_DEVICE_CONF, (uint32_t)(value & 0x7f) << 24);
		return 0;
	case 0x06:                      /* GET_DESCRIPTOR */
		if (!(type & 0x80)) {
			return -1;
		}
		{
			const uint8_t *what = NULL;
			uint32_t n = 0;

			switch (value >> 8) {
			case 1:
				what = device_descriptor;
				n = sizeof(device_descriptor);
				break;
			case 2:
				what = (mac_read(REG_DEVICE_CONF) & DEV_SPEED) == 3 ? config_hs : config_fs;
				n = sizeof(config_fs);
				break;
			case 3:
				if ((value & 0xff) != 0) {
					return -1;      /* no strings but the language itself */
				}
				what = langid_descriptor;
				n = sizeof(langid_descriptor);
				break;
			case 6:
				what = qualifier_descriptor;
				n = sizeof(qualifier_descriptor);
				break;
			default:
				return -1;
			}
			ep0_in_len = length < n ? length : n;
			memcpy(ep0_in, what, ep0_in_len);
			return 1;
		}
	case 0x08:                      /* GET_CONFIGURATION */
		ep0_in[0] = usbnet_configured ? 1 : 0;
		ep0_in_len = length < 1 ? length : 1;
		return (type & 0x80) ? 1 : -1;
	case 0x09:                      /* SET_CONFIGURATION */
		if (value == 1) {
			usbnet_start();
			return usbnet_configured ? 0 : -1;
		}
		if (value == 0) {
			usbnet_stop();
			return 0;
		}
		return -1;
	case 0x0a:                      /* GET_INTERFACE */
		ep0_in[0] = 0;
		ep0_in_len = length < 1 ? length : 1;
		return (type & 0x80) ? 1 : -1;
	case 0x0b:                      /* SET_INTERFACE: no altsettings to speak of */
		return value == 0 ? 0 : -1;
	default:
		return -1;
	}
}

static void
ep0_send_more(void)                 /* EP0_TX: load the FIFO as it drains */
{
	uint32_t csr = mac_read(REG_EP0CSR);

	while (!(csr & EP0_FIFOFULL)) {
		uint32_t chunk = ep0_in_len - ep0_in_pos;

		if (chunk > 64) {
			chunk = 64;
		}
		if (chunk > 0) {
			ep0_fifo_write(ep0_in + ep0_in_pos, chunk);
			ep0_in_pos += chunk;
		}
		csr = (mac_read(REG_EP0CSR) & EP0_W1C_BITS) | EP0_TXPKTRDY;
		if (ep0_in_pos == ep0_in_len) {
			mac_write(REG_EP0CSR, csr | EP0_DATAEND);
			ep0_state = EP0_SETUP;
			return;
		}
		mac_write(REG_EP0CSR, csr);
		csr = mac_read(REG_EP0CSR);
	}
}

static void
ep0_receive(void)                   /* EP0_RX: the encapsulated message, then the status stage */
{
	uint32_t count = mac_read(REG_RXCOUNT0) & 0xffff;
	uint32_t room = ep0_out_want - ep0_out_got;
	uint32_t csr;

	if (count > room || count > 64) {
		csr = (mac_read(REG_EP0CSR) & EP0_W1C_BITS) | EP0_RXPKTRDY | EP0_SENDSTALL;
		mac_write(REG_EP0CSR, csr);
		ep0_state = EP0_SETUP;
		return;
	}
	ep0_fifo_read(ep0_out + ep0_out_got, count);
	ep0_out_got += count;
	csr = (mac_read(REG_EP0CSR) & EP0_W1C_BITS) | EP0_RXPKTRDY;
	if (ep0_out_got == ep0_out_want || count < 64) {
		mac_write(REG_EP0CSR, csr | EP0_DATAEND);
		ep0_state = EP0_SETUP;
		rndis_process(ep0_out, ep0_out_got);
		return;
	}
	mac_write(REG_EP0CSR, csr);
}

static void
usbnet_ep0_poll(void)
{
	uint32_t csr = mac_read(REG_EP0CSR);
	uint8_t setup[8];
	int what;

	if (csr & EP0_SENTSTALL) {
		mac_write(REG_EP0CSR, (csr & EP0_W1C_BITS & ~EP0_SENDSTALL) | EP0_SENTSTALL);
		ep0_state = EP0_SETUP;
		return;
	}
	if (csr & EP0_SETUPPKTRDY) {
		ep0_fifo_read(setup, sizeof(setup));
		what = ep0_dispatch(setup);
		csr = mac_read(REG_EP0CSR) & EP0_W1C_BITS;
		if (what == 1) {
			ep0_in_pos = 0;
			ep0_state = EP0_TX;
			mac_write(REG_EP0CSR, csr | EP0_SETUPPKTRDY | EP0_DPHTX);
			ep0_send_more();
		} else if (what == 2) {
			ep0_state = EP0_RX;
			mac_write(REG_EP0CSR, csr | EP0_SETUPPKTRDY);
		} else if (what == 0) {
			ep0_state = EP0_SETUP;
			mac_write(REG_EP0CSR, csr | EP0_SETUPPKTRDY | EP0_DATAEND);
		} else {
			ep0_state = EP0_SETUP;
			mac_write(REG_EP0CSR, csr | EP0_SETUPPKTRDY | EP0_SENDSTALL);
		}
		return;
	}
	if (ep0_state == EP0_TX) {
		ep0_send_more();
	} else if (ep0_state == EP0_RX && (mac_read(REG_EP0CSR) & EP0_RXPKTRDY)) {
		ep0_receive();
	}
}

/*
 * The bus: a reset or a disconnect un-configures everything. The host that reset us will
 * come back with SET_ADDRESS and SET_CONFIG as if nothing had happened, and so do we.
 */
static void
usbnet_bus_poll(void)
{
	uint32_t intr = mac_read(REG_USB_INTR);

	if (intr & (INTR_RESET | INTR_DISCONN)) {
		mac_write(REG_USB_INTR, intr);        /* write one to clear */
		printf("usb: the bus was reset%s\n", (intr & INTR_DISCONN) ? " (disconnected)" : "");
		if (usbnet_configured || usbnet_address_set != 0) {
			usbnet_stop();
			mac_clr(REG_DEVICE_CONF, DEV_ADDR_MSK);
			usbnet_address_set = 0;
		}
	}
	intr = mac_read(REG_DEV_LINK_INTR);
	if (intr) {
		mac_write(REG_DEV_LINK_INTR, intr);
	}
}

/*
 * SET_CONFIGURATION(1), and the recovery after any reset: the endpoints (their CSRs, the
 * endpoint-FIFO map mtu3 gives a U2-only IP), the QMU queues, and 31 receive buffers offered.
 * The thread's call, with the lock: transmit() must not be arming GPDs against it.
 */
static void
usbnet_start(void)
{
	uint32_t speed;
	unsigned armed;

	if (!usbnet_on) {
		return;                     /* step 1 has no DMA memory and no thread behind it */
	}
	lck_mtx_lock(&usbnet_lock);
	if (usbnet_configured) {
		lck_mtx_unlock(&usbnet_lock);
		return;                     /* the host set the same configuration twice */
	}
	speed = mac_read(REG_DEVICE_CONF) & DEV_SPEED;
	usbnet_highspeed = speed == 3;
	usbnet_bulk_maxp = usbnet_highspeed ? 512 : 64;
	printf("usb: configured at %s speed, bulk packets of %u bytes\n",
	    speed == 3 ? "high" : speed == 1 ? "full" : "an unknown", usbnet_bulk_maxp);

	/* the endpoints: bulk in/out on 1 (slot 1), the interrupt endpoint on 2 (slot 0) */
	mac_write(REG_TX1CSR0, CSR_DMAREQEN | usbnet_bulk_maxp);
	mac_write(REG_TX1CSR1, (0u << 4) | (1u << 8));
	mac_write(REG_TX1CSR2, (64u >> 4) | (9u << 16));
	mac_write(REG_RX1CSR0, CSR_DMAREQEN | usbnet_bulk_maxp);
	mac_write(REG_RX1CSR1, (0u << 4) | (1u << 8));
	mac_write(REG_RX1CSR2, (usbnet_rx_fifo_addr >> 4) | (9u << 16));
	mac_write(REG_TX2CSR0, CSR_DMAREQEN | 8);
	mac_write(REG_TX2CSR1, (1u << 4) | (0u << 8));
	mac_write(REG_TX2CSR2, (576u >> 4) | (9u << 16) |
	    ((usbnet_highspeed ? 6u : 32u) << 24));
	mac_set(REG_QCR0, (1u << EP_BULK_IN) | (1u << EP_NOTIFY) | (1u << (16 + EP_BULK_OUT)));
	mac_set(REG_QCR1, (1u << EP_BULK_IN) | (1u << EP_NOTIFY));
	mac_set(REG_QCR3, 1u << (16 + EP_BULK_OUT));

	/* the queues: started empty (the TX rings), and offered 31 receive buffers */
	mac_write(REG_TXQSAR1, ring_pa(0));
	mac_write(REG_RXQSAR1, ring_pa(1));
	mac_write(REG_TXQSAR2, (uint32_t)kvtophys((vm_offset_t)notify_ring));
	for (armed = 0; armed < RING_DEPTH - 1; armed++) {
		if (rx_free_count == 0 || (rx_enq + 1) % RING_DEPTH == rx_head) {
			break;
		}
		rx_gpd_buf[rx_enq] = rx_free[--rx_free_count];
		gpd_arm(&rx_ring[rx_enq], rx_buf_pa[rx_gpd_buf[rx_enq]], 0, BUFFER_SIZE, false);
		dma_flush((vm_offset_t)&rx_ring[rx_enq], sizeof(struct usbnet_gpd));
		rx_enq = (rx_enq + 1) % RING_DEPTH;
	}
	dma_flush(notify_va, sizeof(notification_message));
	mac_write(REG_TXQCSR1, Q_START);
	mac_write(REG_TXQCSR2, Q_START);
	mac_write(REG_RXQCSR1, Q_START);

	usbnet_configured = true;
	lck_mtx_unlock(&usbnet_lock);
}

/*
 * SET_CONFIGURATION(0), a bus reset, an address change while configured: the queues stopped
 * (and waited to stop), the rings and buffers back to the start, the endpoints reset. The
 * link itself is dropped once the lock is let go.
 */
static void
usbnet_stop(void)
{
	bool was = usbnet_configured;
	unsigned i;

	if (!usbnet_on) {
		return;
	}
	lck_mtx_lock(&usbnet_lock);
	if (usbnet_configured) {
		mac_write(REG_TXQCSR1, Q_STOP);
		mac_write(REG_RXQCSR1, Q_STOP);
		mac_write(REG_TXQCSR2, Q_STOP);
		for (i = 0; i < 5000; i++) {
			if (!(mac_read(REG_TXQCSR1) & Q_ACTIVE) &&
			    !(mac_read(REG_RXQCSR1) & Q_ACTIVE) &&
			    !(mac_read(REG_TXQCSR2) & Q_ACTIVE)) {
				break;
			}
			delay_for_interval(10, NSEC_PER_USEC);
		}
		if (i == 5000) {
			printf("usb: a queue would not stop (TX %#x RX %#x notify %#x)\n",
			    mac_read(REG_TXQCSR1), mac_read(REG_RXQCSR1), mac_read(REG_TXQCSR2));
		}
	}
	mac_write(REG_EP_RST, (1u << 16) << EP_BULK_IN);
	mac_write(REG_EP_RST, 0);
	mac_write(REG_EP_RST, (1u << 16) << EP_NOTIFY);
	mac_write(REG_EP_RST, 0);
	mac_write(REG_EP_RST, 1u << EP_BULK_OUT);
	mac_write(REG_EP_RST, 0);

	tx_head = tx_enq = 0;
	rx_head = rx_enq = 0;
	notify_pos = 0;
	for (i = 0; i < RING_DEPTH; i++) {
		tx_ring[i].flag = 0;
		rx_ring[i].flag = 0;
		tx_free[i] = rx_free[i] = i;
		tx_gpd_buf[i] = rx_gpd_buf[i] = i;
	}
	tx_free_count = rx_free_count = RING_DEPTH;
	dma_flush(dma_rings, PAGE_SIZE);
	usbnet_configured = false;
	usbnet_bulk_maxp = 0;
	ep0_state = EP0_SETUP;
	rndis_response_len = 0;
	lck_mtx_unlock(&usbnet_lock);
	if (was) {
		redwood_net_link_state(false);
	}
}

/*
 * The link's transmit: one frame, wrapped in its RNDIS header, onto the bulk-in ring. Called
 * by the interface's output path (one thread at a time, net.c's net_tx_lock held there), so
 * the lock is all that keeps it from the thread's configure/teardown. False = dropped, and
 * the interface counts it.
 */
static bool
usbnet_transmit(mbuf_t packet, size_t length)
{
	struct usbnet_gpd *gpd;
	uint8_t *buffer;
	unsigned total, bidx;
	bool sent = false;

	if (!usbnet_on) {
		return false;
	}
	lck_mtx_lock(&usbnet_lock);
	if (!usbnet_configured || length == 0 || length > REDWOOD_NET_FRAME_MAX) {
		goto out;
	}
	/* what the hardware finished sending: its buffers are ours again */
	while (tx_head != tx_enq) {
		gpd = &tx_ring[tx_head];
		dma_flush((vm_offset_t)gpd, sizeof(*gpd));
		if (gpd->flag & GPD_HWO) {
			break;
		}
		tx_free[tx_free_count++] = tx_gpd_buf[tx_head];
		tx_head = (tx_head + 1) % RING_DEPTH;
	}
	if ((tx_enq + 1) % RING_DEPTH == tx_head || tx_free_count == 0) {
		goto out;                   /* the host has not taken 31 frames from us yet */
	}
	bidx = tx_free[--tx_free_count];
	buffer = tx_buffers + (size_t)bidx * BUFFER_SIZE;
	put32(buffer, RNDIS_MSG_PACKET);
	put32(buffer + 4, RNDIS_HEADER_SIZE + (uint32_t)length);
	put32(buffer + 8, 36);          /* the frame, from request_id + 36 = byte 44 */
	put32(buffer + 12, (uint32_t)length);
	if (mbuf_copydata(packet, 0, length, buffer + RNDIS_HEADER_SIZE) != 0) {
		tx_free[tx_free_count++] = bidx;
		goto out;
	}
	total = RNDIS_HEADER_SIZE + (uint32_t)length;
	dma_flush((vm_offset_t)buffer, total);
	/* a transfer that fills its packets exactly needs a zero-length packet after it */
	gpd = &tx_ring[tx_enq];
	gpd_arm(gpd, tx_buf_pa[bidx], (uint16_t)total, 0, total % usbnet_bulk_maxp == 0);
	dma_flush((vm_offset_t)gpd, sizeof(*gpd));
	tx_gpd_buf[tx_enq] = bidx;
	tx_enq = (tx_enq + 1) % RING_DEPTH;
	queue_resume(REG_TXQCSR1);
	sent = true;
out:
	lck_mtx_unlock(&usbnet_lock);
	return sent;
}

/*
 * The thread's receive half: what the host sent, reaped GPD by GPD, the frame handed up clear
 * of the lock (nothing of redwood_net.c runs with it), the buffer straight back into play.
 */
static void
usbnet_rx_poll(void)
{
	for (;;) {
		struct usbnet_gpd *gpd;
		uint8_t *buffer;
		uint32_t message_len, data_len;
		unsigned bidx, before;
		bool bad;

		lck_mtx_lock(&usbnet_lock);
		if (!usbnet_configured || rx_head == rx_enq) {
			lck_mtx_unlock(&usbnet_lock);
			break;
		}
		gpd = &rx_ring[rx_head];
		dma_flush((vm_offset_t)gpd, sizeof(*gpd));
		if (gpd->flag & GPD_HWO) {
			lck_mtx_unlock(&usbnet_lock);
			break;
		}
		bidx = rx_gpd_buf[rx_head];
		buffer = rx_buffers + (size_t)bidx * BUFFER_SIZE;
		dma_flush((vm_offset_t)buffer, BUFFER_SIZE);
		message_len = get32(buffer + 4);
		data_len = get32(buffer + 12);
		bad = get32(buffer) != RNDIS_MSG_PACKET ||
		    data_len > REDWOOD_NET_FRAME_MAX ||
		    RNDIS_HEADER_SIZE + data_len > message_len ||
		    message_len > gpd->buf_len;
		if (!bad) {
			memcpy(rx_stage, buffer + RNDIS_HEADER_SIZE, data_len);
		} else if (usb_trace) {
			printf("usb: a bad RNDIS packet was dropped (%u bytes in the buffer)\n", gpd->buf_len);
		}
		rx_free[rx_free_count++] = bidx;
		rx_head = (rx_head + 1) % RING_DEPTH;
		/* top the ring up: GPDs at the tail, buffers from the free list */
		before = rx_enq;
		while (rx_free_count > 0 && (rx_enq + 1) % RING_DEPTH != rx_head) {
			bidx = rx_free[--rx_free_count];
			rx_gpd_buf[rx_enq] = bidx;
			gpd_arm(&rx_ring[rx_enq], rx_buf_pa[bidx], 0, BUFFER_SIZE, false);
			dma_flush((vm_offset_t)&rx_ring[rx_enq], sizeof(struct usbnet_gpd));
			rx_enq = (rx_enq + 1) % RING_DEPTH;
		}
		if (rx_enq != before) {
			queue_resume(REG_RXQCSR1);
		}
		lck_mtx_unlock(&usbnet_lock);
		if (!bad) {
			redwood_net_input(rx_stage, data_len);
		}
	}
}

static void
usbnet_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t last_csr = 0, last_intr = 0, last_link = 0, last_power = 0, last_conf = 0;
	unsigned tick = 0, beats = 0;

	for (;;) {
		usbnet_bus_poll();
		usbnet_ep0_poll();
		usbnet_rx_poll();
		/* the pulse (usb_trace): run 3's whole wire story was invisible -- the pullup never
		 * reached the host and the MAC never saw one byte, and nothing said which. Now any
		 * change of the EP0 status, the bus interrupts, the link, the pullup or the speed
		 * is a line, and one line every five seconds says the thread is alive. */
		if (usb_trace && !usbnet_configured) {
			uint32_t csr = mac_read(REG_EP0CSR), intr = mac_read(REG_USB_INTR);
			uint32_t link = mac_read(REG_DEV_LINK_INTR), power = mac_read(REG_POWER_MGMT);
			uint32_t conf = mac_read(REG_DEVICE_CONF);
			tick++;
			if (csr != last_csr || intr != last_intr || link != last_link ||
			    power != last_power || conf != last_conf || (tick % 5000) == 0) {
				printf("usb: beat %u: EP0CSR %#x, INTR %#x, LINK %#x, POWER %#x, CONF %#x\n",
				    ++beats, csr, intr, link, power, conf);
				last_csr = csr; last_intr = intr; last_link = link;
				last_power = power; last_conf = conf;
			}
		}
		delay_for_interval(1, NSEC_PER_MSEC);
	}
}

static const struct redwood_net_link usbnet_link = {
	.name = "usb",
	.address = { 0x02, 0x72, 0x6f, 0x73, 0x65, 0x01 },
	.wireless = false,
	.up = false,                    /* the cable is a carrier once the host sets RNDIS's filter */
	.transmit = usbnet_transmit,
	.poll = NULL,
};

static bool
usbnet_wait(uint32_t offset, uint32_t bit, const char *what)
{
	for (unsigned i = 0; i < 2000; i++) {
		if (ippc_read(offset) & bit) {
			return true;
		}
		delay_for_interval(10, NSEC_PER_USEC);
	}
	printf("usb: %s never came (STS1 %#x, STS2 %#x, U2 %#x)\n", what,
	    ippc_read(IPPC_PW_STS1), ippc_read(IPPC_PW_STS2), ippc_read(IPPC_U2_CTRL));
	return false;
}

/*
 * Boot-arg usb_step: bring the SSUSB device up (clocks, PHY, power), and past step 1, attach
 * the link and answer the host. Called from redwood_net_init(); the one-time register writes
 * follow mtu3's ssusb_device_on()/mtu3_regs_init() and the PHY recipe of the phone's branch
 * of phy-mtk-mt6785.c.
 */
void
redwood_usbnet_init(void)
{
	thread_t thread;
	vm_offset_t maps[2];
	uint32_t speed, cap_tx, cap_rx;
	unsigned i;

	PE_parse_boot_argn("usb_step", &usb_step, sizeof(usb_step));
	PE_parse_boot_argn("usb_trace", &usb_trace, sizeof(usb_trace));
	if (usb_step == 0) {
		return;
	}
	printf("usb: step %u: the phone's cable as the network link\n", usb_step);

	usb_infra = ml_io_map(INFRA_PHYS, 0x1000);
	usb_top = ml_io_map(TOPCLK_PHYS, 0x1000);
	usb_apmixed = ml_io_map(APMIXED_PHYS, 0x1000);
	usb_sif = ml_io_map(USB_SIF_PHYS, 0x1000);
	usb_ippc = ml_io_map(USB_IPPC_PHYS, 0x100);
	usb_mac = ml_io_map(USB_MAC_PHYS, USB_MAC_SIZE);

	/* the clock chain as Linux's clock driver builds it (clk-mt6785.c): the INFRA2 gates, the
	 * APMIXED 26 MHz reference the device's "ref_ck" is, and the usb_top_sel mux the sys clock
	 * hangs from (read only: its parents, clk26m and univpll divisions, are all alive at boot).
	 * What the boot loader left behind is said first: run 1's IPPC read zero with the gates
	 * "open" (0x80000002 -- read as on, meaning off: the bank is a power-down bank), and these
	 * are the things between the gates and a living block. */
	printf("usb: as the boot left it: CK_CFG_8 %#x, APMIXED %#x, INFRA2 %#x, IPPC %#x/%#x/%#x\n",
	    top_read(REG_CKCFG8), apm_read(REG_APMIXED_CG),
	    *(volatile uint32_t *)(usb_infra + INFRA2_CG_STA),
	    ippc_read(IPPC_PW_STS1), ippc_read(IPPC_PW_STS2), ippc_read(IPPC_U2_CTRL));
	/* the gates: a power-down bank, so the bit goes to the CLEAR bank to open the clock */
	*(volatile uint32_t *)(usb_infra + INFRA2_CG_CLR) = INFRA_USB | INFRA_SSUSB_XHCI;
	apm_write(REG_APMIXED_CG, apm_read(REG_APMIXED_CG) | APMIXED_SSUSB26M);
	delay_for_interval(250, NSEC_PER_USEC);
	printf("usb: clocks on: INFRA2 %#x (the two bits clear), APMIXED %#x\n",
	    *(volatile uint32_t *)(usb_infra + INFRA2_CG_STA), apm_read(REG_APMIXED_CG));

	/* the whole IP out of any reset the boot loader left it in (ssusb_ip_sw_reset) */
	ippc_write(IPPC_PW_CTRL0, ippc_read(IPPC_PW_CTRL0) | 1u);
	delay_for_interval(1, NSEC_PER_USEC);
	ippc_write(IPPC_PW_CTRL0, ippc_read(IPPC_PW_CTRL0) & ~1u);

	/* the PHY: its LDO on, then out of the UART mode the boot loader leaves it in */
	sif_write(PHY_PHYA_REG1, sif_read(PHY_PHYA_REG1) | (1u << 29));
	sif_write(PHY_DTM0, sif_read(PHY_DTM0) & ~((1u << 26) | (1u << 30)));
	sif_write(PHY_DTM1, sif_read(PHY_DTM1) & ~(1u << 16));
	sif_write(PHY_ACR4, sif_read(PHY_ACR4) & ~((1u << 18) | (1u << 17) | (1u << 16) | (1u << 9) | (1u << 8)));
	sif_write(PHY_ACR6, sif_read(PHY_ACR6) & ~(1u << 20));
	sif_write(PHY_DTM0, sif_read(PHY_DTM0) & ~(1u << 18));
	delay_for_interval(800, NSEC_PER_USEC);
	/* VBUS said by hand: a peripheral may not connect until it sees VBUS, and this board's
	 * sense line goes nowhere the driver can read. The FORCE bits only *enable* the overrides
	 * (phy-mtk-ssusb-reg.h: FORCE at 13/12/10, the values RG_VBUSVALID/RG_SESSEND/RG_AVALID at
	 * 5/4/2) -- run 3 set the enables and left the values at whatever the boot loader kept, a
	 * "no VBUS" the SOFT_CONN pullup then politely obeyed: the host never saw a thing. */
	{
		uint32_t dtm1 = sif_read(PHY_DTM1);
		dtm1 |= (1u << 13) | (1u << 5);   /* VBUS valid: forced, and said */
		dtm1 |= (1u << 10) | (1u << 2);   /* A valid: forced, and said */
		dtm1 |= (1u << 12);               /* session end: forced... */
		dtm1 &= ~(1u << 4);               /* ...to "no session end" */
		sif_write(PHY_DTM1, dtm1);
		printf("usb: DTM1 now %#x\n", sif_read(PHY_DTM1));
	}

	/* the IP block's own power: on, the U2 port a device, a reset pulse once it is alive */
	ippc_write(IPPC_PW_CTRL2, ippc_read(IPPC_PW_CTRL2) & ~1u);
	ippc_write(IPPC_U2_CTRL, (ippc_read(IPPC_U2_CTRL) & ~((1u << 0) | (1u << 1) | (1u << 2))) | (1u << 7));
	if (!usbnet_wait(IPPC_PW_STS1, 1u << 10, "the 125M clock") ||
	    !usbnet_wait(IPPC_PW_STS2, 1u << 0, "the second power status")) {
		printf("usb: giving up on a device that never powered on\n");
		return;
	}
	ippc_write(IPPC_DEV_RST, 1u);
	delay_for_interval(1, NSEC_PER_USEC);
	ippc_write(IPPC_DEV_RST, 0u);

	/* the MAC: interrupts off (there is a thread instead), statuses cleared, known defaults */
	mac_write(REG_LV1IECR, 0xffffffffu);
	mac_write(REG_EPIECR, 0xffffffffu);
	mac_write(REG_EPISR, 0xffffffffu);
	mac_write(REG_USB_INTR, 0xffffffffu);
	mac_write(REG_LTSSM_INTR, 0xffffffffu);
	mac_write(REG_DEV_LINK_INTR, 0xffffffffu);
	mac_clr(REG_USB3_CONFIG, 1u);
	mac_set(REG_POWER_MGMT, 1u << 5);          /* high speed allowed */
	mac_write(REG_DEVICE_CONF, 0);
	mac_set(REG_QCR0, 1u);                     /* GPDs of 16 bytes, checksums on */
	mac_set(REG_MISC_CTRL, 3u);                /* VBUS forced on: no sense wire here */
	mac_write(REG_EP0CSR, (mac_read(REG_EP0CSR) & ~EP0_MAXPKTSZ) | 64);

	cap_tx = mac_read(REG_CAP_TXFFSZ);
	cap_rx = mac_read(REG_CAP_RXFFSZ);
	usbnet_fifo_limit = cap_tx / (2 * 512);
	usbnet_rx_fifo_addr = 64 + usbnet_fifo_limit * 512;
	printf("usb: endpoint FIFOs: TX %u x 512 from byte 64, RX from %u (CAP %#x/%#x, EPINFO %#x)\n",
	    usbnet_fifo_limit, usbnet_rx_fifo_addr, cap_tx, cap_rx, mac_read(REG_CAP_EPINFO));
	if (usbnet_fifo_limit < 2) {
		printf("usb: too little FIFO for three endpoints: stopping here\n");
		return;
	}

	if (usb_step >= 2) {
		/* the DMA memory: one page of rings (and the notification in it), 64 buffers */
		if (kmem_alloc(kernel_map, &maps[0], PAGE_SIZE, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
		    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS ||
		    kmem_alloc(kernel_map, &maps[1], round_page(2 * RING_DEPTH * BUFFER_SIZE),
		    KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
			printf("usb: the DMA memory cannot be had\n");
			return;
		}
		dma_rings = maps[0];
		dma_buffers = maps[1];
		tx_ring = (struct usbnet_gpd *)dma_rings;
		rx_ring = (struct usbnet_gpd *)(dma_rings + RING_DEPTH * sizeof(struct usbnet_gpd));
		notify_ring = (struct usbnet_gpd *)(dma_rings + 2 * RING_DEPTH * sizeof(struct usbnet_gpd));
		notify_va = dma_rings + 2 * RING_DEPTH * sizeof(struct usbnet_gpd) + 2 * sizeof(struct usbnet_gpd);
		tx_buffers = (uint8_t *)dma_buffers;
		rx_buffers = tx_buffers + (size_t)RING_DEPTH * BUFFER_SIZE;
		if (kvtophys(dma_rings) + PAGE_SIZE > 0xffffffffull) {
			printf("usb: the rings landed above the DMA window: stopping here\n");
			return;
		}
		for (i = 0; i < RING_DEPTH; i++) {
			if (kvtophys((vm_offset_t)tx_buffers + (size_t)i * BUFFER_SIZE) + BUFFER_SIZE > 0xffffffffull ||
			    kvtophys((vm_offset_t)rx_buffers + (size_t)i * BUFFER_SIZE) + BUFFER_SIZE > 0xffffffffull) {
				printf("usb: a buffer landed above the DMA window: stopping here\n");
				return;
			}
			tx_buf_pa[i] = (uint32_t)kvtophys((vm_offset_t)tx_buffers + (size_t)i * BUFFER_SIZE);
			rx_buf_pa[i] = (uint32_t)kvtophys((vm_offset_t)rx_buffers + (size_t)i * BUFFER_SIZE);
			tx_ring[i].next_gpd = (uint32_t)kvtophys((vm_offset_t)&tx_ring[(i + 1) % RING_DEPTH]);
			rx_ring[i].next_gpd = (uint32_t)kvtophys((vm_offset_t)&rx_ring[(i + 1) % RING_DEPTH]);
			tx_free[i] = rx_free[i] = i;
			tx_gpd_buf[i] = rx_gpd_buf[i] = i;
		}
		tx_free_count = rx_free_count = RING_DEPTH;
		notify_ring[0].next_gpd = (uint32_t)kvtophys((vm_offset_t)&notify_ring[1]);
		notify_ring[1].next_gpd = (uint32_t)kvtophys((vm_offset_t)notify_ring);
		notify_pa = (uint32_t)kvtophys(notify_va);
		memcpy((void *)notify_va, notification_message, sizeof(notification_message));
		dma_flush(dma_rings, PAGE_SIZE);
		printf("usb: rings at physical %#x, buffers around %#llx\n",
		    (uint32_t)kvtophys(dma_rings), (unsigned long long)kvtophys(dma_buffers));

		usbnet_on = true;
		if (!redwood_net_attach(&usbnet_link)) {
			printf("usb: en0 was taken already: the link will idle\n");
		}
		if (kernel_thread_start(usbnet_thread, NULL, &thread) == KERN_SUCCESS) {
			thread_set_thread_name(thread, "the cable's traffic");
			thread_deallocate(thread);
		} else {
			printf("usb: the thread cannot be started\n");
			usbnet_on = false;
		}
	}

	/* the pullup: the host may enumerate us from here (step 1 only wanted the speed seen) */
	mac_set(REG_POWER_MGMT, (1u << 6) | 1u);
	if (usb_step == 1) {
		delay_for_interval(250, NSEC_PER_MSEC);
		speed = mac_read(REG_DEVICE_CONF) & DEV_SPEED;
		printf("usb: the PHY settled at %s speed\n",
		    speed == 3 ? "high" : speed == 1 ? "full" : "no (no host?)");
	}
}
