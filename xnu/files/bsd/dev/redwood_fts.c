/*
 * The touch screen of the Redmi Note 10S (rosemary): a FocalTech FT3518 on the MT6785's i2c0
 * at the 7-bit address 0x38, as a source of touches for /dev/redwood-touch
 * (sys/redwood_input.h). Only with boot-arg touchscreen=3: every address below belongs to
 * this phone.
 *
 * There is no pin or I2C framework in this kernel, so the driver reaches the registers
 * itself, in the order the vendor's Linux driver does (drivers/input/touchscreen/FT3518/ of
 * the phone's own kernel) and imdt's walk of this phone did (logs/imdt1):
 *   pins     GPIO51 is i2c0's clock and GPIO52 its data, both function 0 -- the pin's own
 *            GPIO. The bus is driven by hand, open drain: a line is pulled low by turning
 *            its direction to output (its DOUT stays 0 the whole time) and released by
 *            turning it to input, the board's pull-ups taking it high; no pull register is
 *            touched, the way imdt's scan ran, at 250 kHz and down
 *   reset    GPIO2, active low in the device tree -- left alone: imdt read it 0 while the
 *            chip answered, so it is not wired the way the tree says; the chip boots the
 *            firmware in its flash by itself. Only if it does not answer at all is the pin
 *            pulsed once, and the chip looked for again
 *   chip     registers 0xA3/0x9F its id (0x54 on this phone, as imdt's scan read), 0xA6
 *            its firmware version, 0xA8 its vendor; a report 63 bytes from register 0x00:
 *            byte 2 the number of points, and each point six bytes from byte 3 -- flags
 *            and x's high half, x's low, id and y's high half, y's low, pressure, area
 * No interrupt is used: a thread looks at the interrupt line (GPIO1, low while the chip
 * has a report) 125 times a second (boot-arg touch_poll_us=N: every N microseconds instead
 * of 8000), the way the Goodix driver beside this one does, and reads a report every 80 ms
 * whatever the line says, in case that pin is not wired either. Boot-arg touch_flip=N
 * mirrors the panel: bit 1 left-right, bit 2 top-bottom.
 *
 * Register layouts: Linux drivers/pinctrl/mediatek/pinctrl-mt6785.c (a bit per pin, thirty-
 * two pins to a register at 0x10 strides; four bits of mode, eight pins to a register), the
 * phone's touchscreen.dtsi, and the vendor driver named above.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/redwood_input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);

static vm_offset_t gpio;

static inline uint32_t
rd(uint32_t at)
{
	return *(volatile uint32_t *)(gpio + at);
}

static inline void
wr(uint32_t at, uint32_t value)
{
	*(volatile uint32_t *)(gpio + at) = value;
}

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

static void
sleep_us(uint32_t us)
{
	delay_for_interval(us, NSEC_PER_USEC);
}

/* ---- the pins ------------------------------------------------------------------------------------- */

#define GPIO_PHYS       0x10005000
#define GPIO_SIZE       0x1000

#define DIR_AT(pin)     (0x0000 + 0x10 * ((pin) / 32))
#define DOUT_AT(pin)    (0x0100 + 0x10 * ((pin) / 32))
#define DIN_AT(pin)     (0x0200 + 0x10 * ((pin) / 32))
#define MODE_AT(pin)    (0x0300 + 0x10 * ((pin) / 8))          /* four bits a pin */
#define MODE_BIT(pin)   (((pin) % 8) * 4)

#define PIN_SCL         51              /* i2c0's clock */
#define PIN_SDA         52              /* i2c0's data */
#define PIN_IRQ         1               /* low while the chip has a report */
#define PIN_RESET       2               /* low = reset, in the device tree */

static void
pin_mode_gpio(unsigned pin)
{
	uint32_t at = MODE_AT(pin);

	wr(at, rd(at) & ~(0xfu << MODE_BIT(pin)));
	__asm__ volatile ("dsb sy");
}

static void
pin_output(unsigned pin, bool high)
{
	uint32_t bit = 1u << (pin % 32);

	wr(DOUT_AT(pin), (rd(DOUT_AT(pin)) & ~bit) | (high ? bit : 0));
	wr(DIR_AT(pin), rd(DIR_AT(pin)) | bit);
	__asm__ volatile ("dsb sy");
}

static inline bool
pin_get(unsigned pin)
{
	return (rd(DIN_AT(pin)) >> (pin % 32)) & 1u;
}

/*
 * An I2C line, open drain: the pin's DOUT stays 0 all the time, so driving it is turning the
 * direction to output and releasing is turning it back to input. imdt's scan of this phone
 * ran this way and found the chip at 250 kHz and slower (half clock 2 us and slower).
 */
static uint32_t half_us = 2;            /* boot-arg touch_i2c_half_us=N: the half clock, us */

static void
line_low(unsigned pin)
{
	wr(DIR_AT(pin), rd(DIR_AT(pin)) | 1u << (pin % 32));
}

static void
line_release(unsigned pin)
{
	wr(DIR_AT(pin), rd(DIR_AT(pin)) & ~(1u << (pin % 32)));
}

static inline bool
sda(void)
{
	return pin_get(PIN_SDA);
}

/* Long enough for a pad to settle, and the half clock besides. */
static void
half_clock(void)
{
	(void)rd(DIN_AT(PIN_SDA));
	(void)rd(DIN_AT(PIN_SDA));
	if (half_us > 1) {
		delay_for_interval(half_us - 1, NSEC_PER_USEC);
	}
}

/* ---- I2C, driven by hand -------------------------------------------------------------------------- */

#define FTS_ADDR        0x38            /* the tree's touch@0 */

static bool
i2c_start(void)
{
	line_release(PIN_SDA);
	line_release(PIN_SCL);
	half_clock();
	if (!sda()) {
		return false;               /* somebody holds the line down: not our chip's doing */
	}
	line_low(PIN_SDA);              /* data falls while the clock is high */
	half_clock();
	line_low(PIN_SCL);
	half_clock();
	return true;
}

static void
i2c_restart(void)
{
	line_release(PIN_SDA);
	half_clock();
	line_release(PIN_SCL);
	half_clock();
	line_low(PIN_SDA);
	half_clock();
	line_low(PIN_SCL);
	half_clock();
}

static void
i2c_stop(void)
{
	line_low(PIN_SDA);
	half_clock();
	line_release(PIN_SCL);
	half_clock();
	line_release(PIN_SDA);          /* data rises while the clock is high */
	half_clock();
}

static bool
i2c_out(uint8_t byte)
{
	bool ack;

	for (int bit = 7; bit >= 0; bit--) {
		if (byte >> bit & 1) {
			line_release(PIN_SDA);
		} else {
			line_low(PIN_SDA);
		}
		half_clock();
		line_release(PIN_SCL);
		half_clock();
		line_low(PIN_SCL);
		half_clock();
	}
	line_release(PIN_SDA);          /* the one answering drives it for the acknowledgement */
	half_clock();
	line_release(PIN_SCL);
	half_clock();
	ack = !sda();
	line_low(PIN_SCL);
	half_clock();
	return ack;
}

static uint8_t
i2c_in(bool ack)
{
	uint8_t byte = 0;

	line_release(PIN_SDA);
	for (int bit = 7; bit >= 0; bit--) {
		half_clock();
		line_release(PIN_SCL);
		half_clock();
		byte = (uint8_t)(byte << 1 | (sda() ? 1 : 0));
		line_low(PIN_SCL);
	}
	if (ack) {
		line_low(PIN_SDA);          /* more to come */
	} else {
		line_release(PIN_SDA);      /* the last one */
	}
	half_clock();
	line_release(PIN_SCL);
	half_clock();
	line_low(PIN_SCL);
	half_clock();
	return byte;
}

/* A read of the chip: the register's address written, the read that follows begun with a
 * repeated start, as the vendor's fts_read() sends it. */
static bool
fts_read(uint8_t reg, uint8_t *data, uint32_t length)
{
	if (!i2c_start()) {
		return false;
	}
	if (!i2c_out(FTS_ADDR << 1) || !i2c_out(reg)) {
		i2c_stop();
		return false;
	}
	i2c_restart();
	if (!i2c_out(FTS_ADDR << 1 | 1)) {
		i2c_stop();
		return false;
	}
	for (uint32_t i = 0; i < length; i++) {
		data[i] = i2c_in(i + 1 < length);
	}
	i2c_stop();
	return true;
}

/* ---- the chip ------------------------------------------------------------------------------------- */

#define FTS_REG_CHIP_ID         0xA3
#define FTS_REG_CHIP_ID2        0x9F
#define FTS_REG_FW_VER          0xA6
#define FTS_REG_VENDOR_ID       0xA8
#define FTS_DATA_LEN            63              /* ten points of six bytes and three of header */
#define FTS_MAX_POINTS          10
#define FTS_EVENT_DOWN          0               /* the six bits above the x's high half */
#define FTS_EVENT_UP            1
#define FTS_EVENT_CONTACT       2

/* The tree's panel-max-x/y. The panel is the display here: the window server's 1125 x 2436
 * is scaled onto its 1080 x 2400, so a finger's place on the panel is its place on the
 * display, said in the 0 to 65535 the queue wants. */
#define PANEL_W         1080
#define PANEL_H         2400

static uint32_t flip;                    /* boot-arg touch_flip: 1 mirror left-right, 2 top-bottom */
static uint32_t bad_reads;

/*
 * The vendor's fts_wait_tp_to_valid: the chip says 0 while its firmware is coming and 0xff
 * when nobody drives the line; anything else is a chip that will report. imdt's scan read
 * the id 0x54 on this phone.
 */
static bool
fts_probe(bool verbose)
{
	uint8_t id = 0, id2 = 0, version = 0, vendor = 0;

	for (int attempt = 0; attempt < 4; attempt++) {
		if (fts_read(FTS_REG_CHIP_ID, &id, 1) && fts_read(FTS_REG_CHIP_ID2, &id2, 1) &&
		    fts_read(FTS_REG_FW_VER, &version, 1) && fts_read(FTS_REG_VENDOR_ID, &vendor, 1) &&
		    id != 0 && id != 0xff) {
			printf("fts: id %#x %#x, firmware %#x, vendor %#x\n", id, id2, version, vendor);
			return true;
		}
		if (attempt == 0 && verbose) {
			printf("fts: the chip does not answer yet (id %#x %#x, firmware %#x, vendor %#x)\n",
			    id, id2, version, vendor);
		}
		sleep_ms(200);
	}
	return false;
}

/* imdt's walk: which addresses answer at all, said when the chip does not. */
static void
i2c_who_answers(void)
{
	unsigned found = 0;

	for (uint8_t addr = 8; addr < 0x78; addr++) {
		if (i2c_start() && i2c_out(addr << 1)) {
			printf("fts: %#x answers\n", addr);
			found++;
		}
		i2c_stop();
	}
	printf("fts: %u addresses answer\n", found);
}

/* ---- what a report holds -------------------------------------------------------------------------- */

/*
 * The vendor's fts_read_touchdata/fts_read_parse_touchdata: 63 bytes from register 0x00, of
 * which byte 2 counts the points and every point six bytes from byte 3 -- flags and x's
 * high half (bits 3..0), x's low half, the finger's id (bits 7..4) and y's high half
 * (bits 3..0), y's low half, the pressure, the area. A slot whose id is not one of the ten
 * fingers ends the points there; all 0xff from byte 2 on is the chip asking to be set up
 * again, which a run of this port has no way to do.
 */
static struct {
	bool down, seen;
	uint32_t x, y;
} contact[FTS_MAX_POINTS];

static struct {
	uint32_t reads, reports, points, last_x, last_y;
} seen;

static void
fts_points(const uint8_t *buf)
{
	if ((buf[2] & buf[3] & buf[4] & buf[5] & buf[6]) == 0xff) {
		bad_reads++;                 /* the chip's recovery state; the next read usually is a report again */
		return;
	}
	for (uint32_t id = 0; id < FTS_MAX_POINTS; id++) {
		contact[id].seen = false;
	}
	for (uint32_t i = 0; i < FTS_MAX_POINTS; i++) {
		uint32_t base = 6 * i;
		uint32_t id = buf[5 + base] >> 4;
		uint32_t event = buf[3 + base] >> 6;
		uint32_t x, y;

		if (id >= FTS_MAX_POINTS) {
			break;                   /* the slots after the points the report holds */
		}
		x = ((buf[3 + base] & 0x0f) << 8) | buf[4 + base];
		y = ((buf[5 + base] & 0x0f) << 8) | buf[6 + base];
		if (x >= PANEL_W) {
			x = PANEL_W - 1;
		}
		if (y >= PANEL_H) {
			y = PANEL_H - 1;
		}
		seen.points++;
		seen.last_x = x;
		seen.last_y = y;
		x = x * 65535 / (PANEL_W - 1);
		y = y * 65535 / (PANEL_H - 1);
		if (flip & 1) {
			x = 65535 - x;
		}
		if (flip & 2) {
			y = 65535 - y;
		}
		contact[id].seen = true;
		if (event == FTS_EVENT_UP) {
			if (contact[id].down) {
				contact[id].down = false;
				redwood_touch_post(REDWOOD_TOUCH_UP, id, x, y);
			}
			continue;
		}
		if (!contact[id].down) {
			contact[id].down = true;
			redwood_touch_post(REDWOOD_TOUCH_DOWN, id, x, y);
		} else if (contact[id].x != x || contact[id].y != y) {
			redwood_touch_post(REDWOOD_TOUCH_MOVE, id, x, y);
		}
		contact[id].x = x;
		contact[id].y = y;
	}
	for (uint32_t id = 0; id < FTS_MAX_POINTS; id++) {
		if (contact[id].down && !contact[id].seen) {
			contact[id].down = false;
			redwood_touch_post(REDWOOD_TOUCH_UP, id, contact[id].x, contact[id].y);
		}
	}
	seen.reports++;
}

static void
fts_poll(void)
{
	static uint8_t buf[FTS_DATA_LEN];

	if (!fts_read(0x00, buf, sizeof(buf))) {
		bad_reads++;
		if (bad_reads < 8 || (bad_reads & 0xff) == 0) {
			printf("fts: read failed (%u so far)\n", bad_reads);
		}
		return;
	}
	seen.reads++;
	fts_points(buf);
}

/* ---- the thread that watches the line ------------------------------------------------------------- */

static void
fts_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t poll_us = 8000, speed = 0;
	unsigned idle = 0, idle_looks;

	PE_parse_boot_argn("touch_poll_us", &poll_us, sizeof(poll_us));
	if (poll_us < 250 || poll_us > 100000) {
		poll_us = 8000;
	}
	idle_looks = 80000 / poll_us;   /* a report read every 80 ms whatever the line says */
	if (PE_parse_boot_argn("touch_i2c_half_us", &speed, sizeof(speed)) && speed >= 2 && speed <= 100) {
		half_us = speed;
	}

	gpio = ml_io_map(GPIO_PHYS, GPIO_SIZE);
	pin_mode_gpio(PIN_SCL);
	pin_mode_gpio(PIN_SDA);
	line_release(PIN_SCL);
	line_release(PIN_SDA);
	printf("fts: i2c0's lines released: scl %u, sda %u; reset %u, irq %u\n", pin_get(PIN_SCL), pin_get(PIN_SDA),
	    pin_get(PIN_RESET), pin_get(PIN_IRQ));

	if (!fts_probe(true)) {
		/* the pin the tree calls reset: pulsed once, in case it is wired after all */
		pin_mode_gpio(PIN_RESET);
		pin_output(PIN_RESET, false);
		sleep_ms(2);
		pin_output(PIN_RESET, true);
		sleep_ms(200);
		if (!fts_probe(true)) {
			i2c_who_answers();
			printf("fts: no touch screen\n");
			thread_terminate(current_thread());
			return;
		}
	}
	printf("fts: touch screen ready; the interrupt line is looked at every %u us, the bus at %u kHz\n",
	    poll_us, 500 / half_us);

	for (unsigned tick = 1;; tick++) {
		if (!pin_get(PIN_IRQ)) {
			idle = 0;
			fts_poll();
		} else if (++idle >= idle_looks) {
			idle = 0;
			fts_poll();
		}
		if (tick % (60 * (1000000 / poll_us)) == 0) {
			printf("fts: so far %u reports in %u reads (%u bad), %u finger positions, the last at %u, %u; "
			    "interrupt line %s\n", seen.reports, seen.reads, bad_reads, seen.points, seen.last_x,
			    seen.last_y, pin_get(PIN_IRQ) ? "high" : "low");
		}
		sleep_us(poll_us);
	}
}

void
redwood_fts_init(void)
{
	uint32_t which = 0;
	thread_t thread;

	/* 3 is this phone; 1 and 2 are the other two's, which have drivers of their own beside this. */
	if (!PE_parse_boot_argn("touchscreen", &which, sizeof(which)) || which != 3) {
		return;
	}
	PE_parse_boot_argn("touch_flip", &flip, sizeof(flip));
	if (kernel_thread_start(fts_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		printf("fts: cannot start the thread that reads the touch screen\n");
	}
}
