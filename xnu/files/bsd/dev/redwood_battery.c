/*
 * Battery monitor for Xiaomi redwood (POCO X5 Pro).
 *
 * Redwood selects PM7325B and qcom,battery-charger over PMIC GLINK/ADSP, not
 * the local PM7250B/QG layout.  The resident UEFI contains the matching
 * Qualcomm charger drivers.  xnuloader owns that firmware context on core 0,
 * polls it, and publishes a read-only cache; XNU only consumes that cache.
 * Charger limits and JEITA protection therefore remain firmware-owned.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/redwood_input.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

#define LOADER_BATTERY_MAGIC            0x2154544142574452ULL
#define LOADER_BATTERY_VERSION          1
#define LOADER_BATTERY_VALID            (1u << 0)
#define LOADER_BATTERY_PRESENT          (1u << 1)
#define LOADER_BATTERY_CHARGER_PRESENT  (1u << 2)
#define LOADER_BATTERY_VOLTAGE_VALID    (1u << 3)
#define LOADER_BATTERY_CHARGING         (1u << 4)
#define LOADER_BATTERY_SOC_ESTIMATED    (1u << 5)
#define LOADER_PAGE_SIZE                0x4000UL

struct loader_battery {
	uint64_t magic;
	uint32_t version, size;
	volatile uint32_t sequence;
	uint32_t flags;
	int32_t soc_percent, voltage_mv, current_ma, temperature_c;
	uint32_t charger_action, polls, failures, reserved;
};

static volatile struct loader_battery *shared_battery;
static int battery_soc_percent = -1, battery_voltage_mv = -1, battery_current_ma;
static int battery_temperature_c = -273, battery_present, battery_charger_present;
static int battery_charging, battery_soc_estimated, battery_charger_action = -1;
static uint64_t battery_updates, battery_failures;

/* for others in the kernel (the flashlight's thermal guard): -273 while it is not known */
int
redwood_battery_temperature(void)
{
	return battery_temperature_c;
}

SYSCTL_NODE(_kern, OID_AUTO, redwood, CTLFLAG_RW | CTLFLAG_LOCKED, 0, "redwood phone bits");
SYSCTL_INT(_kern_redwood, OID_AUTO, battery_soc, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_soc_percent, 0, "battery state of charge, 0..100 percent, or -1 if unknown");
SYSCTL_INT(_kern_redwood, OID_AUTO, battery_voltage_mv, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_voltage_mv, 0, "battery voltage in millivolts, or -1 if unknown");
SYSCTL_INT(_kern_redwood, OID_AUTO, battery_current_ma, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_current_ma, 0, "battery current reported by Qualcomm firmware, in milliamps");
SYSCTL_INT(_kern_redwood, OID_AUTO, battery_temperature_c, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_temperature_c, 0, "battery temperature reported by Qualcomm firmware, in Celsius");
SYSCTL_INT(_kern_redwood, OID_AUTO, battery_present, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_present, 0, "1 if Qualcomm firmware reports a battery");
SYSCTL_INT(_kern_redwood, OID_AUTO, battery_charger_present, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_charger_present, 0, "1 if Qualcomm firmware reports an attached charger");
SYSCTL_INT(_kern_redwood, OID_AUTO, battery_charging, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_charging, 0, "1 while an attached charger is charging a non-full battery");
SYSCTL_INT(_kern_redwood, OID_AUTO, battery_soc_estimated, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_soc_estimated, 0, "1 only when SOC had to be estimated from voltage");
SYSCTL_INT(_kern_redwood, OID_AUTO, battery_charger_action, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_charger_action, 0, "last Qualcomm charger action, for diagnostics");

static bool
battery_snapshot(struct loader_battery *out)
{
	uint32_t before, after;

	for (unsigned retry = 0; retry < 4; retry++) {
		before = shared_battery->sequence;
		if (before & 1) continue;
		__asm__ volatile ("dmb ishld" ::: "memory");
		memcpy(out, (const void *)(uintptr_t)shared_battery, sizeof(*out));
		__asm__ volatile ("dmb ishld" ::: "memory");
		after = shared_battery->sequence;
		if (before == after && !(after & 1)) return true;
	}
	return false;
}

static void
battery_poll(void)
{
	struct loader_battery sample;

	if (!battery_snapshot(&sample) || sample.magic != LOADER_BATTERY_MAGIC ||
	    sample.version != LOADER_BATTERY_VERSION || sample.size < sizeof(sample) ||
	    !(sample.flags & LOADER_BATTERY_VALID) || sample.soc_percent < 0 || sample.soc_percent > 100) {
		battery_failures++;
		return;
	}
	battery_soc_percent = sample.soc_percent;
	battery_voltage_mv = (sample.flags & LOADER_BATTERY_VOLTAGE_VALID) ? sample.voltage_mv : -1;
	battery_current_ma = sample.current_ma;
	battery_temperature_c = sample.temperature_c;
	battery_present = (sample.flags & LOADER_BATTERY_PRESENT) != 0;
	battery_charger_present = (sample.flags & LOADER_BATTERY_CHARGER_PRESENT) != 0;
	battery_charging = (sample.flags & LOADER_BATTERY_CHARGING) != 0;
	battery_soc_estimated = (sample.flags & LOADER_BATTERY_SOC_ESTIMATED) != 0;
	battery_charger_action = (int)sample.charger_action;
	battery_updates++;
}

/*
 * What the audio DSP's battery manager says (bsd/dev/redwood_adsp.c, with adsp=1 unless battmgr=0):
 * the values that Android shows, from where Android takes them. They are kept here for the sysctls
 * and passed on to the power source that iOS reads (iokit/Kernel/RedwoodBattery.cpp).
 */
static int battery_full_uah, battery_design_uah, battery_cycles, battery_source_dsp;

SYSCTL_INT(_kern_redwood, OID_AUTO, battery_full_uah, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_full_uah, 0, "the battery's full charge as the fuel gauge has learnt it, in uAh (0 if unknown)");
SYSCTL_INT(_kern_redwood, OID_AUTO, battery_design_uah, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_design_uah, 0, "the battery's charge as designed, in uAh (0 if unknown)");
SYSCTL_INT(_kern_redwood, OID_AUTO, battery_cycles, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_cycles, 0, "the battery's charge cycles (0 if unknown)");
SYSCTL_INT(_kern_redwood, OID_AUTO, battery_from_dsp, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_source_dsp, 0, "1 the audio DSP's battery manager; 2 this kernel's own reading (battery=3)");

extern void RedwoodBatteryReport(const void *state, unsigned size);     /* iokit/Kernel/RedwoodBattery.cpp */

void
redwood_battery_report(const struct redwood_battery_state *b)
{
	int percent = (b->percent_hundredths + 50) / 100;

	battery_soc_percent = percent < 0 ? 0 : percent > 100 ? 100 : percent;
	battery_voltage_mv = b->voltage_uv / 1000;
	battery_current_ma = b->current_ua / 1000;
	battery_temperature_c = (b->temperature_hundredths + 50) / 100;
	battery_present = b->present != 0;
	battery_charger_present = b->external != 0;
	battery_charging = b->status == 1;      /* POWER_SUPPLY_STATUS_CHARGING */
	battery_soc_estimated = 0;
	battery_charger_action = b->status;
	if (b->full_uah > 0) {
		battery_full_uah = b->full_uah;
	}
	if (b->design_uah > 0) {
		battery_design_uah = b->design_uah;
	}
	if (b->cycles > 0) {
		battery_cycles = b->cycles;
	}
	battery_source_dsp = 1;
	battery_updates++;
	RedwoodBatteryReport(b, sizeof(*b));
}

static void
battery_thread(__unused void *parameter, __unused wait_result_t result)
{
	for (unsigned tick = 0;; tick++) {
		battery_poll();
		if (tick % 12 == 0) {
			printf("battery: SOC=%d%%%s, V=%d mV, I=%d mA, T=%d C, battery=%d charger=%d charging=%d "
			    "action=%d (%llu reads, %llu failed)\n", battery_soc_percent,
			    battery_soc_estimated ? " (voltage estimate)" : "", battery_voltage_mv,
			    battery_current_ma, battery_temperature_c, battery_present, battery_charger_present,
			    battery_charging, battery_charger_action, battery_updates, battery_failures);
		}
		delay_for_interval(5000, NSEC_PER_MSEC);
	}
}

/* ---- the Redmi Note 10S (boot-arg battery=3): this phone's own battery, charger and port ---------- */

/*
 * rosemary keeps its battery in the MT6359P PMIC and its charging in the MT6360 companion,
 * and this driver reads both, through the PMIC wrapper the buttons already use. The
 * voltages are the PMIC's AUXADC, taken the way Linux's drivers/iio/adc/mt635x-auxadc.c
 * takes them: the channel's request bit written to RQST0 (0x1108), a wait of the average's
 * worth (the phone's mt6359p.dtsi asks batadc for 128 samples of 10 us), then the channel's
 * out register watched until its bit 15 comes, the value the 12 or 15 bits under it -- the
 * out registers 0x1088 (BATADC), 0x108a (ISENSE), 0x108c (VCDT, the USB port's), 0x108e
 * (BAT_TEMP), every address from include/linux/mfd/mt6359p/registers.h, every channel
 * number from include/dt-bindings/iio/mt635x-auxadc.h. At its start Linux also toggles the
 * AUXADC's reset and its two enable bits (auxadc_reset of that file: HK_TOP_RST_CON0 0xf90
 * bits 3 and 0, HK_TOP_WKEY 0xfb4 cleared, RQST0 bit 7, RQST1 0x110a bit 6); the same is
 * done here. The millivolts are the driver's formula val = raw * r0 * 1800 / r1 >> res
 * with the dividers the phone's dtsi gives: batadc <7 2> over 15 bits, bat_temp <5 2> over
 * 12; VCDT is left at <1 1> over 12, which cannot hold a port's 5 V, so its raw word is
 * kept in a sysctl of its own -- the divider it goes through will be read off a live port.
 * A charger on the port the PMIC itself says: CHR_CON0 (0xa88) bit 0, RGS_CHRDET. What the
 * charger is doing is the MT6360's, on i2c5 at 0x34 (pmu_client.addr of
 * pmic/mt6360/ldo/mt6360_ldo_i2c.c; the companion of the 0x64 the touch's supply rides):
 * CHG_STAT (0x4a) bits 7:6 are 0 standby, 1 charging, 2 done, 3 fault
 * (inc/mt6360_pmu_chg.h), CHG_STAT1 (0xe0) bit 7 the port's power good; the i2c is driven
 * by hand, redwood_fts.c's dialect, a bus the touch leaves alone after its init. The
 * charge curve is the phone's own, battery0_profile_t0 of bat_setting/
 * mt6785_battery_table.dtsi: a hundred rows a percent apart, 4412 mV at full down to
 * 2857 mV, its middle column in tenth-millivolts read here in millivolts. The battery's
 * temperature is the thermistor's millivolts through the mt6785 platform's own 100K NTC
 * table (mtk_battery_table.h), and the charger's its ADC's junction reading -- both in
 * the log now, in degrees. The percent and the current are
 * the fuel gauge's own, the FGADC block below: the counter's coulombs from a voltage
 * anchor, the averaged current beside the voltage in the log. The charger's currents the
 * MT6360's registers set, when the boot-args chg_aicr and chg_ichg ask for them: the
 * input's limit and the battery's own current, conservative steps from half an ampere.
 * The one voltage ever written is the ceiling, and only when chg_cv asks for it -- the
 * battery's own hundred percent where Android fills it to; the port's own negotiation
 * is a story of its own. That story begins here too, read-only: the same companion's
 * TCPC half (the standard TCPCI v1.0 register map at another address of the same bus)
 * is the port's negotiator, and pd_probe opens eyes on it -- its identity, its state,
 * and every offer the brick makes, said and nothing more until negotiation is written.
 */
#define RM_GPIO_PHYS        0x10005000
#define RM_DIR_AT(pin)      (0x0000 + 0x10 * ((pin) / 32))
#define RM_DOUT_AT(pin)     (0x0100 + 0x10 * ((pin) / 32))
#define RM_DIN_AT(pin)      (0x0200 + 0x10 * ((pin) / 32))
#define RM_MODE_AT(pin)     (0x0300 + 0x10 * ((pin) / 8))
#define RM_MODE_BIT(pin)    (((pin) % 8) * 4)
#define RM_PWR_SCL          28              /* i2c5's clock, the MT6360 companion's bus */
#define RM_PWR_SDA          29              /* i2c5's data */

#define MT6359P_AUXADC_ADC0     0x1088      /* BATADC; ISENSE, VCDT, BAT_TEMP every two */
#define MT6359P_AUXADC_RQST0    0x1108      /* a channel's bit: 0 BATADC, 1 ISENSE, 2 VCDT, 3 BAT_TEMP */
#define MT6359P_AUXADC_RQST1    0x110a
#define MT6359P_HK_TOP_RST_CON0 0xf90       /* the AUXADC's reset, bits 3 and 0 */
#define MT6359P_HK_TOP_WKEY     0xfb4
#define MT6359P_CHR_CON0        0xa88       /* bit 0: RGS_CHRDET, a charger on the port */

/* The fuel gauge the PMIC carries (FGADC, mt6359_gauge.c of the phone's kernel): a coulomb
 * counter (CAR) and an averaged current (IAVG), the counter behind a latch -- the request
 * bit 0 of CON1 latches a reading, bit 15 says it is done, bit 3 lets it go, and its words
 * are CAR_CON0's low one and CAR_CON1's high fifteen, thirty-one counts of 0.085 uAh each,
 * the high word's top bit the discharge side and the count running straight through zero
 * between charging and giving back. The current is twenty-eight bits of its own, each
 * 305.176 uA, charging positive, no latch asked of it. This is where Android takes its
 * percent from, and battery=3 does the same now: one anchor at the start -- the open-circuit
 * voltage the table turns into percent, the current's IR (a hundred milliohms, the table's
 * own 73-to-160 column) taken off the terminal voltage first -- and the counter's coulombs
 * after that, each count 0.085 uAh into the battery's 4952000, so the percent stays put when
 * a cable goes in or out and only moves as the battery actually fills or empties. A counter
 * that does not move at a current that should have moved it is a gauge gone asleep, and the
 * anchor is taken again from the voltage until it comes back. The gauge's switch is CON0's
 * bit 0, on since Android's last boot; it is checked and set anyway. */
#define MT6359P_FGADC_CON0      0xd08       /* bit 0: the gauge's switch */
#define MT6359P_FGADC_CON1      0xd0a       /* bit 0 latches, bit 3 clears, bit 15 done */
#define MT6359P_FGADC_CAR_CON0  0xd16       /* the coulomb counter's low word */
#define MT6359P_FGADC_CAR_CON1  0xd18       /* ... and high one, top bit the discharge side */
#define MT6359P_FGADC_IAVG_CON0 0xd2c       /* the average current's low word */
#define MT6359P_FGADC_IAVG_CON1 0xd2e       /* 12 bits more; bit 11 the sign, bit 15 valid */
#define ROSEMARY_QMAX_UAH       4952000     /* the table's own span: 4952 mAh */
#define ROSEMARY_R_BAT_MOHM     100         /* the battery's own resistance, the table's 73 to 160 */

#define MT6360_PMU_ADDR     0x34            /* the charger half of the companion, on i2c5 */
#define MT6360_CHG_STAT     0x4a            /* bits 7:6: 0 standby, 1 charging, 2 done, 3 fault */
#define MT6360_CHG_STAT1    0xe0            /* bit 7: the port's power good */
#define MT6360_CHG_CTRL2    0x12            /* bit 0 the charger's own switch; bits 3:2 where the input's limit comes from, 2 the AICR register */
#define MT6360_CHG_CTRL3    0x13            /* bits 7:2 the input's limit: 100 mA and 50 mA a step to 3250; bit 0 the limit applied */
#define MT6360_CHG_CTRL4    0x14            /* bits 7:1 the battery's ceiling: 3900 mV and 10 mV a step -- written only by the boot-arg chg_cv, to the battery's own hundred percent */
#define MT6360_CHG_CTRL6    0x16            /* bits 7:1 the port's droop floor: 3900 mV and 100 mV a step, the input's own guard */
#define MT6360_CHG_CTRL7    0x17            /* bits 7:2 the battery's current: 100 mA and 100 mA a step to 5 A */
#define MT6360_CHG_CTRL13   0x1d            /* bit 7 the charger's watchdog, nobody here to pet it -- said in the log */

/* The MT6360's own ADC (mt6360_pmu_adc.c, the same i2c address as the charger): the channel
 * asked of RPT_1's bits 7:4, the ADC's switch and the channel's bit written to CONFIG, the
 * report the three bytes at RPT_1..3 once its low nibble names the channel asked of it, the
 * conversion done in well under a tenth of a second. Teardown is the driver's: no channel
 * left asked for, the ADC itself left on. The channels, and what a count of each is worth:
 * the port's voltage a fifth of it, 6.25 mV a count; the battery's 1.25 mV; the port's
 * current 1.9 mA a count under a 400 mA input's limit and 2.5 mA above it; the battery's
 * current 2.5 mA a count, its sign the top bit (an assumption this log will check against
 * the gauge's own current); the charger's junction (counts*105-8000)/100 centigrade. */
#define MT6360_ADC_CONFIG   0x56            /* bit 7 the ADC's switch; channels 8 to 15 a bit each below */
#define MT6360_ADC_RPT1     0x5a            /* the wanted channel in bits 7:4, the reported one in bits 3:0 */
#define MT6360_ADC_VBUSDIV5 1               /* the port's voltage, a fifth of it */
#define MT6360_ADC_VBAT     4               /* the battery's, at the charger's own pin */
#define MT6360_ADC_IBUS     5               /* the port's current */
#define MT6360_ADC_IBAT     6               /* the battery's, at the charger's own pin */
#define MT6360_ADC_TEMP_JC  8               /* the charger's own junction, in centigrade */

/* The port's own negotiator: the TCPC half of the same MT6360, at the companion's other
 * address (0x4e, as the upstream mt6360 core hands it out). Its register map is the
 * standard TCPCI v1.0 one (std_tcpci_v10.h of the vendor tree) over the vendor's own
 * extensions at 0x80 and above (mt6360.h). This is read-only eyes yet: pd_probe asks for
 * the identification and the status registers once, and then watches the receive alert --
 * a message waiting in the FIFO is read, said, and its alert handed back cleared, the
 * get_message handshake of tcpc_mt6360.c with nothing else touched. The PD message format
 * itself: a 16-bit header (the type in bits 4:0, the count of objects in 14:12), then the
 * objects; a source's capabilities are data type 1, each object a power offer. */
#define TCPC_ADDR           0x4e            /* the negotiator half of the companion, on i2c5 */
#define TCPC_VID            0x00            /* the identity the vendor's tree says: 0x29cf */
#define TCPC_PID            0x02            /* 0x6360 */
#define TCPC_DID            0x04
#define TCPC_ALERT          0x10            /* 16 bits, write-one-to-clear */
#define TCPC_ALERT_MASK     0x12            /* 16 bits */
#define TCPC_CTRL           0x19
#define TCPC_ROLE_CTRL      0x1a
#define TCPC_POWER_CTRL     0x1c
#define TCPC_CC_STATUS      0x1d
#define TCPC_POWER_STATUS   0x1e
#define TCPC_MSG_HDR_INFO   0x2e
#define TCPC_RX_DETECT      0x2f            /* the PD receiver's switch: bits for the SOPs */
#define TCPC_RX_BYTE_CNT    0x30            /* then the frame type, then the header's two */
#define TCPC_RX_DATA        0x34            /* the objects, up to twenty-eight bytes */
#define TCPC_MTK_MODE_CTRL3 0x90            /* the vendor's: the PD bias and the oscillator */
#define TCPC_MTK_CLK_CTRL1  0x87            /* the PD clocks: un-gated 0xca (the vendor's own value) */
#define TCPC_MTK_CLK_CTRL2  0x88            /* un-gated 0x54: the 24M and the protocol clocks */
#define TCPC_RX_DETECT_ON   0x21            /* SOP messages and hard resets: a listening sink's ear */
#define TCPC_ALERT_RX_STATUS (1u << 2)      /* a message waits in the FIFO */
#define TCPC_ALERT_RX_OVERFLOW (1u << 10)   /* the FIFO overflowed; the oldest are lost */

/* boot-args chg_aicr / chg_ichg / chg_cv, in milliamperes and millivolts: the currents the
 * charger is given when the battery is asked to actually fill, and the ceiling it fills to
 * -- 0 leaves whatever the bootloader left. The ceiling (CTRL4) is the one voltage ever
 * written here, and only for the battery's own good: the phone's curve tops at 4412 mV and
 * Android itself charges to 4420, where the bootloader's 4350 stops the fill at ninety-four
 * percent. The port's droop floor (CTRL6) and every other voltage stay as found. */
static int chg_aicr_ma, chg_ichg_ma, chg_cv_mv;

/* pd_probe: read-only eyes on the port's negotiator -- the identification and the status
 * registers once, then every message the brick sends, read and said and nothing more.
 * pd_listen, one step further: the receiver's switch on -- the vendor's own set_rx_enable
 * sequence, the clocks un-gated then RX_DETECT set to SOP|HARD_RESET, its value for a
 * listening sink. Nothing else changes: no message is ever sent, the hardware's own
 * GoodCRC answers the brick (the TCPC's protocol layer does that alone), and without a
 * Request from here the brick keeps VBUS at five volts. */
static int pd_probe, pd_listen;
static bool tcpc_ok;
static unsigned tcpc_rx_seen;

static vm_offset_t rm_gpio;
static bool rm_pushpull;                    /* i2c5's ones driven instead of only released */
static int battery_vbus_mv = -1, battery_vcdt_raw = -1, battery_ntc_mv;

extern void redwood_host_keep_add(const char *format, ...) __printflike(1, 2);   /* redwood_mali.c: a line the log's middle cannot lose */

SYSCTL_INT(_kern_redwood, OID_AUTO, battery_vbus_mv, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_vbus_mv, 0, "the USB port's voltage as the PMIC's VCDT channel sees it, in millivolts");
SYSCTL_INT(_kern_redwood, OID_AUTO, battery_vcdt_raw, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_vcdt_raw, 0, "the VCDT channel's raw word, whose scale a live port will tell");
SYSCTL_INT(_kern_redwood, OID_AUTO, battery_ntc_mv, CTLFLAG_RD | CTLFLAG_LOCKED,
    &battery_ntc_mv, 0, "the battery's thermistor as the PMIC's BAT_TEMP channel sees it, in millivolts");

/* The charge curve, battery0_profile_t0 of the phone's bat_setting/mt6785_battery_table.dtsi:
 * the voltage of each percent, 100 rows from full at index 0 to empty at 99. */
static const uint16_t rosemary_soc_mv[100] = {
	4412, 4401, 4390, 4379, 4369, 4358, 4348, 4337, 4325, 4314,
	4302, 4290, 4278, 4266, 4254, 4242, 4230, 4217, 4205, 4193,
	4181, 4169, 4157, 4145, 4134, 4122, 4110, 4099, 4088, 4077,
	4067, 4056, 4046, 4036, 4026, 4016, 4007, 3997, 3987, 3977,
	3964, 3950, 3935, 3923, 3913, 3904, 3897, 3889, 3883, 3876,
	3869, 3863, 3857, 3852, 3847, 3842, 3836, 3831, 3827, 3823,
	3818, 3814, 3810, 3807, 3803, 3800, 3796, 3791, 3784, 3776,
	3768, 3763, 3758, 3753, 3748, 3742, 3737, 3734, 3730, 3724,
	3717, 3711, 3704, 3698, 3690, 3684, 3681, 3681, 3680, 3678,
	3677, 3668, 3643, 3603, 3550, 3482, 3390, 3226, 2880, 2857,
};

/* The percent the voltage says, in hundredths: row i is (100 - i) percent, and a voltage
 * between two rows takes its share of the row above. */
static int
rosemary_soc(uint32_t vbat_mv)
{
	if (vbat_mv >= rosemary_soc_mv[0]) {
		return 10000;
	}
	for (int i = 1; i < 100; i++) {
		if (vbat_mv > rosemary_soc_mv[i]) {
			uint32_t span = (uint32_t)rosemary_soc_mv[i - 1] - rosemary_soc_mv[i];

			return (100 - i) * 100 + (span ? (vbat_mv - rosemary_soc_mv[i]) * 100 / span : 0);
		}
	}
	return 0;
}

/* One AUXADC channel, the Linux way: request, wait for the average, take the ready word. */
static bool
rosemary_auxadc(uint32_t channel, uint32_t mask, uint32_t *raw)
{
	uint32_t out = 0;

	if (!redwood_pmic_write(MT6359P_AUXADC_RQST0, 1u << channel)) {
		return false;
	}
	delay_for_interval(2, NSEC_PER_MSEC);       /* 128 samples of 10 us, with room to spare */
	for (uint32_t spin = 0; spin < 100; spin++) {
		if (!redwood_pmic_read(MT6359P_AUXADC_ADC0 + 2 * channel, &out)) {
			return false;
		}
		if (out & 0x8000) {
			*raw = out & mask;
			return true;
		}
		delay_for_interval(100, NSEC_PER_USEC);
	}
	return false;
}

/* The coulomb counter through its latch, the mt6359_gauge.c way: the request bit set, the
 * done bit waited for, the two words taken, the latch let go. The wrapper writes a register
 * whole, so every step is the read-modify-write Linux's own pmic_config_interface does. */
static bool
rosemary_fg_car(int64_t *counts)
{
	uint32_t con1 = 0, lo = 0, hi = 0;
	bool done = false;

	if (!redwood_pmic_read(MT6359P_FGADC_CON1, &con1) ||
	    !redwood_pmic_write(MT6359P_FGADC_CON1, con1 | 0x0001)) {
		return false;
	}
	for (uint32_t spin = 0; spin < 50; spin++) {
		if (!redwood_pmic_read(MT6359P_FGADC_CON1, &con1)) {
			return false;
		}
		if (con1 & 0x8000) {
			done = true;
			break;
		}
		delay_for_interval(100, NSEC_PER_USEC);
	}
	if (!done || !redwood_pmic_read(MT6359P_FGADC_CAR_CON0, &lo) ||
	    !redwood_pmic_read(MT6359P_FGADC_CAR_CON1, &hi)) {
		(void)redwood_pmic_write(MT6359P_FGADC_CON1, con1 & ~0x0009u);
		return false;
	}
	uint32_t raw31 = (hi & 0x7fff) << 16 | (lo & 0xffff);

	*counts = (hi & 0x8000) ? (int64_t)raw31 - 0x7fffffff : (int64_t)raw31;

	/* let the latch go: the clear bit set with the request dropped, the done bit waited out */
	(void)redwood_pmic_write(MT6359P_FGADC_CON1, (con1 | 0x0008) & ~0x0001u);
	for (uint32_t spin = 0; spin < 50; spin++) {
		uint32_t drop = 0;

		if (!redwood_pmic_read(MT6359P_FGADC_CON1, &drop) || !(drop & 0x8000)) {
			break;
		}
		delay_for_interval(100, NSEC_PER_USEC);
	}
	(void)redwood_pmic_write(MT6359P_FGADC_CON1, con1 & ~0x0009u);
	return true;
}

/* The gauge's averaged current: twenty-eight bits across the two words, the high one's bit 11
 * the sign (set means the battery giving its charge away) and its bit 15 the words being
 * valid, each count 305.176 uA -- the driver's own arithmetic with its car_tune of 100. */
static bool
rosemary_fg_iavg(int64_t *ua)
{
	uint32_t lo = 0, hi = 0;

	if (!redwood_pmic_read(MT6359P_FGADC_IAVG_CON0, &lo) ||
	    !redwood_pmic_read(MT6359P_FGADC_IAVG_CON1, &hi) || !(hi & 0x8000)) {
		return false;
	}
	uint32_t raw = (hi & 0xfff) << 16 | (lo & 0xffff);
	int64_t signed_raw = (hi & 0x800) ? (int64_t)raw - 0x10000000 : (int64_t)raw;

	*ua = signed_raw * 305176 / 1000;
	return true;
}

/* The battery's thermistor: a 100K NTC pulled up by 100K from 2800 mV, the mt6785's own
 * platform table (mtk_battery_table.h, BAT_NTC_100: "we use 100K Rp and 100K NTC"; the
 * resistance from the millivolts the way battery_meter.c has it, R = V * Rpu / (2800 - V)).
 * The rows are the resistance in kilo-ohms every five degrees, -40 to 70 C -- the phone's
 * own dts may lay a different table over the driver's, but the evening's readings (1331 mV
 * after a day of runs) land where a warm room should. */
static const uint16_t rosemary_ntc_kohm[23] = {
	4251, 3005, 2149, 1554, 1135, 838, 624, 469, 356, 272, 209, 163,
	127, 100, 79, 63, 51, 41, 33, 27, 22, 18, 15,
};

static int
rosemary_ntc_c(int ntc_mv)
{
	int r;

	if (ntc_mv <= 0 || ntc_mv >= 2800) {
		return -273;
	}
	r = ntc_mv * 100 / (2800 - ntc_mv);      /* kilo-ohms */
	for (int i = 1; i < 23; i++) {
		if (r >= rosemary_ntc_kohm[i]) {
			int r1 = rosemary_ntc_kohm[i - 1], r2 = rosemary_ntc_kohm[i];

			return -40 + 5 * (i - 1) + (r1 - r) * 5 / (r1 - r2);
		}
	}
	return 70;
}

/* ---- i2c5 by hand, for the MT6360: the touch's own copy of this is busy on i2c0 -------------------- */

static inline uint32_t
rm_rd(uint32_t at)
{
	return *(volatile uint32_t *)(rm_gpio + at);
}

static inline void
rm_wr(uint32_t at, uint32_t value)
{
	*(volatile uint32_t *)(rm_gpio + at) = value;
}

static void
rm_line_low(unsigned pin)
{
	rm_wr(RM_DOUT_AT(pin), rm_rd(RM_DOUT_AT(pin)) & ~(1u << (pin % 32)));
	rm_wr(RM_DIR_AT(pin), rm_rd(RM_DIR_AT(pin)) | 1u << (pin % 32));
}

static void
rm_line_release(unsigned pin)
{
	rm_wr(RM_DIR_AT(pin), rm_rd(RM_DIR_AT(pin)) & ~(1u << (pin % 32)));
}

static void
rm_line_high(unsigned pin)
{
	if (rm_pushpull) {
		rm_wr(RM_DOUT_AT(pin), rm_rd(RM_DOUT_AT(pin)) | 1u << (pin % 32));
		rm_wr(RM_DIR_AT(pin), rm_rd(RM_DIR_AT(pin)) | 1u << (pin % 32));
	} else {
		rm_line_release(pin);
	}
}

static inline bool
rm_sda(void)
{
	return (rm_rd(RM_DIN_AT(RM_PWR_SDA)) >> (RM_PWR_SDA % 32)) & 1u;
}

static void
rm_half_clock(void)
{
	(void)rm_rd(RM_DIN_AT(RM_PWR_SDA));
	(void)rm_rd(RM_DIN_AT(RM_PWR_SDA));
	delay_for_interval(1, NSEC_PER_USEC);
}

static bool
rm_i2c_start(void)
{
	rm_line_high(RM_PWR_SDA);
	rm_line_high(RM_PWR_SCL);
	rm_half_clock();
	if (!rm_sda()) {
		return false;               /* somebody holds the line down */
	}
	rm_line_low(RM_PWR_SDA);
	rm_half_clock();
	rm_line_low(RM_PWR_SCL);
	rm_half_clock();
	return true;
}

static void
rm_i2c_stop(void)
{
	rm_line_low(RM_PWR_SDA);
	rm_half_clock();
	rm_line_high(RM_PWR_SCL);
	rm_half_clock();
	rm_line_high(RM_PWR_SDA);
	rm_half_clock();
}

static void
rm_i2c_restart(void)
{
	rm_line_high(RM_PWR_SDA);
	rm_half_clock();
	rm_line_high(RM_PWR_SCL);
	rm_half_clock();
	rm_line_low(RM_PWR_SDA);
	rm_half_clock();
	rm_line_low(RM_PWR_SCL);
	rm_half_clock();
}

static bool
rm_i2c_out(uint8_t byte)
{
	bool ack;

	for (int bit = 7; bit >= 0; bit--) {
		if (byte >> bit & 1) {
			rm_line_high(RM_PWR_SDA);
		} else {
			rm_line_low(RM_PWR_SDA);
		}
		rm_half_clock();
		rm_line_high(RM_PWR_SCL);
		rm_half_clock();
		rm_line_low(RM_PWR_SCL);
		rm_half_clock();
	}
	rm_line_release(RM_PWR_SDA);
	rm_half_clock();
	rm_line_high(RM_PWR_SCL);
	rm_half_clock();
	ack = !rm_sda();
	rm_line_low(RM_PWR_SCL);
	rm_half_clock();
	return ack;
}

static uint8_t
rm_i2c_in(bool ack)
{
	uint8_t byte = 0;

	rm_line_release(RM_PWR_SDA);
	for (int bit = 7; bit >= 0; bit--) {
		rm_half_clock();
		rm_line_high(RM_PWR_SCL);
		rm_half_clock();
		byte = (uint8_t)(byte << 1 | (rm_sda() ? 1 : 0));
		rm_line_low(RM_PWR_SCL);
	}
	if (ack) {
		rm_line_low(RM_PWR_SDA);    /* the receiver's acknowledgement: another byte follows */
	} else {
		rm_line_release(RM_PWR_SDA);  /* no acknowledgement: the last byte */
	}
	rm_half_clock();
	rm_line_high(RM_PWR_SCL);
	rm_half_clock();
	rm_line_low(RM_PWR_SCL);
	rm_half_clock();
	rm_line_release(RM_PWR_SDA);
	return byte;
}

/* One register of any companion on the bus, by its address -- the charger and the port's
 * negotiator are two halves of the same MT6360 at two addresses. The plain i2c the
 * vendor's own drivers speak (mt6360_pmu_i2c.c, tcpc_mt6360.c: no checksum anywhere),
 * the blocks their block transfers, the words little-endian as both read them. */
static bool
rm_dev_read(uint8_t addr, uint8_t reg, uint8_t *value)
{
	if (!rm_i2c_start() || !rm_i2c_out(addr << 1) || !rm_i2c_out(reg)) {
		rm_i2c_stop();
		return false;
	}
	rm_i2c_restart();
	if (!rm_i2c_out(addr << 1 | 1)) {
		rm_i2c_stop();
		return false;
	}
	*value = rm_i2c_in(false);
	rm_i2c_stop();
	return true;
}

static bool
rm_dev_read_block(uint8_t addr, uint8_t reg, uint8_t *buf, int len)
{
	if (!rm_i2c_start() || !rm_i2c_out(addr << 1) || !rm_i2c_out(reg)) {
		rm_i2c_stop();
		return false;
	}
	rm_i2c_restart();
	if (!rm_i2c_out(addr << 1 | 1)) {
		rm_i2c_stop();
		return false;
	}
	for (int i = 0; i < len; i++) {
		buf[i] = rm_i2c_in(i + 1 < len);
	}
	rm_i2c_stop();
	return true;
}

static bool
rm_dev_write(uint8_t addr, uint8_t reg, uint8_t value)
{
	if (!rm_i2c_start() || !rm_i2c_out(addr << 1) || !rm_i2c_out(reg) ||
	    !rm_i2c_out(value)) {
		rm_i2c_stop();
		return false;
	}
	rm_i2c_stop();
	return true;
}

static bool
rm_dev_write_block(uint8_t addr, uint8_t reg, const uint8_t *buf, int len)
{
	if (!rm_i2c_start() || !rm_i2c_out(addr << 1) || !rm_i2c_out(reg)) {
		rm_i2c_stop();
		return false;
	}
	for (int i = 0; i < len; i++) {
		if (!rm_i2c_out(buf[i])) {
			rm_i2c_stop();
			return false;
		}
	}
	rm_i2c_stop();
	return true;
}

/* One register of the MT6360's charger half, the plain i2c the vendor's own driver speaks
 * on this address (mt6360_pmu_i2c.c reads it with i2c_smbus, no checksum anywhere). */
static bool
mt6360_read(uint8_t reg, uint8_t *value)
{
	return rm_dev_read(MT6360_PMU_ADDR, reg, value);
}

/* A register of the charger written whole, and the read-modify-write the vendor's own driver
 * does for its fields (mt6360_pmu_reg_update_bits). */
static bool
mt6360_write(uint8_t reg, uint8_t value)
{
	return rm_dev_write(MT6360_PMU_ADDR, reg, value);
}

static bool
mt6360_update(uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t v = 0;

	if (!mt6360_read(reg, &v)) {
		return false;
	}
	return mt6360_write(reg, (uint8_t)((v & ~mask) | (value & mask)));
}

/* One channel of the charger's own ADC, the mt6360_pmu_adc.c way without its interrupt:
 * the channel asked of the report's selector, the ADC and the channel switched on, the
 * report watched until its low nibble names the channel (the cycle never asks twice in a
 * row, so the name means fresh), and the teardown the driver leaves behind. */
static bool
mt6360_adc(uint8_t channel, int32_t *raw)
{
	uint8_t cfg_hi = 0x80, cfg_lo = 0, rpt0 = 0, hi = 0, lo = 0;
	bool done = false;

	if (channel >= 8) {
		cfg_hi |= (uint8_t)(1u << (channel - 8));
	} else {
		cfg_lo = (uint8_t)(1u << channel);
	}
	if (!mt6360_update(MT6360_ADC_RPT1, 0xf0, channel << 4) ||
	    !mt6360_write(MT6360_ADC_CONFIG, cfg_hi) || !mt6360_write(MT6360_ADC_CONFIG + 1, cfg_lo)) {
		return false;
	}
	delay_for_interval(25, NSEC_PER_MSEC);   /* the conversion's own time */
	for (uint32_t spin = 0; spin < 15; spin++) {
		if (!mt6360_read(MT6360_ADC_RPT1, &rpt0)) {
			break;
		}
		if ((rpt0 & 0x0f) == channel) {
			done = true;
			break;
		}
		delay_for_interval(10, NSEC_PER_MSEC);
	}
	if (done && mt6360_read(MT6360_ADC_RPT1 + 1, &hi) && mt6360_read(MT6360_ADC_RPT1 + 2, &lo)) {
		*raw = (int32_t)((hi << 8) | lo);
	}
	(void)mt6360_write(MT6360_ADC_CONFIG, 0x80);         /* no channel asked for; the ADC left on */
	(void)mt6360_write(MT6360_ADC_CONFIG + 1, 0);
	(void)mt6360_update(MT6360_ADC_RPT1, 0xf0, 0xf0);
	return done;
}

/* ---- the port's own negotiator, read-only: the TCPC half of the MT6360 -------------------- */

static bool
tcpc_read_word(uint8_t reg, uint16_t *value)
{
	uint8_t buf[2];

	if (!rm_dev_read_block(TCPC_ADDR, reg, buf, 2)) {
		return false;
	}
	*value = (uint16_t)(buf[0] | buf[1] << 8);
	return true;
}

static bool
tcpc_write_word(uint8_t reg, uint16_t value)
{
	uint8_t buf[2] = { (uint8_t)value, (uint8_t)(value >> 8) };

	return rm_dev_write_block(TCPC_ADDR, reg, buf, 2);
}

/* One power offer, said the way a person reads a brick's label: a fixed one by its voltage
 * and current, a ranged one (the APDO a programmable brick offers) by its window. The
 * encodings are the spec's own, fixed supply PDO bits 19:10 the voltage in fifties and
 * 9:0 the current in tens, the APDO its window in hundreds and its current in fifties. */
static void
pd_pdo_say(uint32_t pdo, char *out, int len)
{
	uint32_t kind = pdo >> 30;

	if (kind == 0) {
		snprintf(out, len, "%lu mV @ %lu mA", (unsigned long)(((pdo >> 10) & 0x3ff) * 50),
		    (unsigned long)((pdo & 0x3ff) * 10));
	} else if (kind == 3) {
		snprintf(out, len, "%lu-%lu mV @ %lu mA (ranged)",
		    (unsigned long)(((pdo >> 8) & 0x7f) * 100),
		    (unsigned long)(((pdo >> 17) & 0xff) * 100),
		    (unsigned long)((pdo & 0x7f) * 50));
	} else {
		snprintf(out, len, "kind %lu: %#lx", (unsigned long)kind, (unsigned long)pdo);
	}
}

/* A message from the brick, if one waits: read, said, and its alert handed back cleared --
 * the vendor's own get_message handshake, nothing else touched. The header's own two
 * objects come with the count in the FIFO's first bytes; the objects themselves follow. */
static void
tcpc_drain_rx(void)
{
	uint16_t alert = 0;

	if (!tcpc_read_word(TCPC_ALERT, &alert) || !(alert & TCPC_ALERT_RX_STATUS)) {
		return;
	}
	uint8_t head[4] = { 0, 0, 0, 0 }, payload[28];

	if (!rm_dev_read_block(TCPC_ADDR, TCPC_RX_BYTE_CNT, head, 4)) {
		return;
	}
	int n = head[0] > 3 ? head[0] - 3 : 0;

	if (n > 28) {
		n = 28;                     /* the frame's own count, clamped to the objects' room */
	}
	if (n > 0 && !rm_dev_read_block(TCPC_ADDR, TCPC_RX_DATA, payload, n)) {
		n = 0;
	}
	uint16_t hdr = (uint16_t)(head[2] | head[3] << 8);
	int objects = (hdr >> 12) & 7, type = hdr & 0x1f;

	if (objects > 7) {
		objects = 7;
	}
	if (objects * 4 > n) {
		objects = n / 4;            /* whatever the FIFO actually holds */
	}
	printf("pd: a message, type %d%s%s, %d objects, alert %#06x:", type,
	    type == 1 ? " (the source's offers)" : "", type == 13 ? " (a soft reset)" : "",
	    objects, alert);
	for (int i = 0; i < objects; i++) {
		char said[48];
		uint32_t pdo = payload[i * 4] | payload[i * 4 + 1] << 8 |
		    (uint32_t)payload[i * 4 + 2] << 16 | (uint32_t)payload[i * 4 + 3] << 24;

		pd_pdo_say(pdo, said, sizeof(said));
		printf(" [%s]", said);
	}
	printf("\n");
	if (tcpc_rx_seen == 0) {
		redwood_host_keep_add("pd: the brick's first message, type %d, %d objects", type, objects);
	}
	tcpc_rx_seen++;
	(void)tcpc_write_word(TCPC_ALERT, TCPC_ALERT_RX_STATUS | TCPC_ALERT_RX_OVERFLOW);
}

static void
rosemary_battery_thread(__unused void *parameter, __unused wait_result_t result)
{
	struct redwood_battery_state state;
	uint64_t reads = 0, failed = 0;
	bool mt6360 = false;
	bool anchored = false;                    /* the coulomb anchor is taken */
	int anchor_soc = 0;                       /* the percent the anchor holds, in hundredths */
	int64_t anchor_car = 0, car_prev = 0;     /* the counter at the anchor and at the last tick */
	unsigned frozen = 0;                      /* ticks the counter has not moved at a real current */

	memset(&state, 0, sizeof(state));
	state.size = (int32_t)sizeof(state);
	state.present = 1;
	state.temperature_hundredths = 2500;       /* until the first thermistor reading */

	/* the touch's init has the PMIC's bus for its supply; give it a moment to let go */
	delay_for_interval(10, NSEC_PER_SEC);
	rm_gpio = ml_io_map(RM_GPIO_PHYS, 0x1000);
	rm_wr(RM_MODE_AT(RM_PWR_SCL), rm_rd(RM_MODE_AT(RM_PWR_SCL)) & ~(0xfu << RM_MODE_BIT(RM_PWR_SCL)));
	rm_wr(RM_MODE_AT(RM_PWR_SDA), rm_rd(RM_MODE_AT(RM_PWR_SDA)) & ~(0xfu << RM_MODE_BIT(RM_PWR_SDA)));

	/* the AUXADC's reset, the way Linux's probe applies it (auxadc_reset of mt635x-auxadc.c) */
	uint32_t w;

	(void)redwood_pmic_read(MT6359P_HK_TOP_RST_CON0, &w);
	(void)redwood_pmic_write(MT6359P_HK_TOP_RST_CON0, w | 0x9);
	(void)redwood_pmic_write(MT6359P_HK_TOP_RST_CON0, w & ~0x9u);
	(void)redwood_pmic_write(MT6359P_HK_TOP_WKEY, 0);
	(void)redwood_pmic_read(MT6359P_AUXADC_RQST0, &w);
	(void)redwood_pmic_write(MT6359P_AUXADC_RQST0, w | 0x80);
	(void)redwood_pmic_read(MT6359P_AUXADC_RQST1, &w);
	(void)redwood_pmic_write(MT6359P_AUXADC_RQST1, w | 0x40);

	/* the fuel gauge's switch, on since Android's last boot and set here if it ever is not */
	if (redwood_pmic_read(MT6359P_FGADC_CON0, &w) && !(w & 1)) {
		(void)redwood_pmic_write(MT6359P_FGADC_CON0, w | 1);
		printf("battery: the fuel gauge was off; its switch is set\n");
	}

	/* the companion answers open drain first, then with ones driven: which way is a finding */
	for (int pushpull = 0; pushpull < 2; pushpull++) {
		uint8_t stat = 0;

		rm_pushpull = pushpull != 0;
		rm_line_low(RM_PWR_SCL);
		rm_line_release(RM_PWR_SCL);
		rm_line_low(RM_PWR_SDA);
		rm_line_release(RM_PWR_SDA);
		if (mt6360_read(MT6360_CHG_STAT, &stat)) {
			printf("battery: the MT6360 answers at %#x on i2c5, %s (chgstat %#x)\n", MT6360_PMU_ADDR,
			    rm_pushpull ? "ones driven" : "open drain", stat);
			mt6360 = true;
			break;
		}
	}
	if (!mt6360) {
		printf("battery: the MT6360 does not answer at %#x; the charger's state will be the PMIC's own bit\n",
		    MT6360_PMU_ADDR);
	}

	/* what the bootloader left in the charger, said once and kept for the host: the switch, the
	 * input's limit and where it comes from, the battery's current and ceiling, the port's
	 * droop floor, and the watchdog nobody here pets */
	if (mt6360) {
		uint8_t ctrl2 = 0, ctrl3 = 0, ctrl4 = 0, ctrl6 = 0, ctrl7 = 0, ctrl13 = 0;

		(void)mt6360_read(MT6360_CHG_CTRL2, &ctrl2);
		(void)mt6360_read(MT6360_CHG_CTRL3, &ctrl3);
		(void)mt6360_read(MT6360_CHG_CTRL4, &ctrl4);
		(void)mt6360_read(MT6360_CHG_CTRL6, &ctrl6);
		(void)mt6360_read(MT6360_CHG_CTRL7, &ctrl7);
		(void)mt6360_read(MT6360_CHG_CTRL13, &ctrl13);
		printf("battery: the MT6360 as found: charge %s, the limit's source %u, the input %u mA, "
		    "the battery %u mA, the ceiling %u mV, the droop floor %u mV, the watchdog %s\n",
		    ctrl2 & 1 ? "on" : "off", (ctrl2 >> 2) & 3, 100 + ((ctrl3 >> 2) & 0x3f) * 50,
		    100 + ((ctrl7 >> 2) & 0x3f) * 100, 3900 + ((ctrl4 >> 1) & 0x7f) * 10,
		    3900 + ((ctrl6 >> 1) & 0x7f) * 100, ctrl13 & 0x80 ? "on" : "off");
		redwood_host_keep_add("battery: the MT6360 as found: charge %s, the limit's source %u, "
		    "the input %u mA, the battery %u mA, the ceiling %u mV, the droop floor %u mV, the watchdog %s",
		    ctrl2 & 1 ? "on" : "off", (ctrl2 >> 2) & 3, 100 + ((ctrl3 >> 2) & 0x3f) * 50,
		    100 + ((ctrl7 >> 2) & 0x3f) * 100, 3900 + ((ctrl4 >> 1) & 0x7f) * 10,
		    3900 + ((ctrl6 >> 1) & 0x7f) * 100, ctrl13 & 0x80 ? "on" : "off");

		/* the currents, if asked for: the input's limit turned to the AICR register first
		 * (the source the bootloader left may not honor it), the input brought up from
		 * 500 mA the way the vendor's probe settles it, then the battery's own current */
		if (chg_aicr_ma > 0 || chg_ichg_ma > 0 || chg_cv_mv > 0) {
			int aicr = chg_aicr_ma;

			if (aicr <= 0) {
				aicr = 100 + ((ctrl3 >> 2) & 0x3f) * 50;    /* keep the limit, take the register's side */
			}
			if (aicr < 100) aicr = 100;
			if (aicr > 3250) aicr = 3250;
			(void)mt6360_update(MT6360_CHG_CTRL2, 0x0d, (2 << 2) | 1);
			if (aicr > 500) {
				(void)mt6360_update(MT6360_CHG_CTRL3, 0xfc, 8 << 2);    /* 500 mA while the port settles */
				delay_for_interval(100, NSEC_PER_MSEC);
			}
			(void)mt6360_update(MT6360_CHG_CTRL3, 0xfc, (uint8_t)((aicr - 100) / 50) << 2);
			if (chg_ichg_ma > 0) {
				int ichg = chg_ichg_ma < 100 ? 100 : chg_ichg_ma > 5000 ? 5000 : chg_ichg_ma;

				(void)mt6360_update(MT6360_CHG_CTRL7, 0xfc, (uint8_t)((ichg - 100) / 100) << 2);
			}
			if (chg_cv_mv > 0) {
				int cv = chg_cv_mv < 3900 ? 3900 : chg_cv_mv > 4710 ? 4710 : chg_cv_mv;

				(void)mt6360_update(MT6360_CHG_CTRL4, 0xfe, (uint8_t)((cv - 3900) / 10) << 1);
			}
			(void)mt6360_read(MT6360_CHG_CTRL2, &ctrl2);
			(void)mt6360_read(MT6360_CHG_CTRL3, &ctrl3);
			(void)mt6360_read(MT6360_CHG_CTRL4, &ctrl4);
			(void)mt6360_read(MT6360_CHG_CTRL7, &ctrl7);
			printf("battery: the MT6360 set: charge %s, the limit's source %u, the input %u mA, "
			    "the battery %u mA, the ceiling %u mV\n", ctrl2 & 1 ? "on" : "off", (ctrl2 >> 2) & 3,
			    100 + ((ctrl3 >> 2) & 0x3f) * 50, 100 + ((ctrl7 >> 2) & 0x3f) * 100,
			    3900 + ((ctrl4 >> 1) & 0x7f) * 10);
			redwood_host_keep_add("battery: the MT6360 set: charge %s, the limit's source %u, "
			    "the input %u mA, the battery %u mA, the ceiling %u mV", ctrl2 & 1 ? "on" : "off",
			    (ctrl2 >> 2) & 3, 100 + ((ctrl3 >> 2) & 0x3f) * 50, 100 + ((ctrl7 >> 2) & 0x3f) * 100,
			    3900 + ((ctrl4 >> 1) & 0x7f) * 10);
		}
	}

	/* the port's own negotiator, read-only eyes when pd_probe asks: who it is, and the
	 * state the bootloader left it in -- the attachment, the receiver's switch, the PD
	 * bias. Everything but the alert handshake is left exactly as found. pd_listen goes
	 * one step on: the receiver's switch turned to the vendor's listening value, so the
	 * brick's own words land in the log; no word is ever sent back. */
	if (pd_probe || pd_listen) {
		uint16_t vid = 0, pid = 0, did = 0;

		tcpc_ok = tcpc_read_word(TCPC_VID, &vid) && tcpc_read_word(TCPC_PID, &pid) &&
		    tcpc_read_word(TCPC_DID, &did);
		if (!tcpc_ok) {
			printf("battery: no answer at %#x on i2c5; the negotiator is elsewhere\n", TCPC_ADDR);
			redwood_host_keep_add("battery: no answer at %#x; the negotiator is elsewhere", TCPC_ADDR);
		} else {
			uint16_t alert = 0, amask = 0;
			uint8_t tctrl = 0, role = 0, pctrl = 0, cc = 0, pstat = 0, hdr = 0, rxdet = 0;
			uint8_t mode3 = 0, clk1 = 0, clk2 = 0;

			(void)tcpc_read_word(TCPC_ALERT, &alert);
			(void)tcpc_read_word(TCPC_ALERT_MASK, &amask);
			(void)rm_dev_read(TCPC_ADDR, TCPC_CTRL, &tctrl);
			(void)rm_dev_read(TCPC_ADDR, TCPC_ROLE_CTRL, &role);
			(void)rm_dev_read(TCPC_ADDR, TCPC_POWER_CTRL, &pctrl);
			(void)rm_dev_read(TCPC_ADDR, TCPC_CC_STATUS, &cc);
			(void)rm_dev_read(TCPC_ADDR, TCPC_POWER_STATUS, &pstat);
			(void)rm_dev_read(TCPC_ADDR, TCPC_MSG_HDR_INFO, &hdr);
			(void)rm_dev_read(TCPC_ADDR, TCPC_RX_DETECT, &rxdet);
			(void)rm_dev_read(TCPC_ADDR, TCPC_MTK_CLK_CTRL1, &clk1);
			(void)rm_dev_read(TCPC_ADDR, TCPC_MTK_CLK_CTRL2, &clk2);
			(void)rm_dev_read(TCPC_ADDR, TCPC_MTK_MODE_CTRL3, &mode3);
			printf("battery: the TCPC at %#x: id %04x:%04x did %04x, cc %#x (looking %u, "
			    "result %u, cc1 %u cc2 %u), vbus %u, rx-detect %#x, hdr-info %#x, mode3 %#x, "
			    "clks %#x/%#x, alert %04x (mask %04x)\n", TCPC_ADDR, vid, pid, did, cc,
			    cc >> 7 & 1, cc >> 5 & 3, cc & 3, cc >> 2 & 3,
			    (pstat >> 2) & 1, rxdet, hdr, mode3, clk1, clk2, alert, amask);
			redwood_host_keep_add("battery: the TCPC at %#x: id %04x:%04x did %04x, cc %#x, "
			    "vbus %u, rx-detect %#x, mode3 %#x", TCPC_ADDR, vid, pid, did, cc,
			    (pstat >> 2) & 1, rxdet, mode3);
			if (alert & TCPC_ALERT_RX_STATUS) {
				tcpc_drain_rx();    /* whatever waited through the boot, said now */
			}
			if (pd_listen) {
				/* the vendor's set_rx_enable: the clocks un-gated, then the ear on */
				uint8_t rxdet = 0;

				(void)rm_dev_write(TCPC_ADDR, TCPC_MTK_CLK_CTRL1, 0xca);
				(void)rm_dev_write(TCPC_ADDR, TCPC_MTK_CLK_CTRL2, 0x54);
				(void)rm_dev_write(TCPC_ADDR, TCPC_RX_DETECT, TCPC_RX_DETECT_ON);
				(void)rm_dev_read(TCPC_ADDR, TCPC_RX_DETECT, &rxdet);
				printf("battery: the TCPC's ear on (rx-detect now %#x); listening, never speaking\n", rxdet);
				redwood_host_keep_add("battery: the TCPC's ear on (rx-detect %#x); listening, never speaking", rxdet);
			}
		}
	}

	for (unsigned tick = 0;; tick++) {
		uint32_t batadc = 0, vcdt = 0, ntc = 0, chrdet = 0;
		uint8_t stat = 0, stat1 = 0;
		int chgstat = -1, pwr_rdy = 0, charger, status, soc, ma, ocv;
		uint32_t vbat, vbus;
		int64_t car = 0, ua = 0;
		bool car_ok, by_coulombs = false;

		if (!rosemary_auxadc(0, 0x7fff, &batadc) || !rosemary_auxadc(2, 0x0fff, &vcdt) ||
		    !rosemary_auxadc(3, 0x0fff, &ntc) || !redwood_pmic_read(MT6359P_CHR_CON0, &chrdet)) {
			failed++;
			delay_for_interval(5000, NSEC_PER_MSEC);
			continue;
		}
		reads++;

		vbat = batadc * 6300u >> 15;              /* <7 2> over 15 bits: raw * 7 * 1800 / 2 */
		vbus = vcdt * 1800u >> 12;                /* <1 1> over 12: the raw word says the rest */
		battery_vbus_mv = (int)vbus;
		battery_vcdt_raw = (int)vcdt;
		battery_ntc_mv = (int)(ntc * 4500u >> 12);    /* <5 2> over 12 bits */

		/* the charger's own ADC: the port's voltage and current, the battery's at its pin,
		 * and the two temperatures -- the readings the fast charge will be watched with */
		int32_t adc_vbus = 0, adc_ibus = 0, adc_ibat = 0, adc_vbat = 0, adc_jc = 0;
		bool adc_ok = mt6360 && mt6360_adc(MT6360_ADC_VBUSDIV5, &adc_vbus) &&
		    mt6360_adc(MT6360_ADC_IBUS, &adc_ibus) && mt6360_adc(MT6360_ADC_IBAT, &adc_ibat) &&
		    mt6360_adc(MT6360_ADC_VBAT, &adc_vbat) && mt6360_adc(MT6360_ADC_TEMP_JC, &adc_jc);
		int vbus2_mv = adc_ok ? (int)(adc_vbus * 6250u / 1000) : 0;
		int vbat2_mv = adc_ok ? (int)(adc_vbat * 1250u / 1000) : 0;
		int ibat2_ma = adc_ok ? (int)(((adc_ibat ^ 0x8000u) - 0x8000) * 5 / 2) : 0;  /* signed counts, 2.5 mA each */
		int jc_c = adc_ok ? (int)(adc_jc * 105 - 8000) / 100 : 0;
		int ibus2_ma = 0;

		if (adc_ok) {
			uint8_t ctrl3 = 0;

			(void)mt6360_read(MT6360_CHG_CTRL3, &ctrl3);
			ibus2_ma = (int)adc_ibus * (((ctrl3 >> 2) & 0x3f) < 6 ? 19 : 25) / 10;
		}

		/* the brick's own words, if it sends any while we watch (read and handed back;
		 * with pd_listen the ear is on, so its offers land here) */
		if ((pd_probe || pd_listen) && tcpc_ok) {
			tcpc_drain_rx();
		}

		car_ok = rosemary_fg_car(&car);
		(void)rosemary_fg_iavg(&ua);
		ma = (int)(ua / 1000);                    /* the averaged current, charging positive */
		ocv = (int)vbat - ma * ROSEMARY_R_BAT_MOHM / 1000;  /* its IR off the terminal voltage */

		/* the anchor at the start, and again whenever the counter has stopped counting: the
		 * voltage's own estimate, taken at the gauge's side of the resistor */
		if (car_ok) {
			if (!anchored) {
				anchor_soc = rosemary_soc(ocv > 0 ? (uint32_t)ocv : vbat);
				anchor_car = car;
				anchored = true;
				frozen = 0;
			} else if (car != car_prev) {
				frozen = 0;
			} else if (ma > 100 || ma < -100) {
				if (++frozen >= 3) {
					anchor_soc = rosemary_soc(ocv > 0 ? (uint32_t)ocv : vbat);
					anchor_car = car;
					frozen = 0;
				}
			}
			car_prev = car;
		}
		if (car_ok && anchored) {
			/* the coulombs since the anchor: each count 0.085 uAh into the battery's own 4952 mAh */
			int64_t duah = (car - anchor_car) * 85 / 1000;
			int dsoc = (int)(duah * 10000 / ROSEMARY_QMAX_UAH);

			soc = anchor_soc + dsoc;
			if (soc < 0) soc = 0;
			if (soc > 10000) soc = 10000;
			by_coulombs = true;
		} else {
			soc = rosemary_soc(vbat);
		}

		if (mt6360) {
			if (mt6360_read(MT6360_CHG_STAT, &stat)) {
				chgstat = (stat >> 6) & 3;
			}
			if (mt6360_read(MT6360_CHG_STAT1, &stat1)) {
				pwr_rdy = stat1 >> 7 & 1;
			}
		}
		charger = (chrdet & 1) != 0 || pwr_rdy != 0;

		/* Linux's POWER_SUPPLY_STATUS_*: 1 charging, 2 discharging, 3 not charging, 4 full */
		if (!charger) {
			status = 2;
		} else if (chgstat == 1) {
			status = 1;
		} else if (chgstat == 2) {
			status = 4;
		} else if (chgstat == 3) {
			status = 3;                   /* a fault: not charging, and said so below */
		} else {
			status = chgstat == 0 ? 3 : (soc < 10000 ? 1 : 4);
		}

		state.percent_hundredths = soc;
		state.voltage_uv = (int32_t)(vbat * 1000);
		state.current_ua = (int32_t)ua;
		state.temperature_hundredths = (int32_t)rosemary_ntc_c(battery_ntc_mv) * 100;
		state.present = vbat >= 2500 && vbat <= 5500;
		state.external = charger;
		state.status = status;
		state.usb_type = charger ? 2 : 0;         /* a charger, of no kind told yet */
		state.usb_voltage_uv = (int32_t)(vbus * 1000);
		redwood_battery_report(&state);
		battery_source_dsp = 2;                  /* not the DSP: this driver's own reading */

		if (tick % 12 == 0) {
			printf("battery: SOC=%d.%02d%% by %s, V=%u mV, I=%d mA, CAR=%lld, T=%d C, jc=%d C, "
			    "port=%d mV (ibus %d mA), MT6360: VBAT %d mV, IBAT %d mA, %s, charger=%d "
			    "(chrdet %u, pwr_rdy %d, chgstat %d), %llu reads, %llu failed\n",
			    soc / 100, soc % 100, by_coulombs ? "coulombs" : "voltage", vbat, ma,
			    (long long)car, rosemary_ntc_c(battery_ntc_mv), jc_c, vbus2_mv, ibus2_ma,
			    vbat2_mv, ibat2_ma, adc_ok ? "adc ok" : "no adc", charger,
			    chrdet & 1, pwr_rdy, chgstat, reads, failed);
		}
		delay_for_interval(5000, NSEC_PER_MSEC);
	}
}

void
redwood_battery_init(void)
{
	uint32_t enabled = 0;
	uint64_t shared_pa = 0;
	vm_offset_t page;
	thread_t thread;

	if (!PE_parse_boot_argn("battery", &enabled, sizeof(enabled)) || !enabled) return;
	if (enabled == 3) {
		/* the Redmi Note 10S's own: the PMIC's AUXADC and the charger companion, above */
		(void)PE_parse_boot_argn("chg_aicr", &chg_aicr_ma, sizeof(chg_aicr_ma));
		(void)PE_parse_boot_argn("chg_ichg", &chg_ichg_ma, sizeof(chg_ichg_ma));
		(void)PE_parse_boot_argn("chg_cv", &chg_cv_mv, sizeof(chg_cv_mv));
		(void)PE_parse_boot_argn("pd_probe", &pd_probe, sizeof(pd_probe));
	(void)PE_parse_boot_argn("pd_listen", &pd_listen, sizeof(pd_listen));
		if (kernel_thread_start(rosemary_battery_thread, NULL, &thread) == KERN_SUCCESS) {
			thread_deallocate(thread);
			printf("battery: this phone's own, read every 5 seconds\n");
		} else {
			printf("battery: cannot start the thread that reads this phone's battery\n");
		}
		return;
	}
	if (!PE_parse_boot_argn("battery_shared_pa", &shared_pa, sizeof(shared_pa)) || shared_pa == 0) {
		printf("battery: xnuloader did not provide battery_shared_pa\n");
		return;
	}
	page = ml_io_map((vm_offset_t)(shared_pa & ~(LOADER_PAGE_SIZE - 1)), LOADER_PAGE_SIZE);
	shared_battery = (volatile struct loader_battery *)(page + (shared_pa & (LOADER_PAGE_SIZE - 1)));
	if (shared_battery->magic != LOADER_BATTERY_MAGIC || shared_battery->version != LOADER_BATTERY_VERSION ||
	    shared_battery->size < sizeof(struct loader_battery)) {
		printf("battery: bad xnuloader cache at %#llx (magic %#llx version %u size %u)\n", shared_pa,
		    shared_battery->magic, shared_battery->version, shared_battery->size);
		return;
	}
	printf("battery: Qualcomm UEFI cache at pa %#llx; polling every 5 seconds\n", shared_pa);
	if (kernel_thread_start(battery_thread, NULL, &thread) == KERN_SUCCESS) thread_deallocate(thread);
}
