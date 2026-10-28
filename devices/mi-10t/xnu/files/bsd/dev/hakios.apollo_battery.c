/*
 * The battery of the Xiaomi Mi 10T (apollo), with boot-arg apollo_battery=1 (bsd/dev/hakios.apollo_input.c).
 *
 * By default (apollo_battery_source=spmi) the battery is read from the PMIC itself, every five seconds, through the
 * SPMI arbiter's observer channels -- reads only. The PM8150B (SPMI slave 2) has the fuel gauge of the fourth kind
 * (Linux's qpnp-fg-gen4, not the QG of the smaller PMICs: arch/arm64/boot/dts/venor/qcom/pm8150b.dtsi has
 * "qcom,fg-gen4" with BATT_SOC at 0x4000, BATT_INFO at 0x4100, RRADC at 0x4200, MEM_IF at 0x4300), which runs its
 * own algorithm (coulomb counting blended with the OCV of the battery profile the UEFI loaded into it) inside the PMIC;
 * its monotonic SOC, the battery's voltage and current are plain registers (gauge_read()). The charger's own state
 * (charging, terminated, a cable in) comes from the PM8150B's charger (qpnp-smb5), the temperature from the gauge's
 * thermistor ADC (temperature_choose()). Nothing of the firmware is asked, and the loader need not poll.
 *
 * With apollo_battery_source=loader, the older way: the phone's own UEFI talks to the gauge, with Xiaomi's Qualcomm
 * charger drivers (ChargerExDxe and QcomChargerDxeLA, in uefi/files/Binaries/apollo). The loader, which stays on the
 * first core, asks them every few seconds and keeps the answer on a page of its own (xnuloader/loader.c,
 * charger_poll(): read only, no charging action taken) whose address it gives as battery_shared_pa, and that page is
 * read here every five seconds. That has the UEFI's charger driver and this kernel's SPMI users drive the one arbiter
 * from two systems at once, which is suspected of wedging it (and the loader's core with it).
 *
 * Either way the result goes to the power source iOS reads (iokit/Kernel/hakios.battery.cpp, HakiosBatteryReport(),
 * shared), and to the sysctls kern.apollo.battery_*.
 *
 * Boot-arg apollo_charger=N (the PM8150B's charger itself, SPMI slave 2: peripherals 0x10 to 0x16, through the arbiter
 * as hakios.apollo_backlight.c reaches it; a write only goes through a channel that system 0 owns): 1 logs its registers
 * (charge state, limits, power path, APSD, USBIN) once a minute and when the loader's charger_present changes; 2 also,
 * every five seconds while a cable is in, pets its watchdog (whose bite disables charging) and clears a USBIN suspend,
 * a disabled charge enable and a charge pause; 3 also sets the input and charge currents (chg_currents()). None of
 * that is written unless the loader's page is fresh (its `polls` advanced within CHG_FRESH_SECONDS) and the battery's
 * temperature it gives is a real one: a loader that has stopped polling, or a gauge that gives no temperature, leaves
 * the charger to its own watchdog, whose bite stops the charging. From the PMIC itself (the default), "fresh" is that
 * a read of the gauge succeeded within CHG_FRESH_SECONDS, and the temperature is this tick's own, read and believed.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.apollo_input.h>
#include <sys/hakios.input.h>          /* struct hakios_battery_state: the power source's report, shared */
#include <sys/sysctl.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

/* into the loader's log as well (as the other apollo drivers): the level's changes are to be seen during a run */
extern void loader_log_putc(char c);

static int say(const char *format, ...) __printflike(1, 2);

static int
say(const char *format, ...)
{
	char line[300];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	return printf("%s", line);
}
extern void HakiosBatteryReport(const void *state, unsigned size);     /* iokit/Kernel/hakios.battery.cpp */

/* the loader's page (xnuloader/loader.c: struct loader_battery) */
#define LOADER_BATTERY_MAGIC            0x2154544142574452ULL           /* "RDWBATT!" */
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
	volatile uint32_t sequence;     /* odd while the loader writes */
	uint32_t flags;
	int32_t soc_percent, voltage_mv, current_ma, temperature_c;
	uint32_t charger_action, polls, failures, reserved;
};

static volatile struct loader_battery *shared;

static int apollo_battery_soc = -1, apollo_battery_voltage_mv = -1, apollo_battery_current_ma;
static int apollo_battery_temperature_c = -273, apollo_battery_present, apollo_battery_charger;
static int apollo_battery_charging, apollo_battery_estimated, apollo_battery_from_pmic;

SYSCTL_NODE(_kern, OID_AUTO, apollo, CTLFLAG_RW | CTLFLAG_LOCKED, 0, "the Mi 10T's own drivers");
SYSCTL_INT(_kern_apollo, OID_AUTO, battery_soc, CTLFLAG_RD | CTLFLAG_LOCKED, &apollo_battery_soc, 0,
    "battery charge, 0..100 percent, or -1 if unknown");
SYSCTL_INT(_kern_apollo, OID_AUTO, battery_voltage_mv, CTLFLAG_RD | CTLFLAG_LOCKED, &apollo_battery_voltage_mv, 0,
    "battery voltage in millivolts, or -1 if unknown");
SYSCTL_INT(_kern_apollo, OID_AUTO, battery_current_ma, CTLFLAG_RD | CTLFLAG_LOCKED, &apollo_battery_current_ma, 0,
    "battery current in milliamps: the gauge's (more than 0 while discharging), or as the UEFI's charger driver gives it");
SYSCTL_INT(_kern_apollo, OID_AUTO, battery_temperature_c, CTLFLAG_RD | CTLFLAG_LOCKED, &apollo_battery_temperature_c, 0,
    "battery temperature in degrees Celsius, or -273 if unknown");
SYSCTL_INT(_kern_apollo, OID_AUTO, battery_present, CTLFLAG_RD | CTLFLAG_LOCKED, &apollo_battery_present, 0,
    "1 if the charger driver sees a battery");
SYSCTL_INT(_kern_apollo, OID_AUTO, battery_charger_present, CTLFLAG_RD | CTLFLAG_LOCKED, &apollo_battery_charger, 0,
    "1 if something is plugged into the USB port");
SYSCTL_INT(_kern_apollo, OID_AUTO, battery_charging, CTLFLAG_RD | CTLFLAG_LOCKED, &apollo_battery_charging, 0,
    "1 while the battery is charging");
SYSCTL_INT(_kern_apollo, OID_AUTO, battery_soc_estimated, CTLFLAG_RD | CTLFLAG_LOCKED, &apollo_battery_estimated, 0,
    "1 when the charge is estimated from the voltage (the fuel gauge has not answered)");
SYSCTL_INT(_kern_apollo, OID_AUTO, battery_from_pmic, CTLFLAG_RD | CTLFLAG_LOCKED, &apollo_battery_from_pmic, 0,
    "1 when the battery is read from the PMIC over SPMI, 0 when from the loader's page");

/* a consistent copy of the page: the loader makes the sequence odd while it writes */
static bool
snapshot(struct loader_battery *out)
{
	for (int i = 0; i < 8; i++) {
		uint32_t before = shared->sequence, after;

		if (before & 1) {
			delay_for_interval(1, NSEC_PER_MSEC);
			continue;
		}
		__asm__ volatile ("dmb ish" ::: "memory");
		memcpy(out, (const void *)(uintptr_t)shared, sizeof(*out));
		__asm__ volatile ("dmb ish" ::: "memory");
		after = shared->sequence;
		if (before == after) {
			return true;
		}
	}
	return false;
}

static int32_t
clamp32(int64_t value)
{
	return value > 0x7fffffffLL ? 0x7fffffff : value < -0x80000000LL ? (int32_t)(-0x7fffffff - 1) : (int32_t)value;
}

/* ---- the PM8150B's charger (Linux's qcom-smb5; boot-arg apollo_charger=N) -------------------------------- */

#define CHG_FRESH_SECONDS       15ULL           /* three of the loader's polls, or of this driver's own reads */

/*
 * The charger is SPMI slave 2 (the PM8150B): peripherals 0x10 CHGR, 0x11 DCDC, 0x13 USBIN, 0x14 DCIN, 0x15 TYPEC and
 * 0x16 MISC, registers as drivers/power/supply/qcom_smbx.c / Xiaomi's smb5-lib have them. The arbiter and the way
 * of reading and writing it are those of hakios.apollo_backlight.c (version 5: reads through the observer at 0xe600000 +
 * 0x80 per channel, writes through the channel region at 0xc600000 + 0x10000 per channel), copied here.
 * apollo_charger=1 reads and logs the registers; =2 also does the minimal repairs of chg_fix(); =3 also sets the input
 * and charge currents from what the source says it can give (chg_currents()).
 */
#define ARB_CORE_PHYS           0x0c440000UL
#define ARB_CORE_SIZE           0x1100UL
#define ARB_CNFG_PHYS           0x0c40a000UL
#define ARB_CNFG_SIZE           0x1000UL
#define ARB_CHANNELS_PHYS       0x0c600000UL    /* the device tree's "chnls": 0x10000 per channel (v5) */
#define ARB_OBSERVER_PHYS       0x0e600000UL    /* reads only: 0x10000 per system, 0x80 per channel */
#define ARB_REGION_SIZE         0x10000UL
#define ARB_SYSTEM              0
#define ARB_FEATURES            0x004           /* [10:0] channels */
#define ARB_CHANNEL(n)          (0x900 + 4 * (n))       /* [19:8] slave << 8 | peripheral */
#define ARB_OWNER(n)            (0x700 + 4 * (n))       /* [2:0] the system that owns channel n */
#define ARB_MAX_CHANNELS        512

#define CHANNEL_COMMAND         0x00
#define CHANNEL_STATUS          0x08
#define CHANNEL_RDATA0          0x18
#define CHANNEL_WDATA0          0x10
#define STATUS_DONE             0x1
#define STATUS_FAILURE          0x2
#define STATUS_DENIED           0x4
#define STATUS_DROPPED          0x8
#define OP_EXT_WRITEL           0u
#define OP_EXT_READL            1u
#define TRANSFER_TIMEOUT_US     2000

static inline uint32_t
rd(vm_offset_t address)
{
	return *(volatile uint32_t *)address;
}

static inline void
wr(vm_offset_t address, uint32_t value)
{
	*(volatile uint32_t *)address = value;
}

static vm_offset_t arb_core, arb_cnfg, arb_observer;

/*
 * One SPMI transfer at a time through the arbiter, for every apollo driver (Linux's spmi-pmic-arb.c holds one lock,
 * pmic_arb->lock, around every transfer of the bus, whatever the channel). Declared here, extern in
 * sys/hakios.apollo_input.h: hakios.apollo_buttons.c and hakios.apollo_backlight.c each still have a lock of their own
 * and are to take this one instead. A spin lock: a transfer is polled for at most TRANSFER_TIMEOUT_US.
 */
static LCK_GRP_DECLARE(apollo_spmi_bus_grp, "apollo-spmi-bus");
LCK_SPIN_DECLARE(apollo_spmi_bus_lock, &apollo_spmi_bus_grp);

struct chg_peripheral {
	const char *name;
	uint16_t id;                    /* slave << 8 | peripheral */
	uint16_t channel;
	bool found;
	bool writable;                  /* the channel is owned by system 0 (this one) */
	bool gauge;                     /* the fuel gauge's: never written here, whoever owns it */
	bool refused_said;
	vm_offset_t writes;             /* its page of the channel region */
};
static struct chg_peripheral chg_periph[] = {
	{ .name = "CHGR", .id = 0x210 }, { .name = "DCDC", .id = 0x211 }, { .name = "USBIN", .id = 0x213 },
	{ .name = "DCIN", .id = 0x214 }, { .name = "TYPEC", .id = 0x215 }, { .name = "MISC", .id = 0x216 },
	/* the fuel gauge (qpnp-fg-gen4: pm8150b.dtsi's qcom,fg-batt-soc@4000, -batt-info@4100, -rradc@4200) */
	{ .name = "FG_BATT_SOC", .id = 0x240, .gauge = true }, { .name = "FG_BATT_INFO", .id = 0x241, .gauge = true },
	{ .name = "FG_RRADC", .id = 0x242, .gauge = true },
};
#define CHG_PERIPHERALS (sizeof(chg_periph) / sizeof(chg_periph[0]))

static uint32_t chg_level;              /* apollo_charger: 0 nothing, 1 read, 2 read and repair */
static bool chg_ready;
static uint32_t chg_writes, chg_write_failures;
static struct {
	uint64_t reads, failed;
} bus;

static const char *
chg_status_text(uint32_t status)
{
	return !(status & STATUS_DONE) ? "no answer from the arbiter" : (status & STATUS_DENIED) ? "the arbiter refuses" :
	       (status & STATUS_FAILURE) ? "the transfer failed" : "the transfer was dropped";
}

static struct chg_peripheral *
chg_for(uint16_t address)
{
	for (unsigned i = 0; i < CHG_PERIPHERALS; i++) {
		if ((chg_periph[i].id & 0xff) == (address >> 8)) {
			return &chg_periph[i];
		}
	}
	return NULL;
}

/*
 * `count` (1 to 4) bytes from `address` on (peripheral << 8 | offset, slave 2), through this system's observer channel
 * of the peripheral: a read only, whoever owns the channel. NULL, or what went wrong.
 */
static const char *
spmi_read(uint16_t address, uint8_t *bytes, uint32_t count)
{
	struct chg_peripheral *p = chg_for(address);
	vm_offset_t channel;
	uint64_t start, interval;
	uint32_t status, data;

	if (p == NULL || !p->found) {
		return "no channel";
	}
	if (count < 1 || count > 4 || (address & 0xff) + count > 0x100) {
		return "a read the arbiter cannot do";
	}
	channel = arb_observer + 0x10000UL * ARB_SYSTEM + 0x80UL * p->channel;
	nanoseconds_to_absolutetime(TRANSFER_TIMEOUT_US * (uint64_t)NSEC_PER_USEC, &interval);
	lck_spin_lock(&apollo_spmi_bus_lock);
	start = mach_absolute_time();
	wr(channel + CHANNEL_COMMAND, OP_EXT_READL << 27 | (uint32_t)(address & 0xff) << 4 | (count - 1));
	do {
		status = rd(channel + CHANNEL_STATUS);
	} while (!(status & STATUS_DONE) && mach_absolute_time() - start < interval);
	data = rd(channel + CHANNEL_RDATA0);
	lck_spin_unlock(&apollo_spmi_bus_lock);
	bus.reads++;
	if (!(status & STATUS_DONE) || (status & (STATUS_DENIED | STATUS_FAILURE | STATUS_DROPPED))) {
		bus.failed++;
		return chg_status_text(status);
	}
	for (uint32_t i = 0; i < count; i++) {
		bytes[i] = (uint8_t)(data >> (8 * i));
	}
	return NULL;
}

/* one byte at `address`; NULL, or what went wrong */
static const char *
chg_read(uint16_t address, uint8_t *value)
{
	return spmi_read(address, value, 1);
}

/* one byte into `address`: only through a channel that system 0 owns; NULL, or what went wrong */
static const char *
chg_write(uint16_t address, uint8_t value)
{
	struct chg_peripheral *p = chg_for(address);
	vm_offset_t channel;
	uint64_t start, interval;
	uint32_t status;

	if (p != NULL && p->gauge) {
		chg_write_failures++;
		return "the gauge is only read";
	}
	if (p == NULL || !p->found || !p->writable) {
		if (p != NULL && !p->refused_said) {
			p->refused_said = true;
			say("apollo charger: not writing %s (%04x): its channel is not owned by system 0\n", p->name, address);
		}
		chg_write_failures++;
		return "channel not ours";
	}
	channel = p->writes;
	nanoseconds_to_absolutetime(TRANSFER_TIMEOUT_US * (uint64_t)NSEC_PER_USEC, &interval);
	lck_spin_lock(&apollo_spmi_bus_lock);
	start = mach_absolute_time();
	wr(channel + CHANNEL_WDATA0, value);
	wr(channel + CHANNEL_COMMAND, OP_EXT_WRITEL << 27 | (uint32_t)(address & 0xff) << 4 | 0);
	do {
		status = rd(channel + CHANNEL_STATUS);
	} while (!(status & STATUS_DONE) && mach_absolute_time() - start < interval);
	lck_spin_unlock(&apollo_spmi_bus_lock);
	chg_writes++;
	if (!(status & STATUS_DONE) || (status & (STATUS_DENIED | STATUS_FAILURE | STATUS_DROPPED))) {
		chg_write_failures++;
		return chg_status_text(status);
	}
	return NULL;
}

/* the peripheral's channel: the first, unless a later one is this system's (as Linux chooses) */
static bool
chg_find_channel(struct chg_peripheral *p, uint32_t channels)
{
	uint32_t owner;

	for (uint32_t n = 0; n < channels; n++) {
		uint32_t map = rd(arb_core + ARB_CHANNEL(n));

		if (map == 0 || ((map >> 8) & 0xfff) != p->id) {
			continue;
		}
		if (!p->found || (rd(arb_cnfg + ARB_OWNER(n)) & 7) == ARB_SYSTEM) {
			p->found = true;
			p->channel = (uint16_t)n;
		}
	}
	if (!p->found) {
		say("apollo battery: no channel is %s (slave 2, peripheral %#x)\n", p->name, p->id & 0xff);
		return false;
	}
	owner = rd(arb_cnfg + ARB_OWNER(p->channel)) & 7;
	p->writable = owner == ARB_SYSTEM && !p->gauge;
	say("apollo battery: %s is channel %u (owned by system %u%s)\n", p->name, p->channel, owner,
	    p->gauge ? ": only read" : p->writable ? "" : ": read only");
	if (p->writable) {
		p->writes = ml_io_map(ARB_CHANNELS_PHYS + 0x10000UL * p->channel, 0x1000);
	}
	return true;
}

/* the registers logged, each as peripheral << 8 | offset */
static const uint16_t chg_regs[] = {
	0x1006, 0x1007, 0x100b, 0x100d, 0x1042, 0x1043, 0x1061, 0x1070,
	0x1106, 0x1107, 0x1108, 0x110a, 0x110b,
	0x1307, 0x1308, 0x1309, 0x1310, 0x1340, 0x1342, 0x1362, 0x1365, 0x1366, 0x1370, 0x1380,
	0x1410, 0x1440, 0x1506, 0x1509, 0x150b, 0x1606, 0x1651, 0x1653,
};
#define CHG_REG_COUNT (sizeof(chg_regs) / sizeof(chg_regs[0]))

static void
chg_log(void)
{
	static const char *const states[8] = { "inhibit", "trickle", "pre", "fullon", "taper", "term", "pause", "disabled" };
	static const char *const apsd[7] = { "SDP", "OCP", "CDP", "DCP", "FLOAT", "QC2", "QC3" };
	uint8_t v[CHG_REG_COUNT];
	bool ok[CHG_REG_COUNT];
	const char *problem = NULL, *first_problem = NULL;
	char line[200], names[48];
	int at;

	for (unsigned i = 0; i < CHG_REG_COUNT; i++) {
		ok[i] = (problem = chg_read(chg_regs[i], &v[i])) == NULL;
		if (!ok[i] && first_problem == NULL) {
			first_problem = problem;
		}
	}
	for (unsigned i = 0; i < CHG_REG_COUNT; i += 8) {
		at = snprintf(line, sizeof(line), "apollo charger:");
		for (unsigned j = i; j < i + 8 && j < CHG_REG_COUNT; j++) {
			if (ok[j]) {
				at += snprintf(line + at, sizeof(line) - (size_t)at, " %04x=%02x", chg_regs[j], v[j]);
			} else {
				at += snprintf(line + at, sizeof(line) - (size_t)at, " %04x=??", chg_regs[j]);
			}
		}
		say("%s\n", line);
	}
	if (first_problem != NULL) {
		say("apollo charger: a read failed: %s\n", first_problem);
	}
	/* indexes into chg_regs: 0 1006, 6 1061, 7 1070, 10 1108, 12 110b, 14 1308, 16 1310, 22 1370 */
	at = snprintf(line, sizeof(line), "apollo charger: state %s",
	    ok[0] ? states[v[0] & 7] : "??");
	if (ok[6]) {
		at += snprintf(line + at, sizeof(line) - (size_t)at, ", FCC %u mA", v[6] * 50u);
	}
	if (ok[7]) {
		at += snprintf(line + at, sizeof(line) - (size_t)at, ", FV %u mV", 3600u + v[7] * 10u);
	}
	if (ok[10]) {
		at += snprintf(line + at, sizeof(line) - (size_t)at, ", ICL %u mA", v[10] * 50u);
	}
	if (ok[22]) {
		at += snprintf(line + at, sizeof(line) - (size_t)at, ", USB ICL cfg %u mA", v[22] * 50u);
	}
	if (ok[12]) {
		at += snprintf(line + at, sizeof(line) - (size_t)at, ", path: usbin %s%s%s", (v[12] & 0x40) ? "SUSPENDED" : "active",
		    (v[12] & 0x10) ? " used" : "", (v[12] & 1) ? " valid" : " not valid");
	}
	if (ok[16]) {
		at += snprintf(line + at, sizeof(line) - (size_t)at, ", plugin %d", (v[16] >> 4) & 1);
	}
	if (ok[14]) {
		names[0] = 0;
		for (unsigned b = 0, n = 0; b < 7; b++) {
			if (v[14] & (1u << b)) {
				n += (unsigned)snprintf(names + n, sizeof(names) - n, "%s%s", n ? "+" : "", apsd[b]);
			}
		}
		at += snprintf(line + at, sizeof(line) - (size_t)at, ", APSD %s", names[0] ? names : "none");
	}
	snprintf(line + at, sizeof(line) - (size_t)at, ", writes %u (%u failed)", chg_writes, chg_write_failures);
	say("%s\n", line);
}

/*
 * apollo_charger=3: the currents. The phone's UEFI leaves the USB input at 500 mA (USBIN_CURRENT_LIMIT_CFG 0x1370 = 0x0a)
 * and the battery at 1.5 A (CHGR_FAST_CHARGE_CURRENT_CFG 0x1061 = 0x1e), whatever is plugged in, and nothing here runs
 * Xiaomi's charger driver, which would raise them. The input limit is set from what the source says (Xiaomi's
 * smb5-lib.c: smblib_get_prop_ufp_mode() reads TYPE_C_SNK_STATUS 0x1506 -- bit 1 Rp 3.0 A, bit 2 Rp 1.5 A, bit 3 default
 * USB; APSD_RESULT_STATUS 0x1308 bit 2 CDP, bit 3 DCP), at most the device tree's qcom,usb-icl-ua (3 A); the charger is in
 * the mode where that register is the limit after APSD already (smblib_icl_override(SW_OVERRIDE_HC_MODE): ICL_OPTIONS
 * 0x1366 bit 0 set, CMD_ICL_OVERRIDE 0x1342 bit 0 clear, USBIN_LOAD_CFG 0x1365 bit 4 set -- as found), and the
 * hardware's AICL (USBIN_AICL_OPTIONS 0x1380 bit 2, on) still backs off on a source that sags. The battery's current
 * goes to 2 A: Xiaomi's own limit for a battery it has not authenticated (mi,fcc-batt-unverify-ua), which is what it is
 * here. Both go back down when the battery is warm: at 45 degrees 1 A in and 1 A to the battery, at 50 the UEFI's 500 mA.
 * The float voltage stays as the UEFI set it (4.40 V; the tree allows 4.48).
 */
#define CHG_ICL_STEP_MA         50              /* USBIN_CURRENT_LIMIT_CFG and CHGR_FAST_CHARGE_CURRENT_CFG: 50 mA a step */
#define CHG_ICL_MAX_MA          3000            /* qcom,usb-icl-ua */
#define CHG_FCC_MA              2000            /* mi,fcc-batt-unverify-ua */

/*
 * What the charger may be told depends on the battery's temperature, JEITA-like (the cell's own limits are not known
 * here; these are the usual ones for a phone's Li-ion cell): under 0 degrees no charging should happen at all, so
 * chg_fix() does not re-enable it and chg_currents() puts both currents at the floor (500 mA); 0..9 degrees the
 * battery's current is held to 1 A (a cool cell takes less); 10..44 the full currents; from 45 the reduced ones below,
 * and above 45 chg_fix() no longer re-enables charging or pets the watchdog either -- the charger's own protection
 * decides. A temperature outside CHG_TEMP_SANE_* (the loader's -273 "unknown", or garbage) is no temperature: then
 * the currents go to the floor and chg_fix() does nothing.
 */
#define CHG_TEMP_SANE_MIN       (-40)
#define CHG_TEMP_SANE_MAX       100
#define CHG_TEMP_COLD           0               /* below: no charging */
#define CHG_TEMP_COOL           10              /* below: the battery's current at most 1 A */
#define CHG_TEMP_WARM           45              /* from: reduced currents; above: no repairs */
#define CHG_FLOOR_MA            500

static void
chg_currents(bool temperature_valid, int temperature_c)
{
	static uint32_t told_icl = ~0u, told_fcc = ~0u;
	uint8_t snk = 0, apsd = 0, raw = 0;
	uint32_t icl, fcc = CHG_FCC_MA;
	const char *why;

	if (chg_read(0x1506, &snk) != NULL || chg_read(0x1308, &apsd) != NULL) {
		return;
	}
	if (snk & 0x02) {
		icl = 3000, why = "USB-C Rp 3.0 A";
	} else if (snk & 0x04) {
		icl = 1500, why = "USB-C Rp 1.5 A";
	} else if (apsd & 0x08) {
		icl = 1500, why = "a dedicated charger (BC 1.2 DCP)";
	} else if (apsd & 0x04) {
		icl = 1500, why = "a charging downstream port (CDP)";
	} else {
		icl = 500, why = "a plain USB port";
	}
	if (!temperature_valid) {
		icl = CHG_FLOOR_MA, fcc = CHG_FLOOR_MA, why = "THE BATTERY'S TEMPERATURE IS NOT KNOWN";
	} else if (temperature_c < CHG_TEMP_COLD) {
		icl = CHG_FLOOR_MA, fcc = CHG_FLOOR_MA, why = "THE BATTERY IS BELOW 0 DEGREES";
	} else if (temperature_c >= 50) {
		icl = 500, fcc = 1000, why = "THE BATTERY IS AT 50 DEGREES OR MORE";
	} else if (temperature_c >= CHG_TEMP_WARM) {
		icl = icl > 1000 ? 1000 : icl, fcc = 1000, why = "the battery is at 45 degrees or more";
	} else if (temperature_c < CHG_TEMP_COOL) {
		fcc = 1000, why = "the battery is below 10 degrees";
	}
	icl = icl > CHG_ICL_MAX_MA ? CHG_ICL_MAX_MA : icl;
	if (chg_read(0x1370, &raw) == NULL && raw != icl / CHG_ICL_STEP_MA) {
		if (chg_write(0x1370, (uint8_t)(icl / CHG_ICL_STEP_MA)) == NULL && told_icl != icl) {
			say("apollo charger: the input limit %u -> %u mA (%s)\n", raw * CHG_ICL_STEP_MA, icl, why);
			told_icl = icl;
		}
	}
	if (chg_read(0x1061, &raw) == NULL && raw != fcc / CHG_ICL_STEP_MA) {
		if (chg_write(0x1061, (uint8_t)(fcc / CHG_ICL_STEP_MA)) == NULL && told_fcc != fcc) {
			say("apollo charger: the battery's current %u -> %u mA (battery at %d degrees%s)\n", raw * CHG_ICL_STEP_MA, fcc,
			    temperature_c, temperature_valid ? "" : ", not a real temperature");
			told_fcc = fcc;
		}
	}
}

/* apollo_charger=2: the minimal repairs, while a cable is in */
static void
chg_fix(void)
{
	static bool suspend_said, enable_said, pause_said;
	uint8_t v;
	const char *problem;

	if (chg_read(0x1310, &v) != NULL || !(v & 0x10)) {
		return;
	}
	/* BARK_BITE_WDOG_PET: nothing here services the bark, and the bite disables charging */
	if ((problem = chg_write(0x1643, 0x01)) != NULL && strcmp(problem, "channel not ours") != 0) {
		say("apollo charger: the watchdog pet failed: %s\n", problem);
	}
	if (chg_read(0x1340, &v) == NULL) {
		if (v & 1) {
			if (!suspend_said) {
				say("apollo charger: USBIN is suspended (%02x): clearing it\n", v);
				suspend_said = true;
			}
			chg_write(0x1340, (uint8_t)(v & ~1));
		} else {
			suspend_said = false;
		}
	}
	if (chg_read(0x1042, &v) == NULL) {
		if (!(v & 1)) {
			if (!enable_said) {
				say("apollo charger: charging is not enabled (%02x): enabling it\n", v);
				enable_said = true;
			}
			chg_write(0x1042, (uint8_t)(v | 1));
		} else {
			enable_said = false;
		}
	}
	if (chg_read(0x1043, &v) == NULL) {
		if (v & 1) {
			if (!pause_said) {
				say("apollo charger: charging is paused (%02x): resuming it\n", v);
				pause_said = true;
			}
			chg_write(0x1043, (uint8_t)(v & ~1));
		} else {
			pause_said = false;
		}
	}
}
/* the arbiter mapped and every peripheral's channel found: for the gauge's reads and for apollo_charger=N */
static bool
spmi_init(void)
{
	uint32_t channels;

	arb_core = ml_io_map(ARB_CORE_PHYS, ARB_CORE_SIZE);
	arb_cnfg = ml_io_map(ARB_CNFG_PHYS, ARB_CNFG_SIZE);
	arb_observer = ml_io_map(ARB_OBSERVER_PHYS, ARB_REGION_SIZE);
	if (arb_core == 0 || arb_cnfg == 0 || arb_observer == 0) {
		say("apollo battery: cannot map the SPMI arbiter\n");
		return false;
	}
	channels = rd(arb_core + ARB_FEATURES) & 0x7ff;
	if (channels > ARB_MAX_CHANNELS) {
		channels = ARB_MAX_CHANNELS;
	}
	for (unsigned i = 0; i < CHG_PERIPHERALS; i++) {
		chg_find_channel(&chg_periph[i], channels);
	}
	return true;
}

static void
chg_init(void)
{
	chg_ready = true;
	say("apollo charger: level %u (%s)\n", chg_level, chg_level >= 3 ? "reads, repairs, and the currents from the source" : chg_level >= 2 ? "reads, and the minimal repairs" : "reads only");
}

/* apollo_charger=N, once a tick of either source: `fresh` and the temperature as that source has them */
static void
chg_tick(unsigned tick, bool fresh, bool temperature_valid, int temperature_c, int charger)
{
	static int last_charger = -1;

	if (!chg_ready) {
		return;
	}
	if (tick % 12 == 0 || charger != last_charger) {
		chg_log();
	}
	last_charger = charger;
	/* stale: nothing written at all, and the watchdog left to bite */
	if (chg_level >= 2 && fresh && temperature_valid && temperature_c >= CHG_TEMP_COLD &&
	    temperature_c <= CHG_TEMP_WARM) {
		chg_fix();
	}
	if (chg_level >= 3 && fresh && charger) {
		chg_currents(temperature_valid, temperature_c);
	}
}

/* ---- the PM8150B's fuel gauge, read over SPMI (apollo_battery_source=spmi, the default) ---------------------- */

/*
 * Registers of Xiaomi's kernel (kernel-apollo: drivers/power/supply/qcom/fg-reg.h, read as fg-util.c reads them),
 * each as peripheral << 8 | offset of slave 2:
 *   BATT_SOC_FG_MONOTONIC_SOC 0x4009 and its copy 0x400a: the gauge's own state of charge, 0..255 (fg_get_msoc_raw():
 *     believed when the two agree), as Android shows it after fg_get_msoc()'s scaling, msoc_percent() here;
 *   BATT_SOC_EN_CTL 0x4046 bit 7: the gauge's algorithm runs; BATT_SOC_INT_RT_STS 0x4010 bit 1: "soc-ready";
 *   BATT_INFO_VBATT 0x41a0..a1, IBATT 0x41a2..a3 and their copies 0x41a6..a9 (fg_get_battery_voltage() and
 *     _current(): 122.07 uV a step; 488.281 uA a step, signed, more than 0 while the battery discharges);
 *   ADC_RR_INT_RT_STS 0x4210 bit 0: the battery is missing (the gauge's own test);
 *   ADC_RR_BATT_TEMP 0x4288..89: the thermistor's reading, 10 bits; BATT_INFO_BATT_TEMP 0x4150..51, 11 bits, and
 *     BATT_INFO_BATT_TEMP_STS 0x4106 (bit 0 "available"). Xiaomi's gen4 driver reads neither for the temperature
 *     (it takes the gauge's memory, BATT_TEMP_WORD, through MEM_IF -- writes, which are not done here): the formats are
 *     whole degrees for the first (signed, 10 bits: seen on the phone, 0x21 = 33 C where Android said 35.5 C), and gen3's
 *     (quarter kelvin, 11 bits: fg_get_battery_temp() of qpnp-fg-gen3) for the second, which reads 0 here; see
 *     temperature_choose() for how they are trusted.
 * And of the charger (smb5-reg.h): BATTERY_CHARGER_STATUS_1 0x1006 [2:0] its state, BATTERY_CHARGER_STATUS_7 0x100d
 * its own comparators on the battery's thermistor (JEITA: bit 5 hot, 4 cold, 3 too hot, 2 too cold, 1/0 the same,
 * "AFP"), USBIN_INT_RT_STS 0x1310 bit 4 a cable in.
 */
#define FG_SOC_MONOTONIC        0x4009
#define FG_SOC_INT_RT_STS       0x4010
#define FG_SOC_READY            0x02
#define FG_SOC_EN_CTL           0x4046
#define FG_ALGORITHM_EN         0x80
#define FG_INFO_TEMP_STS        0x4106
#define FG_INFO_BATT_TEMP       0x4150
#define FG_INFO_VBATT_IBATT     0x41a0
#define FG_INFO_VBATT_IBATT_CP  0x41a6
#define FG_RR_INT_RT_STS        0x4210
#define FG_RR_BATT_MISSING      0x01
#define FG_RR_BATT_TEMP         0x4288
#define FG_READ_TRIES           5               /* fg-util.c's MAX_READ_TRIES */
#define CHGR_STATUS_1           0x1006
#define CHGR_STATUS_7           0x100d
#define CHGR_JEITA_HOT          0x2a            /* hot (soft), too hot, too hot (AFP) */
#define CHGR_JEITA_TOO_COLD     0x05            /* too cold, too cold (AFP) */
#define USBIN_INT_RT_STS        0x1310
#define USBIN_PLUGIN            0x10
enum { CHG_STATE_TRICKLE = 1, CHG_STATE_TAPER = 4, CHG_STATE_TERMINATED = 5 };

/*
 * The battery (fg-gen4-batterydata-apollo-sun-5000mah.dtsi: qcom,nom-batt-capacity-mah 5000), for counting the charge
 * when the gauge gives no SOC; and the resistance that the estimate from the voltage takes the load out with (the
 * cell's and the path's, typical of such a cell: not measured on this phone).
 */
#define BATTERY_CAPACITY_MAH    5000
#define BATTERY_RESISTANCE_MOHM 100
#define TEMP_AGREE_QUARTERS     20              /* the two thermistor readings within 5 degrees of each other */

struct gauge {
	bool ok;                        /* the voltage and the current, each as its copy */
	const char *problem;
	int32_t voltage_mv, current_ma; /* the gauge's sign: more than 0 while the battery discharges */
	bool msoc_ok;                   /* the monotonic SOC and its copy agree */
	uint8_t msoc_raw, msoc_copy;
	bool en_ok, soc_rt_ok, missing_ok;
	uint8_t en_ctl, soc_rt, rr_rt;
	bool rr_temp_ok, info_temp_ok, temp_sts_ok;
	uint16_t rr_temp_raw, info_temp_raw;
	uint8_t temp_sts;
	int32_t rr_temp_q, info_temp_q;         /* in quarters of a degree Celsius */
	bool chg_ok, jeita_ok;
	uint8_t status1, status7, usbin;
};

static void
gauge_read(struct gauge *g)
{
	uint8_t a[4], b[4], t[2];
	const char *problem = NULL;

	memset(g, 0, sizeof(*g));
	for (int i = 0; i < FG_READ_TRIES && !g->ok; i++) {
		if ((problem = spmi_read(FG_INFO_VBATT_IBATT, a, 4)) != NULL ||
		    (problem = spmi_read(FG_INFO_VBATT_IBATT_CP, b, 4)) != NULL) {
			break;
		}
		g->ok = memcmp(a, b, sizeof(a)) == 0;
	}
	if (g->ok) {
		uint32_t v = (uint32_t)a[1] << 8 | a[0];
		int32_t i = (int16_t)((uint16_t)a[3] << 8 | a[2]);

		g->voltage_mv = (int32_t)((uint64_t)v * 122070 / 1000000);
		g->current_ma = (int32_t)((int64_t)i * 488281 / 1000000);
	} else {
		g->problem = problem != NULL ? problem : "the voltage and current do not agree with their copies";
	}
	for (int i = 0; i < FG_READ_TRIES; i++) {
		if (spmi_read(FG_SOC_MONOTONIC, t, 2) != NULL) {
			break;
		}
		g->msoc_raw = t[0];
		g->msoc_copy = t[1];
		if (t[0] == t[1]) {
			g->msoc_ok = true;
			break;
		}
	}
	g->en_ok = spmi_read(FG_SOC_EN_CTL, &g->en_ctl, 1) == NULL;
	g->soc_rt_ok = spmi_read(FG_SOC_INT_RT_STS, &g->soc_rt, 1) == NULL;
	g->missing_ok = spmi_read(FG_RR_INT_RT_STS, &g->rr_rt, 1) == NULL;
	if (spmi_read(FG_RR_BATT_TEMP, t, 2) == NULL) {
		g->rr_temp_ok = true;
		g->rr_temp_raw = (uint16_t)((t[1] & 0x03) << 8 | t[0]);
		/*
		 * whole degrees, not quarters: it read 0x21 (33) on a phone charging indoors, where Android said 35.5 C a few
		 * minutes later (28 October); as quarter degrees that was 8 C, and the cold-charging rule held the charger at 1 A
		 */
		g->rr_temp_q = 4 * (g->rr_temp_raw >= 0x200 ? (int32_t)g->rr_temp_raw - 0x400 : (int32_t)g->rr_temp_raw);
	}
	if (spmi_read(FG_INFO_BATT_TEMP, t, 2) == NULL) {
		g->info_temp_ok = true;
		g->info_temp_raw = (uint16_t)((t[1] & 0x07) << 8 | t[0]);
		g->info_temp_q = (int32_t)g->info_temp_raw - 1093;      /* 273.15 kelvin, in quarters */
	}
	g->temp_sts_ok = spmi_read(FG_INFO_TEMP_STS, &g->temp_sts, 1) == NULL;
	g->chg_ok = spmi_read(CHGR_STATUS_1, &g->status1, 1) == NULL &&
	    spmi_read(USBIN_INT_RT_STS, &g->usbin, 1) == NULL;
	g->jeita_ok = spmi_read(CHGR_STATUS_7, &g->status7, 1) == NULL;
}

/* the gauge's SOC is believed: read whole, the algorithm running, a battery there */
static bool
gauge_soc_valid(const struct gauge *g)
{
	return g->msoc_ok && g->en_ok && (g->en_ctl & FG_ALGORITHM_EN) && g->missing_ok &&
	       !(g->rr_rt & FG_RR_BATT_MISSING);
}

/* fg_get_msoc() as Xiaomi's kernel has it for apollo (no qcom,fg-increase-100soc-time-2; report_full not known here) */
static int
msoc_percent(uint8_t raw)
{
	if (raw == 0xff) {
		return 100;
	}
	if (raw == 0) {
		return 0;
	}
	return ((raw - 1) * 98 + 253 / 2) / 253 + 1;
}

static bool
temp_sane(int32_t quarters)
{
	return quarters >= CHG_TEMP_SANE_MIN * 4 && quarters <= CHG_TEMP_SANE_MAX * 4;
}

/*
 * The battery's temperature, whole degrees, or false. Two readings of the one thermistor, in two formats neither of
 * which has been checked on this phone: a reading in range is used; when both are, they must agree within 5 degrees,
 * or neither is believed. The charger's own comparators on the same thermistor (BATTERY_CHARGER_STATUS_7, the limits
 * the firmware gave it) have the last word: a "hot" from the charger with a reading under CHG_TEMP_WARM, or a "too
 * cold" with one from CHG_TEMP_COOL, and the reading is wrong. A temperature not believed is no temperature: the
 * charger's currents go to the floor and nothing is repaired (chg_tick()).
 */
static bool
temperature_choose(const struct gauge *g, int *celsius, const char **source)
{
	bool rr = g->rr_temp_ok && temp_sane(g->rr_temp_q), info = g->info_temp_ok && temp_sane(g->info_temp_q);
	int32_t q;

	if (rr && info) {
		if (g->rr_temp_q - g->info_temp_q > TEMP_AGREE_QUARTERS || g->info_temp_q - g->rr_temp_q > TEMP_AGREE_QUARTERS) {
			*source = "the two readings disagree";
			return false;
		}
		q = g->rr_temp_q, *source = "the gauge's thermistor ADC (BATT_INFO agrees)";
	} else if (rr) {
		q = g->rr_temp_q, *source = "the gauge's thermistor ADC";
	} else if (info) {
		q = g->info_temp_q, *source = "the gauge's BATT_INFO";
	} else {
		*source = "no reading in range";
		return false;
	}
	*celsius = q >= 0 ? q / 4 : -((-q + 3) / 4);
	if (g->jeita_ok && (g->status7 & CHGR_JEITA_HOT) && *celsius < CHG_TEMP_WARM) {
		*source = "the charger says hot, the reading does not";
		return false;
	}
	if (g->jeita_ok && (g->status7 & CHGR_JEITA_TOO_COLD) && *celsius >= CHG_TEMP_COOL) {
		*source = "the charger says too cold, the reading does not";
		return false;
	}
	return true;
}

/* the loader's soc_from_voltage() (xnuloader/loader.c), in thousandths of a percent */
static int32_t
soc_from_voltage(int32_t mv)
{
	if (mv <= 3300) {
		return 0;
	}
	if (mv < 3700) {
		return (mv - 3300) * 10000 / 400;
	}
	if (mv < 3900) {
		return 10000 + (mv - 3700) * 40000 / 200;
	}
	if (mv < 4200) {
		return 50000 + (mv - 3900) * 40000 / 300;
	}
	if (mv < 4400) {
		return 90000 + (mv - 4200) * 10000 / 200;
	}
	return 100000;
}

/*
 * The charge, in thousandths of a percent. The gauge's own SOC whenever it is believed (gauge_soc_valid()): it is the
 * PMIC's coulomb counter, corrected by the OCV of the profile the UEFI loaded, the number Android shows. When it is
 * not: from the gauge's last SOC if there was one, else from the voltage (the load taken out: the open-circuit
 * voltage is the terminal one plus the current times BATTERY_RESISTANCE_MOHM), the current counted from there on
 * (trapezoids between two reads, over BATTERY_CAPACITY_MAH), and pulled slowly toward the voltage's estimate --
 * by a sixtieth a tick (a time constant of five minutes) when the gauge never answered, a 720th (an hour) after it had.
 * A charger that has terminated says full.
 */
static int32_t soc_milli = -1;
static bool soc_had_gauge;

static const char *
soc_update(const struct gauge *g, int32_t previous_ma, uint64_t elapsed_ms, bool terminated)
{
	int32_t ocv = g->voltage_mv + g->current_ma * BATTERY_RESISTANCE_MOHM / 1000, estimate = soc_from_voltage(ocv);

	if (gauge_soc_valid(g)) {
		soc_milli = msoc_percent(g->msoc_raw) * 1000;
		soc_had_gauge = true;
		return "the gauge";
	}
	if (soc_milli < 0) {
		soc_milli = estimate;
		return "the voltage";
	}
	if (elapsed_ms > 10 * 60 * 1000) {
		elapsed_ms = 10 * 60 * 1000;
	}
	soc_milli -= (int32_t)(((int64_t)previous_ma + g->current_ma) * (int64_t)elapsed_ms / 2 /
	    (36 * BATTERY_CAPACITY_MAH));
	soc_milli += (estimate - soc_milli) / (soc_had_gauge ? 720 : 60);
	if (terminated) {
		soc_milli = 100000;
	}
	soc_milli = soc_milli < 0 ? 0 : soc_milli > 100000 ? 100000 : soc_milli;
	return soc_had_gauge ? "the gauge's last SOC and the current since" : "the voltage and the current since";
}

/* the first look, said whole: what the gauge is and what it gives */
static void
gauge_describe(void)
{
	static const char *const states[8] = { "inhibit", "trickle", "pre", "fullon", "taper", "term", "pause", "disabled" };
	struct gauge g;
	uint8_t id[2];
	const char *source = NULL;
	int celsius = -273;
	bool believed;

	for (unsigned i = 0; i < CHG_PERIPHERALS; i++) {
		if (chg_periph[i].gauge && chg_periph[i].found) {
			const char *problem = spmi_read((uint16_t)((chg_periph[i].id & 0xff) << 8 | 0x04), id, 2);

			if (problem == NULL) {
				say("apollo battery: %s: type %#x subtype %#x\n", chg_periph[i].name, id[0], id[1]);
			} else {
				say("apollo battery: %s: %s\n", chg_periph[i].name, problem);
			}
		}
	}
	gauge_read(&g);
	say("apollo battery: gauge: %s%d mV, %d mA (VBATT/IBATT 0x41a0, copies 0x41a6)\n",
	    g.ok ? "" : "NOT READ: ", g.voltage_mv, g.current_ma);
	if (!g.ok) {
		say("apollo battery: gauge: %s\n", g.problem);
	}
	say("apollo battery: gauge: monotonic SOC 0x4009 %#x, copy %#x (%s; %d%%), EN_CTL %#x (algorithm %s), "
	    "SOC_INT_RT_STS %#x (soc-ready %d), RR_INT_RT_STS %#x (battery %s): its SOC is %s\n", g.msoc_raw, g.msoc_copy,
	    g.msoc_ok ? "agree" : "DIFFER", msoc_percent(g.msoc_raw), g.en_ctl, !g.en_ok ? "??" :
	    (g.en_ctl & FG_ALGORITHM_EN) ? "runs" : "STOPPED", g.soc_rt, (g.soc_rt & FG_SOC_READY) != 0, g.rr_rt,
	    !g.missing_ok ? "??" : (g.rr_rt & FG_RR_BATT_MISSING) ? "MISSING" : "present",
	    gauge_soc_valid(&g) ? "VALID" : "NOT VALID (the voltage and the current are used)");
	believed = temperature_choose(&g, &celsius, &source);
	say("apollo battery: temperature: RRADC 0x4288 %s%#x (%d.%02d C as whole degrees), BATT_INFO 0x4150 %s%#x "
	    "(%d.%02d C as quarter kelvin), TEMP_STS %#x, the charger's JEITA %#x: %s %d C, from %s\n",
	    g.rr_temp_ok ? "" : "unread ", g.rr_temp_raw, g.rr_temp_q / 4, (g.rr_temp_q < 0 ? -g.rr_temp_q : g.rr_temp_q) % 4 * 25,
	    g.info_temp_ok ? "" : "unread ", g.info_temp_raw, g.info_temp_q / 4,
	    (g.info_temp_q < 0 ? -g.info_temp_q : g.info_temp_q) % 4 * 25, g.temp_sts, g.status7,
	    believed ? "believed:" : "NOT BELIEVED:", celsius, source);
	say("apollo battery: charger: %s, state %s, USBIN_INT_RT_STS %#x (a cable %s)\n", g.chg_ok ? "read" : "NOT READ",
	    states[g.status1 & 7], g.usbin, (g.usbin & USBIN_PLUGIN) ? "in" : "out");
}

static void
report(const struct hakios_battery_state *state)
{
	HakiosBatteryReport(state, sizeof(*state));
}

static void
apollo_battery_pmic_thread(__unused void *parameter, __unused wait_result_t result)
{
	static const char *const states[8] = { "inhibit", "trickle", "pre", "fullon", "taper", "term", "pause", "disabled" };
	struct gauge g;
	struct hakios_battery_state state;
	uint64_t reports = 0, misses = 0, fresh_interval, last_ok = 0, last_time = 0, now, ms;
	int32_t previous_ma = 0;
	int shown = -1, temperature_c = -273;
	bool fresh, was_fresh = true, temperature_valid, was_valid = true, was_gauge = true;
	const char *soc_source = "nothing yet", *temperature_source = "nothing yet", *last_problem = NULL;

	nanoseconds_to_absolutetime(CHG_FRESH_SECONDS * NSEC_PER_SEC, &fresh_interval);
	gauge_describe();
	for (unsigned tick = 0;; tick++) {
		gauge_read(&g);
		now = mach_absolute_time();
		temperature_valid = false;
		if (!g.ok) {
			misses++;
			if (g.problem != last_problem) {
				say("apollo battery: the gauge is not read: %s\n", g.problem);
				last_problem = g.problem;
			}
		} else {
			bool charger = g.chg_ok ? (g.usbin & USBIN_PLUGIN) != 0 : g.current_ma < -50;
			uint8_t chg_state = g.status1 & 7;
			bool charging = g.chg_ok ? chg_state >= CHG_STATE_TRICKLE && chg_state <= CHG_STATE_TAPER : g.current_ma < -50;
			bool terminated = g.chg_ok && charger && chg_state == CHG_STATE_TERMINATED;
			int celsius = -273;

			last_problem = NULL;
			absolutetime_to_nanoseconds(last_time != 0 ? now - last_time : 0, &ms);
			ms /= NSEC_PER_MSEC;
			soc_source = soc_update(&g, last_time != 0 ? previous_ma : g.current_ma, ms, terminated);
			previous_ma = g.current_ma;
			last_time = last_ok = now;
			if (gauge_soc_valid(&g) != was_gauge) {
				was_gauge = !was_gauge;
				say("apollo battery: the gauge's SOC %s (raw %#x/%#x, EN_CTL %#x, RR_INT_RT_STS %#x)%s\n",
				    was_gauge ? "is valid again" : "IS NOT VALID", g.msoc_raw, g.msoc_copy, g.en_ctl, g.rr_rt,
				    was_gauge ? "" : soc_had_gauge ? ": its last value and the current counted on" :
				    ": from the voltage and the current");
			}
			temperature_valid = temperature_choose(&g, &celsius, &temperature_source);
			temperature_c = temperature_valid ? celsius : -273;
			if (temperature_valid != was_valid) {
				was_valid = temperature_valid;
				say("apollo battery: the temperature is %s (%s; RRADC %#x, BATT_INFO %#x, JEITA %#x)\n",
				    temperature_valid ? "believed again" : "NOT BELIEVED", temperature_source, g.rr_temp_raw,
				    g.info_temp_raw, g.status7);
			}

			apollo_battery_soc = (soc_milli + 500) / 1000;
			apollo_battery_voltage_mv = g.voltage_mv >= 2500 && g.voltage_mv <= 5000 ? g.voltage_mv : -1;
			apollo_battery_current_ma = g.current_ma;
			apollo_battery_temperature_c = temperature_c;
			apollo_battery_present = !g.missing_ok || !(g.rr_rt & FG_RR_BATT_MISSING);
			apollo_battery_charger = charger;
			apollo_battery_charging = charging;
			apollo_battery_estimated = !gauge_soc_valid(&g);

			memset(&state, 0, sizeof(state));
			state.size = sizeof(state);
			state.percent_hundredths = apollo_battery_soc * 100;
			state.voltage_uv = apollo_battery_voltage_mv > 0 ? apollo_battery_voltage_mv * 1000 : 0;
			/* the power source wants more than 0 while charging: the gauge's sign is the other way */
			state.current_ua = clamp32(-(int64_t)g.current_ma * 1000);
			state.temperature_hundredths = clamp32((int64_t)apollo_battery_temperature_c * 100);
			state.present = apollo_battery_present;
			state.external = apollo_battery_charger;
			state.status = apollo_battery_charging ? 1 : apollo_battery_charger ?
			    (apollo_battery_soc >= 100 || terminated ? 4 : 3) : 2;
			state.design_uah = BATTERY_CAPACITY_MAH * 1000;
			report(&state);
			reports++;
			if (apollo_battery_soc != shown) {
				say("apollo battery: iOS is told %d%% (was %d%%; from %s; %d mV, %d mA, %d C, charger %d)\n",
				    apollo_battery_soc, shown, soc_source, apollo_battery_voltage_mv, g.current_ma, temperature_c,
				    apollo_battery_charger);
				shown = apollo_battery_soc;
			}
		}
		fresh = last_ok != 0 && now - last_ok < fresh_interval;
		if (chg_ready && chg_level >= 2 && fresh != was_fresh) {
			say("apollo charger: the gauge's readings are %s\n", fresh ? "fresh again: repairs resume" :
			    "STALE (no SPMI read of the gauge for 15 seconds): no writes to the charger, its watchdog is no longer petted");
		}
		was_fresh = fresh;
		if (tick % 12 == 0) {      /* once a minute */
			absolutetime_to_nanoseconds(last_ok != 0 ? now - last_ok : 0, &ms);
			say("apollo battery: %d%% (from %s), %d mV, %d mA, %d C (%s), battery %d, charger %d, charging %d, "
			    "charger state %s, gauge raw %#x (%llu reports, %llu misses; SPMI %llu reads, %llu failed; last good read %llu s ago)\n",
			    apollo_battery_soc, soc_source, apollo_battery_voltage_mv, apollo_battery_current_ma,
			    apollo_battery_temperature_c, temperature_source, apollo_battery_present, apollo_battery_charger,
			    apollo_battery_charging, g.chg_ok ? states[g.status1 & 7] : "??", g.msoc_raw, reports, misses,
			    bus.reads, bus.failed, last_ok != 0 ? ms / NSEC_PER_SEC : 0);
		}
		chg_tick(tick, fresh, fresh && temperature_valid, temperature_c, apollo_battery_charger);
		delay_for_interval(5000, NSEC_PER_MSEC);
	}
}

/* ---- the loader's page (apollo_battery_source=loader) ------------------------------------------------------ */

static void
apollo_battery_thread(__unused void *parameter, __unused wait_result_t result)
{
	struct loader_battery b;
	struct hakios_battery_state state;
	uint64_t reports = 0, misses = 0;
	int gauge = -1, shown = -1;     /* the fuel gauge's last own answer; what iOS was last given */
	bool was_estimate = false;
	/*
	 * Whether the loader still polls: its `polls` must have advanced within CHG_FRESH_SECONDS (it polls every five
	 * seconds while the UEFI's storage stack is in) for anything to be written to the charger. Until it is first seen
	 * to advance, the page counts as stale.
	 */
	uint32_t last_polls = shared->polls;
	uint64_t last_advance = 0, fresh_interval;
	bool fresh = false, was_fresh = false;
	bool temperature_valid = false;         /* from this tick's snapshot only: never an old value */
	int temperature_c = -273;

	nanoseconds_to_absolutetime(CHG_FRESH_SECONDS * NSEC_PER_SEC, &fresh_interval);
	for (unsigned tick = 0;; tick++) {
		bool got = snapshot(&b);

		temperature_valid = false;
		if (got && b.polls != last_polls) {
			last_polls = b.polls;
			last_advance = mach_absolute_time();
		}
		fresh = last_advance != 0 && mach_absolute_time() - last_advance < fresh_interval;
		if (chg_ready && chg_level >= 2 && fresh != was_fresh) {
			say("apollo charger: the loader's page is %s\n", fresh ? "fresh again: repairs resume" :
			    "STALE (the loader has stopped polling): no writes to the charger, its watchdog is no longer petted");
		}
		was_fresh = fresh;
		if (!got || !(b.flags & LOADER_BATTERY_VALID)) {
			misses++;
		} else {
			apollo_battery_soc = b.soc_percent < 0 ? 0 : b.soc_percent > 100 ? 100 : b.soc_percent;
			/*
			 * When the fuel gauge has not answered, the loader estimates the charge from the voltage
			 * (soc_from_voltage()), which follows the load: 59% one moment and 73% the next under a busy
			 * processor. Once the gauge has answered, its last answer stands in for such an estimate.
			 */
			if (!(b.flags & LOADER_BATTERY_SOC_ESTIMATED)) {
				gauge = apollo_battery_soc;
			} else if (gauge >= 0) {
				apollo_battery_soc = gauge;
			}
			if (((b.flags & LOADER_BATTERY_SOC_ESTIMATED) != 0) != was_estimate) {
				was_estimate = !was_estimate;
				say("apollo battery: the fuel gauge %s (the loader's estimate from %d mV: %d%%)%s\n",
				    was_estimate ? "HAS STOPPED ANSWERING" : "answers again", b.voltage_mv, b.soc_percent,
				    was_estimate && gauge >= 0 ? "; its last answer stands" : "");
			}
			apollo_battery_voltage_mv = (b.flags & LOADER_BATTERY_VOLTAGE_VALID) ? b.voltage_mv : -1;
			apollo_battery_current_ma = b.current_ma;
			apollo_battery_temperature_c = b.temperature_c;
			temperature_valid = fresh && b.temperature_c >= CHG_TEMP_SANE_MIN && b.temperature_c <= CHG_TEMP_SANE_MAX;
			temperature_c = b.temperature_c;
			apollo_battery_present = (b.flags & LOADER_BATTERY_PRESENT) != 0;
			apollo_battery_charger = (b.flags & LOADER_BATTERY_CHARGER_PRESENT) != 0;
			apollo_battery_charging = (b.flags & LOADER_BATTERY_CHARGING) != 0;
			apollo_battery_estimated = (b.flags & LOADER_BATTERY_SOC_ESTIMATED) != 0;

			memset(&state, 0, sizeof(state));
			state.size = sizeof(state);
			state.percent_hundredths = apollo_battery_soc * 100;
			state.voltage_uv = apollo_battery_voltage_mv > 0 ? apollo_battery_voltage_mv * 1000 : 0;
			/*
			 * The power source wants more than 0 while charging. Which sign Qualcomm's driver gives a charging current
			 * is not settled on this phone: while charging, its magnitude is taken; otherwise its negative.
			 */
			state.current_ua = clamp32((apollo_battery_charging ? 1 : -1) *
			    (b.current_ma < 0 ? -(int64_t)b.current_ma : (int64_t)b.current_ma) * 1000);
			state.temperature_hundredths = clamp32((int64_t)b.temperature_c * 100);
			/* the firmware's charger driver answered: there is a battery, whatever its presence call said */
			state.present = 1;
			state.external = apollo_battery_charger;
			state.status = apollo_battery_charging ? 1 : apollo_battery_charger ? (apollo_battery_soc >= 100 ? 4 : 3) : 2;
			report(&state);
			reports++;
			if (apollo_battery_soc != shown) {
				say("apollo battery: iOS is told %d%% (was %d%%; %d mV, %d mA, %d C, charger %d)\n", apollo_battery_soc, shown,
				    apollo_battery_voltage_mv, b.current_ma, b.temperature_c, apollo_battery_charger);
				shown = apollo_battery_soc;
			}
		}
		if (tick % 12 == 0) {      /* once a minute */
			printf("apollo battery: %d%%%s, %d mV, %d mA, %d C, battery %d, charger %d, charging %d "
			    "(%llu reports, %llu misses; the loader polled %u times, %u failed)\n", apollo_battery_soc,
			    apollo_battery_estimated ? " (from the voltage)" : "", apollo_battery_voltage_mv,
			    apollo_battery_current_ma, apollo_battery_temperature_c, apollo_battery_present, apollo_battery_charger,
			    apollo_battery_charging, reports, misses, shared->polls, shared->failures);
		}
		chg_tick(tick, fresh, temperature_valid, temperature_c, apollo_battery_charger);
		delay_for_interval(5000, NSEC_PER_MSEC);
	}
}

/* apollo_battery_source=loader: the loader's page mapped and checked */
static bool
loader_page_init(void)
{
	uint64_t pa = 0;
	vm_offset_t page;

	if (!PE_parse_boot_argn("battery_shared_pa", &pa, sizeof(pa)) || pa == 0) {
		printf("apollo battery: the loader gave no battery_shared_pa (is it a loader that knows apollo_battery=1?)\n");
		return false;
	}
	if ((pa & (LOADER_PAGE_SIZE - 1)) + sizeof(struct loader_battery) > LOADER_PAGE_SIZE) {
		printf("apollo battery: battery_shared_pa %#llx: the page's struct would run past its 16 KB mapping\n", pa);
		return false;
	}
	page = ml_io_map((vm_offset_t)(pa & ~(LOADER_PAGE_SIZE - 1)), LOADER_PAGE_SIZE);
	shared = (volatile struct loader_battery *)(page + (pa & (LOADER_PAGE_SIZE - 1)));
	if (shared->magic != LOADER_BATTERY_MAGIC || shared->version != LOADER_BATTERY_VERSION ||
	    shared->size < sizeof(struct loader_battery)) {
		printf("apollo battery: no loader battery page at %#llx (magic %#llx, version %u, size %u)\n", pa, shared->magic,
		    shared->version, shared->size);
		return false;
	}
	printf("apollo battery: the UEFI charger drivers' state at pa %#llx, read every 5 seconds\n", pa);
	return true;
}

void
apollo_battery_init(void)
{
	uint32_t enabled = 0;
	char source[16] = "spmi";
	bool from_loader;
	thread_t thread;

	if (!PE_parse_boot_argn("apollo_battery", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	(void)PE_parse_boot_arg_str("apollo_battery_source", source, sizeof(source));
	from_loader = strcmp(source, "loader") == 0;
	if (!from_loader && strcmp(source, "spmi") != 0) {
		say("apollo battery: apollo_battery_source=%s is not known (spmi or loader): spmi\n", source);
	}
	(void)PE_parse_boot_argn("apollo_charger", &chg_level, sizeof(chg_level));
	if (from_loader) {
		if (!loader_page_init()) {
			return;
		}
		if (chg_level != 0 && spmi_init()) {
			chg_init();
		}
	} else {
		bool gauge = true;

		if (!spmi_init()) {
			return;
		}
		for (unsigned i = 0; i < CHG_PERIPHERALS; i++) {
			gauge &= !chg_periph[i].gauge || chg_periph[i].found;
		}
		if (!gauge) {
			say("apollo battery: the PM8150B's fuel gauge has no channel: no battery (apollo_battery_source=loader for the old way)\n");
			return;
		}
		apollo_battery_from_pmic = 1;
		say("apollo battery: from the PM8150B's fuel gauge itself (FG gen4, slave 2: BATT_SOC 0x4000, BATT_INFO 0x4100, "
		    "RRADC 0x4200) and its charger, read over SPMI every 5 seconds; the loader's page is not used\n");
		if (chg_level != 0) {
			chg_init();
		}
	}
	if (kernel_thread_start(from_loader ? apollo_battery_thread : apollo_battery_pmic_thread, NULL, &thread) ==
	    KERN_SUCCESS) {
		thread_set_thread_name(thread, "apollo battery");
		thread_deallocate(thread);
	}
}
