/*
 * The battery of the Xiaomi Mi 10T (apollo), with boot-arg apollo_battery=1 (bsd/dev/hakios.apollo_input.c).
 *
 * Nothing in this kernel talks to the PM8150B's fuel gauge: the phone's own UEFI does, with Xiaomi's Qualcomm charger
 * drivers (ChargerExDxe and QcomChargerDxeLA, in uefi/files/Binaries/apollo). The loader, which stays on the first
 * core, asks them every few seconds and keeps the answer on a page of its own (xnuloader/loader.c, charger_poll():
 * read only, no charging action taken) whose address it gives as battery_shared_pa. Here that page is read every five
 * seconds and handed to the power source iOS reads (iokit/Kernel/hakios.battery.cpp, HakiosBatteryReport(), shared),
 * and to the sysctls kern.apollo.battery_*.
 *
 * Boot-arg apollo_charger=N (the PM8150B's charger itself, SPMI slave 2: peripherals 0x10 to 0x16, through the arbiter
 * as hakios.apollo_backlight.c reaches it; a write only goes through a channel that system 0 owns): 1 logs its registers
 * (charge state, limits, power path, APSD, USBIN) once a minute and when the loader's charger_present changes; 2 also,
 * every five seconds while a cable is in, pets its watchdog (whose bite disables charging) and clears a USBIN suspend,
 * a disabled charge enable and a charge pause. No limit (ICL, FCC, FV) is written.
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
static int apollo_battery_charging, apollo_battery_estimated;

SYSCTL_NODE(_kern, OID_AUTO, apollo, CTLFLAG_RW | CTLFLAG_LOCKED, 0, "the Mi 10T's own drivers");
SYSCTL_INT(_kern_apollo, OID_AUTO, battery_soc, CTLFLAG_RD | CTLFLAG_LOCKED, &apollo_battery_soc, 0,
    "battery charge, 0..100 percent, or -1 if unknown");
SYSCTL_INT(_kern_apollo, OID_AUTO, battery_voltage_mv, CTLFLAG_RD | CTLFLAG_LOCKED, &apollo_battery_voltage_mv, 0,
    "battery voltage in millivolts, or -1 if unknown");
SYSCTL_INT(_kern_apollo, OID_AUTO, battery_current_ma, CTLFLAG_RD | CTLFLAG_LOCKED, &apollo_battery_current_ma, 0,
    "battery current as the UEFI's charger driver gives it, in milliamps");
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

/* ---- the PM8150B's charger (Linux's qcom-smb5; boot-arg apollo_charger=N) -------------------------------- */

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
static LCK_GRP_DECLARE(apollo_charger_grp, "apollo-charger");
static LCK_SPIN_DECLARE(apollo_charger_lock, &apollo_charger_grp);

struct chg_peripheral {
	const char *name;
	uint16_t id;                    /* slave << 8 | peripheral */
	uint16_t channel;
	bool found;
	bool writable;                  /* the channel is owned by system 0 (this one) */
	bool refused_said;
	vm_offset_t writes;             /* its page of the channel region */
};
static struct chg_peripheral chg_periph[] = {
	{ .name = "CHGR", .id = 0x210 }, { .name = "DCDC", .id = 0x211 }, { .name = "USBIN", .id = 0x213 },
	{ .name = "DCIN", .id = 0x214 }, { .name = "TYPEC", .id = 0x215 }, { .name = "MISC", .id = 0x216 },
};
#define CHG_PERIPHERALS (sizeof(chg_periph) / sizeof(chg_periph[0]))

static uint32_t chg_level;              /* apollo_charger: 0 nothing, 1 read, 2 read and repair */
static bool chg_ready;
static uint32_t chg_writes, chg_write_failures;

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

/* one byte at `address` (peripheral << 8 | offset, slave 2); NULL, or what went wrong */
static const char *
chg_read(uint16_t address, uint8_t *value)
{
	struct chg_peripheral *p = chg_for(address);
	vm_offset_t channel;
	uint64_t start = mach_absolute_time(), interval;
	uint32_t status, data;

	if (p == NULL || !p->found) {
		return "no channel";
	}
	channel = arb_observer + 0x10000UL * ARB_SYSTEM + 0x80UL * p->channel;
	nanoseconds_to_absolutetime(TRANSFER_TIMEOUT_US * (uint64_t)NSEC_PER_USEC, &interval);
	lck_spin_lock(&apollo_charger_lock);
	wr(channel + CHANNEL_COMMAND, OP_EXT_READL << 27 | (uint32_t)(address & 0xff) << 4 | 0);
	do {
		status = rd(channel + CHANNEL_STATUS);
	} while (!(status & STATUS_DONE) && mach_absolute_time() - start < interval);
	data = rd(channel + CHANNEL_RDATA0);
	lck_spin_unlock(&apollo_charger_lock);
	if (!(status & STATUS_DONE) || (status & (STATUS_DENIED | STATUS_FAILURE | STATUS_DROPPED))) {
		return chg_status_text(status);
	}
	*value = (uint8_t)data;
	return NULL;
}

/* one byte into `address`: only through a channel that system 0 owns; NULL, or what went wrong */
static const char *
chg_write(uint16_t address, uint8_t value)
{
	struct chg_peripheral *p = chg_for(address);
	vm_offset_t channel;
	uint64_t start = mach_absolute_time(), interval;
	uint32_t status;

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
	lck_spin_lock(&apollo_charger_lock);
	wr(channel + CHANNEL_WDATA0, value);
	wr(channel + CHANNEL_COMMAND, OP_EXT_WRITEL << 27 | (uint32_t)(address & 0xff) << 4 | 0);
	do {
		status = rd(channel + CHANNEL_STATUS);
	} while (!(status & STATUS_DONE) && mach_absolute_time() - start < interval);
	lck_spin_unlock(&apollo_charger_lock);
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
		say("apollo charger: no channel is %s (slave 2, peripheral %#x)\n", p->name, p->id & 0xff);
		return false;
	}
	owner = rd(arb_cnfg + ARB_OWNER(p->channel)) & 7;
	p->writable = owner == ARB_SYSTEM;
	say("apollo charger: %s is channel %u (owned by system %u%s)\n", p->name, p->channel, owner,
	    p->writable ? "" : ": read only");
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

static void
chg_currents(void)
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
	if (apollo_battery_temperature_c >= 50) {
		icl = 500, fcc = 1000, why = "THE BATTERY IS AT 50 DEGREES OR MORE";
	} else if (apollo_battery_temperature_c >= 45) {
		icl = icl > 1000 ? 1000 : icl, fcc = 1000, why = "the battery is at 45 degrees or more";
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
			say("apollo charger: the battery's current %u -> %u mA (battery at %d degrees)\n", raw * CHG_ICL_STEP_MA, fcc,
			    apollo_battery_temperature_c);
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

static void
chg_init(void)
{
	uint32_t channels;

	arb_core = ml_io_map(ARB_CORE_PHYS, ARB_CORE_SIZE);
	arb_cnfg = ml_io_map(ARB_CNFG_PHYS, ARB_CNFG_SIZE);
	arb_observer = ml_io_map(ARB_OBSERVER_PHYS, ARB_REGION_SIZE);
	channels = rd(arb_core + ARB_FEATURES) & 0x7ff;
	if (channels > ARB_MAX_CHANNELS) {
		channels = ARB_MAX_CHANNELS;
	}
	for (unsigned i = 0; i < CHG_PERIPHERALS; i++) {
		chg_find_channel(&chg_periph[i], channels);
	}
	chg_ready = true;
	say("apollo charger: level %u (%s)\n", chg_level, chg_level >= 3 ? "reads, repairs, and the currents from the source" : chg_level >= 2 ? "reads, and the minimal repairs" : "reads only");
}

static void
apollo_battery_thread(__unused void *parameter, __unused wait_result_t result)
{
	struct loader_battery b;
	struct hakios_battery_state state;
	uint64_t reports = 0, misses = 0;
	int gauge = -1, shown = -1;     /* the fuel gauge's last own answer; what iOS was last given */
	bool was_estimate = false;
	int last_charger = -1;

	for (unsigned tick = 0;; tick++) {
		if (!snapshot(&b) || !(b.flags & LOADER_BATTERY_VALID)) {
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
			state.current_ua = (apollo_battery_charging ? 1 : -1) * (b.current_ma < 0 ? -b.current_ma : b.current_ma) * 1000;
			state.temperature_hundredths = b.temperature_c * 100;
			/* the firmware's charger driver answered: there is a battery, whatever its presence call said */
			state.present = 1;
			state.external = apollo_battery_charger;
			state.status = apollo_battery_charging ? 1 : apollo_battery_charger ? (apollo_battery_soc >= 100 ? 4 : 3) : 2;
			HakiosBatteryReport(&state, sizeof(state));
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
		if (chg_ready) {
			if (tick % 12 == 0 || apollo_battery_charger != last_charger) {
				chg_log();
			}
			last_charger = apollo_battery_charger;
			if (chg_level >= 2) {
				chg_fix();
			}
			if (chg_level >= 3 && apollo_battery_charger) {
				chg_currents();
			}
		}
		delay_for_interval(5000, NSEC_PER_MSEC);
	}
}

void
apollo_battery_init(void)
{
	uint32_t enabled = 0;
	uint64_t pa = 0;
	vm_offset_t page;
	thread_t thread;

	if (!PE_parse_boot_argn("apollo_battery", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	if (!PE_parse_boot_argn("battery_shared_pa", &pa, sizeof(pa)) || pa == 0) {
		printf("apollo battery: the loader gave no battery_shared_pa (is it a loader that knows apollo_battery=1?)\n");
		return;
	}
	page = ml_io_map((vm_offset_t)(pa & ~(LOADER_PAGE_SIZE - 1)), LOADER_PAGE_SIZE);
	shared = (volatile struct loader_battery *)(page + (pa & (LOADER_PAGE_SIZE - 1)));
	if (shared->magic != LOADER_BATTERY_MAGIC || shared->version != LOADER_BATTERY_VERSION ||
	    shared->size < sizeof(struct loader_battery)) {
		printf("apollo battery: no loader battery page at %#llx (magic %#llx, version %u, size %u)\n", pa, shared->magic,
		    shared->version, shared->size);
		return;
	}
	printf("apollo battery: the UEFI charger drivers' state at pa %#llx, read every 5 seconds\n", pa);
	if (PE_parse_boot_argn("apollo_charger", &chg_level, sizeof(chg_level)) && chg_level != 0) {
		chg_init();
	}
	if (kernel_thread_start(apollo_battery_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
