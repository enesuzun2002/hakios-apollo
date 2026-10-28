/*
 * The backlight of the Xiaomi Mi 10T (apollo), with boot-arg apollo_backlight=P (1 to 100: the brightness the run starts
 * with; without it the level the firmware left stays until iOS sets another) -- bsd/dev/hakios.apollo_input.c.
 *
 * The Mi 10T's panel is an LCD whose light is the PM8150L's WLED (the device tree's panel mdss_dsi_j3s_37_02_0a_dsc_video:
 * qcom,mdss-dsi-bl-pmic-control-type "bl_ctrl_wled", levels 2 to 0xfff; the node qcom,wled@d800 of the pmic at SPMI
 * slave 5, "qcom,pm8150l-spmi-wled": the control block at 0xd800 and the sink block at 0xd900, strings 0 to 2). It is a
 * WLED of the fifth kind, whose registers are those of Linux's drivers/video/backlight/qcom-wled.c (WLED5): the
 * brightness of module A (or B, whichever the first string takes its current from) is a 12-bit (or 15-bit) value at
 * sink +0x53/+0x54 (+0x63/+0x64), taken over when the sync register (+0x65) is toggled; the module's enable is bit 7
 * of the control block's register 0x46. The firmware has set all of that up, because it had the panel lit: only the
 * brightness and the enable are written here, and the registers are read and logged first, so that a run says what
 * it found.
 *
 * What the driver gives iOS: the brightness (iOS's 16.16 fixed-point IOMFBBrightnessLevel, set through
 * IOMobileFramebuffer's properties: iokit/Kernel/MiniIOMobileFramebuffer.cpp) and the display's power (its request for
 * power change: off dims the light to nothing, on gives it back). And for a look from outside, the sysctls
 * kern.apollo.backlight (percent) and kern.apollo.backlight_raw.
 *
 * The SPMI arbiter is the one hakios.apollo_buttons.c reads through (version 5, 0xc440000); writes go through the channel
 * region (the device tree's "chnls", 0xc600000: in version 5 a page of 0x10000 for each channel, pmic_arb_offset_v5; the
 * first tries wrote elsewhere and hung the phone), reads through the observer (0xe600000): Linux's
 * drivers/spmi/spmi-pmic-arb.c.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.apollo_input.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

/* into the loader's log as well (as the other apollo drivers) */
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

/* ---- the SPMI arbiter ------------------------------------------------------------------------------- */

#define ARB_CORE_PHYS           0x0c440000UL
#define ARB_CORE_SIZE           0x1100UL
#define ARB_CNFG_PHYS           0x0c40a000UL
#define ARB_CNFG_SIZE           0x1000UL
#define ARB_CHANNELS_PHYS       0x0c600000UL    /* the device tree's "chnls": 0x10000 per channel (v5: not per system) */
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
#define OP_EXT_WRITEL           0u              /* a register of 8 bits of address, 1 to 8 bytes */
#define OP_EXT_READL            1u
#define TRANSFER_TIMEOUT_US     2000

static vm_offset_t arb_core, arb_cnfg, arb_observer;
static LCK_GRP_DECLARE(apollo_backlight_grp, "apollo-backlight");
/*
 * The brightness's state (level, lit, wanted_fraction, what the sysctls show) and the writes that change it: iOS's
 * brightness (IOMFB's property, or the sysctl from backboardd), the display's power and the run's end come from
 * different threads, each through write_level() under this. (apollo_spmi_bus_lock, the arbiter's, is only around one transfer.)
 */
static LCK_MTX_DECLARE(apollo_backlight_state_lock, &apollo_backlight_grp);

struct peripheral {
	const char *name;
	uint16_t id;                    /* slave << 8 | peripheral */
	uint16_t channel;
	bool found;
	vm_offset_t writes;             /* its page of the channel region */
};
static struct peripheral ctrl = { .name = "the WLED's control block", .id = 0x5d8 };    /* slave 5, registers 0xd800 */
static struct peripheral sink = { .name = "the WLED's sink block", .id = 0x5d9 };       /* slave 5, registers 0xd900 */

static const char *
spmi_status_text(uint32_t status)
{
	return !(status & STATUS_DONE) ? "no answer from the arbiter" : (status & STATUS_DENIED) ? "the arbiter refuses" :
	       (status & STATUS_FAILURE) ? "the transfer failed" : "the transfer was dropped";
}

/* `count` (1 to 4) registers from `reg` on; NULL, or what went wrong */
static const char *
spmi_read(const struct peripheral *p, uint8_t reg, uint8_t *bytes, uint32_t count)
{
	vm_offset_t channel = arb_observer + 0x10000UL * ARB_SYSTEM + 0x80UL * p->channel;
	uint64_t start = mach_absolute_time(), interval;
	uint32_t status, data;

	nanoseconds_to_absolutetime(TRANSFER_TIMEOUT_US * (uint64_t)NSEC_PER_USEC, &interval);
	lck_spin_lock(&apollo_spmi_bus_lock);  /* (one lock for the arbiter, shared with the battery and buttons drivers) */
	wr(channel + CHANNEL_COMMAND, OP_EXT_READL << 27 | (uint32_t)reg << 4 | (count - 1));
	do {
		status = rd(channel + CHANNEL_STATUS);
	} while (!(status & STATUS_DONE) && mach_absolute_time() - start < interval);
	data = rd(channel + CHANNEL_RDATA0);
	lck_spin_unlock(&apollo_spmi_bus_lock);
	if (!(status & STATUS_DONE) || (status & (STATUS_DENIED | STATUS_FAILURE | STATUS_DROPPED))) {
		return spmi_status_text(status);
	}
	for (uint32_t i = 0; i < count; i++) {
		bytes[i] = (uint8_t)(data >> (8 * i));
	}
	return NULL;
}

/* `count` (1 to 4) bytes into the registers from `reg` on; NULL, or what went wrong */
static const char *
spmi_write(const struct peripheral *p, uint8_t reg, const uint8_t *bytes, uint32_t count)
{
	vm_offset_t channel = p->writes;
	uint64_t start = mach_absolute_time(), interval;
	uint32_t status, data = 0;

	for (uint32_t i = 0; i < count; i++) {
		data |= (uint32_t)bytes[i] << (8 * i);
	}
	nanoseconds_to_absolutetime(TRANSFER_TIMEOUT_US * (uint64_t)NSEC_PER_USEC, &interval);
	lck_spin_lock(&apollo_spmi_bus_lock);  /* (one lock for the arbiter, shared with the battery and buttons drivers) */
	wr(channel + CHANNEL_WDATA0, data);
	wr(channel + CHANNEL_COMMAND, OP_EXT_WRITEL << 27 | (uint32_t)reg << 4 | (count - 1));
	do {
		status = rd(channel + CHANNEL_STATUS);
	} while (!(status & STATUS_DONE) && mach_absolute_time() - start < interval);
	lck_spin_unlock(&apollo_spmi_bus_lock);
	if (!(status & STATUS_DONE) || (status & (STATUS_DENIED | STATUS_FAILURE | STATUS_DROPPED))) {
		return spmi_status_text(status);
	}
	return NULL;
}

/* the peripheral's channel: the first, unless a later one is this system's (as Linux chooses) */
static bool
find_channel(struct peripheral *p, uint32_t channels)
{
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
	if (p->found) {
		say("apollo backlight: %s is channel %u (owned by system %u)\n", p->name, p->channel,
		    rd(arb_cnfg + ARB_OWNER(p->channel)) & 7);
	} else {
		say("apollo backlight: no channel is %s\n", p->name);
	}
	return p->found;
}

/* ---- the WLED (Linux's qcom-wled.c, WLED5) ---------------------------------------------------------- */

#define WLED_CTRL_TYPE          0x04
#define WLED_CTRL_SUBTYPE       0x05
#define WLED_CTRL_MOD_EN        0x46            /* bit 7: the module on */
#define WLED_CTRL_MOD_EN_BIT    0x80
#define WLED5_SINK_MOD_EN(m)    ((m) ? 0x60 : 0x50)             /* bit 7 */
#define WLED5_SINK_SRC_SEL(m)   ((m) ? 0x61 : 0x51)
#define WLED5_SINK_WIDTH(m)     ((m) ? 0x62 : 0x52)             /* 0: 12 bits, 1: 15 bits */
#define WLED5_SINK_BRIGHT(m)    ((m) ? 0x63 : 0x53)             /* low byte; the high byte follows */
#define WLED5_SINK_SYNC         0x65                            /* bit 0: module A, bit 1: module B */
#define WLED5_SINK_STR_SRC(n)   (0x73 + 0x10 * (n))             /* 0: module A, 1: module B */
#define WLED5_LOW_LIMIT_PERMILLE 1                              /* the lower limit of a lit WLED5: 0.1% */

static bool ready;                      /* the WLED was read and is as expected: brightness may be written */
static uint32_t module;                 /* 0: A, 1: B: the module the first string takes its current from */
static uint32_t maximum = 0xfff;        /* the brightness register's top: 12 or 15 bits */
static uint32_t level;                  /* the brightness register's value now */
static bool lit = true;                 /* the module is enabled */
static uint32_t wanted_fraction = 0x8000;       /* the last brightness iOS asked for, 16.16: what a power-on gives back */
static int raw_shown, percent_shown;    /* the sysctls */

static void
log_registers(void)
{
	static const uint8_t ctrl_regs[] = { 0x04, 0x05, 0x08, 0x10, 0x46, 0x48, 0x4c, 0x4d, 0x4e, 0x5f };
	static const uint8_t sink_regs[] = { 0x46, 0x47, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65,
		0x72, 0x73, 0x82, 0x83, 0x92, 0x93, 0xa2, 0xa3 };
	char line[200];
	int at;
	uint8_t value;

	at = snprintf(line, sizeof(line), "apollo backlight: control block:");
	for (unsigned i = 0; i < sizeof(ctrl_regs); i++) {
		at += snprintf(line + at, sizeof(line) - (size_t)at, " %02x=", ctrl_regs[i]);
		if (spmi_read(&ctrl, ctrl_regs[i], &value, 1) == NULL) {
			at += snprintf(line + at, sizeof(line) - (size_t)at, "%02x", value);
		} else {
			at += snprintf(line + at, sizeof(line) - (size_t)at, "??");
		}
	}
	say("%s\n", line);
	at = snprintf(line, sizeof(line), "apollo backlight: sink block:");
	for (unsigned i = 0; i < sizeof(sink_regs); i++) {
		at += snprintf(line + at, sizeof(line) - (size_t)at, " %02x=", sink_regs[i]);
		if (spmi_read(&sink, sink_regs[i], &value, 1) == NULL) {
			at += snprintf(line + at, sizeof(line) - (size_t)at, "%02x", value);
		} else {
			at += snprintf(line + at, sizeof(line) - (size_t)at, "??");
		}
	}
	say("%s\n", line);
}

/* the brightness register as the hardware has it */
static bool
read_level(uint32_t *value)
{
	uint8_t bytes[2];

	if (spmi_read(&sink, WLED5_SINK_BRIGHT(module), bytes, 2) != NULL) {
		return false;
	}
	*value = (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8;
	return true;
}

static void
publish(void)
{
	raw_shown = (int)level;
	percent_shown = lit ? (int)((uint64_t)level * 100 / maximum) : 0;
}

/* Linux's wled5_set_brightness, wled5_mod_sync_toggle and wled_module_enable, in the order wled_update_status has them */
static bool
write_level(uint32_t brightness)
{
	uint8_t bytes[2], sync;
	const char *problem;
	uint32_t low = maximum * WLED5_LOW_LIMIT_PERMILLE / 1000;

	if (brightness > maximum) {
		brightness = maximum;
	}
	if (brightness != 0 && brightness < low) {
		brightness = low;
	}
	if (brightness != 0) {
		bytes[0] = (uint8_t)brightness;
		bytes[1] = (uint8_t)(brightness >> 8);
		if ((problem = spmi_write(&sink, WLED5_SINK_BRIGHT(module), bytes, 2)) != NULL) {
			say("apollo backlight: the brightness is not taken: %s\n", problem);
			return false;
		}
		sync = (uint8_t)(1u << module);
		if ((problem = spmi_write(&sink, WLED5_SINK_SYNC, &sync, 1)) != NULL) {
			say("apollo backlight: the sync is not taken: %s\n", problem);
			return false;
		}
		sync = 0;
		spmi_write(&sink, WLED5_SINK_SYNC, &sync, 1);
		level = brightness;
	}
	if ((brightness != 0) != lit) {
		uint8_t enable;

		if (spmi_read(&ctrl, WLED_CTRL_MOD_EN, &enable, 1) != NULL) {
			return false;
		}
		enable = (uint8_t)((enable & ~WLED_CTRL_MOD_EN_BIT) | (brightness != 0 ? WLED_CTRL_MOD_EN_BIT : 0));
		if ((problem = spmi_write(&ctrl, WLED_CTRL_MOD_EN, &enable, 1)) != NULL) {
			say("apollo backlight: the module's enable is not taken: %s\n", problem);
			return false;
		}
		lit = brightness != 0;
	}
	publish();
	return true;
}

/*
 * iOS's brightness is 0 to 1 (16.16 fixed point) and perceived, the register's value is a current: the register is
 * taken as the square of the brightness (a power of 2.0 for 2.2), never under 8 of 4095 so that the lowest setting
 * still shows something.
 */
static uint32_t
level_for(uint32_t fraction)
{
	uint64_t square;
	uint32_t floor = maximum * 8 / 0xfff;

	if (fraction > 0x10000) {
		fraction = 0x10000;
	}
	square = ((uint64_t)fraction * fraction) >> 16;
	square = square * maximum >> 16;
	return square < floor ? floor : (uint32_t)square;
}

/* iOS asks for a brightness: 0 (dark) to 0x10000 (full), as IOMFBBrightnessLevel has it */
void
apollo_backlight_set(uint32_t fraction)
{
	if (!ready) {
		return;
	}
	lck_mtx_lock(&apollo_backlight_state_lock);
	wanted_fraction = fraction > 0x10000 ? 0x10000 : fraction;
	if (lit) {
		write_level(level_for(wanted_fraction));
	}
	lck_mtx_unlock(&apollo_backlight_state_lock);
}

/* the display is switched off (the light with it) or on again (at the brightness that was last asked for) */
void
apollo_backlight_power(bool on)
{
	lck_mtx_lock(&apollo_backlight_state_lock);
	apollo_touch_screen(on);        /* the touch follows the screen, with or without the backlight driver */
	apollo_cpufreq_screen(on);      /* and the cores slow down while it is off */
	if (!ready || on == lit) {
		lck_mtx_unlock(&apollo_backlight_state_lock);
		return;
	}
	if (on) {
		write_level(level_for(wanted_fraction));
	} else {
		write_level(0);
	}
	lck_mtx_unlock(&apollo_backlight_state_lock);
	say("apollo backlight: the display is %s\n", on ? "on" : "off");
}

/* for the run's end: the light as the firmware had it (on, at the level it was at), whatever iOS did */
static uint32_t level_found;

void
apollo_backlight_stop(void)
{
	if (ready) {
		lck_mtx_lock(&apollo_backlight_state_lock);
		write_level(level_found != 0 ? level_found : maximum / 2);
		lck_mtx_unlock(&apollo_backlight_state_lock);
	}
}

/* ---- sysctls ----------------------------------------------------------------------------------------- */

SYSCTL_DECL(_kern_apollo);
SYSCTL_INT(_kern_apollo, OID_AUTO, backlight_raw, CTLFLAG_RD | CTLFLAG_LOCKED, &raw_shown, 0,
    "the WLED's brightness register: 0 to 4095 (or 32767)");

static int
sysctl_backlight_percent(__unused struct sysctl_oid *oid, __unused void *arg1, __unused int arg2,
    struct sysctl_req *req)
{
	int value, error;

	/*
	 * The sysctl is compiled into every phone's kernel: where the Mi 10T's backlight is not driven (another phone, or
	 * the WLED not as expected) it says so, read or written -- userland/src/inject.c's brightness_slider() takes any
	 * error from reading it as "no backlight of ours" and installs nothing.
	 */
	if (!ready) {
		return ENODEV;
	}
	value = percent_shown;
	error = sysctl_handle_int(oid, &value, 0, req);
	if (error == 0 && req->newptr != USER_ADDR_NULL) {
		if (value < 0 || value > 100) {
			return EINVAL;
		}
		apollo_backlight_set((uint32_t)value * 0x10000 / 100);
	}
	return error;
}
/* (anybody may set it: the brightness slider is backboardd's, which runs as mobile -- userland/src/inject.c) */
SYSCTL_PROC(_kern_apollo, OID_AUTO, backlight, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_ANYBODY | CTLFLAG_LOCKED, 0, 0,
    sysctl_backlight_percent, "I", "the backlight, 0 to 100 percent (a perceived scale)");

/* ---- start ------------------------------------------------------------------------------------------- */

void
apollo_backlight_init(void)
{
	uint32_t percent = 0, channels;
	uint8_t id[2], enable, source;
	uint32_t now = 0;
	const char *problem;

	if (!PE_parse_boot_argn("apollo_backlight", &percent, sizeof(percent)) || percent == 0) {
		return;
	}
	arb_core = ml_io_map(ARB_CORE_PHYS, ARB_CORE_SIZE);
	arb_cnfg = ml_io_map(ARB_CNFG_PHYS, ARB_CNFG_SIZE);
	arb_observer = ml_io_map(ARB_OBSERVER_PHYS, ARB_REGION_SIZE);
	channels = rd(arb_core + ARB_FEATURES) & 0x7ff;
	if (channels > ARB_MAX_CHANNELS) {
		channels = ARB_MAX_CHANNELS;
	}
	if (!find_channel(&ctrl, channels) || !find_channel(&sink, channels)) {
		return;
	}
	ctrl.writes = ml_io_map(ARB_CHANNELS_PHYS + 0x10000UL * ctrl.channel, 0x1000);
	sink.writes = ml_io_map(ARB_CHANNELS_PHYS + 0x10000UL * sink.channel, 0x1000);
	if ((problem = spmi_read(&ctrl, WLED_CTRL_TYPE, id, 2)) != NULL) {
		say("apollo backlight: cannot read the WLED: %s\n", problem);
		return;
	}
	say("apollo backlight: the WLED's control block is type %#x, subtype %#x\n", id[0], id[1]);
	log_registers();

	/* the module the first string takes its current from, and how wide its brightness is */
	if (spmi_read(&sink, WLED5_SINK_STR_SRC(0), &source, 1) != NULL) {
		return;
	}
	module = (source & 1) ? 1 : 0;
	if (spmi_read(&sink, WLED5_SINK_WIDTH(module), &source, 1) != NULL) {
		return;
	}
	maximum = (source & 1) ? 0x7fff : 0xfff;
	if (!read_level(&level) || spmi_read(&ctrl, WLED_CTRL_MOD_EN, &enable, 1) != NULL) {
		return;
	}
	lit = (enable & WLED_CTRL_MOD_EN_BIT) != 0;
	level_found = level;
	publish();
	say("apollo backlight: module %c, %u bits: the firmware left it %s at %u of %u (%u%%)\n", 'A' + module,
	    maximum == 0x7fff ? 15 : 12, lit ? "on" : "OFF", level, maximum, (uint32_t)((uint64_t)level * 100 / maximum));
	if (!lit) {
		say("apollo backlight: the WLED is off: left alone\n");
		return;
	}
	ready = true;
	/* P percent of perceived brightness */
	apollo_backlight_set((percent > 100 ? 100 : percent) * 0x10000 / 100);
	say("apollo backlight: set to %u%%: the register reads %u now (wrote %u)\n", percent > 100 ? 100 : percent,
	    read_level(&now) ? now : 0, level);
}
