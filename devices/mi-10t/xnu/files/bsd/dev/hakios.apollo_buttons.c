/*
 * The keys of the Xiaomi Mi 10T (apollo), with boot-arg apollo_buttons=1 (bsd/dev/hakios.apollo_input.c): power and volume
 * down are inputs of the PM8150's power-on block, volume up is the PM8150's GPIO6. All three are read through the
 * SPMI arbiter every 16 ms and handed to backboardd as button records through /dev/hakios-touch.
 *
 * From the phone's device tree: qcom,power-on@800 of the pm8150 (slave 0) has "kpdpwr" as interrupt 0 and "resin" as
 * interrupt 1 -- the power-on block of the first kind (Linux qpnp-power-on, gen1/gen2), whose status register has
 * power in bit 0 and volume down (RESIN) in bit 1; and gpio_keys/vol_up is GPIO6 of that PMIC, low while it is held.
 * The arbiter is the one at 0xc440000 (version 5, "qcom,spmi-pmic-arb"), this system being 0 ("qcom,ee").
 *
 * Read only: nothing is written to the PMIC, the way the firmware left it is the way it stays.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.apollo_input.h>
#include <sys/hakios.input.h>          /* hakios_touch_post(): the queue backboardd reads, shared */
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);

/* Into the loader's log as well: a phone that resets keeps only that (as in bsd/dev/hakios.apollo_touch.c). */
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

/* ---- the SPMI arbiter (Linux drivers/spmi/spmi-pmic-arb.c, version 5) ------------------------------ */

#define ARB_CORE_PHYS           0x0c440000UL    /* the version, and which peripheral each channel is */
#define ARB_CORE_SIZE           0x1100UL
#define ARB_CNFG_PHYS           0x0c40a000UL    /* which system owns each channel */
#define ARB_CNFG_SIZE           0x1000UL
#define ARB_OBSERVER_PHYS       0x0e600000UL    /* 0x10000 per system, 0x80 per channel: reads only */
#define ARB_OBSERVER_SIZE       0x10000UL
#define ARB_SYSTEM              0

#define ARB_VERSION             0x000
#define ARB_FEATURES            0x004           /* [10:0] channels */
#define ARB_CHANNEL(n)          (0x900 + 4 * (n))       /* [19:8] slave << 8 | peripheral */
#define ARB_OWNER(n)            (0x700 + 4 * (n))       /* [2:0] the system that owns channel n */
#define ARB_MAX_CHANNELS        512

#define OBSERVER_COMMAND        0x00
#define OBSERVER_STATUS         0x08
#define OBSERVER_DATA0          0x18
#define STATUS_DONE             0x1
#define STATUS_FAILURE          0x2
#define STATUS_DENIED           0x4
#define STATUS_DROPPED          0x8
#define COMMAND_READ            (1u << 27)      /* [11:4] register, [2:0] bytes - 1 */
#define READ_TIMEOUT_US         2000

static vm_offset_t arb_core, arb_cnfg, arb_observer;

/* one transfer at a time on the arbiter, for all the apollo drivers that use it: apollo_spmi_bus_lock (hakios.apollo_battery.c; Linux: one lock for the bus) */

struct peripheral {
	const char *name;
	uint16_t id;                    /* slave << 8 | peripheral */
	uint16_t channel;
	bool found;
};
static struct peripheral pon = { .name = "the PM8150's power-on block", .id = 0x008 };   /* registers 0x0800, slave 0 */
static struct peripheral pin = { .name = "GPIO6 of the PM8150", .id = 0x0c5 };           /* registers 0xc500, slave 0 */

static struct {
	uint64_t reads, failed;
} bus;

/* `count` (1 to 4) registers from `reg` on; NULL, or what went wrong */
static const char *
spmi_read(const struct peripheral *p, uint8_t reg, uint8_t *bytes, uint32_t count)
{
	vm_offset_t channel = arb_observer + 0x10000UL * ARB_SYSTEM + 0x80UL * p->channel;
	uint64_t start = mach_absolute_time(), interval;
	uint32_t status, data;

	nanoseconds_to_absolutetime(READ_TIMEOUT_US * (uint64_t)NSEC_PER_USEC, &interval);
	lck_spin_lock(&apollo_spmi_bus_lock);       /* (one lock for the arbiter: hakios.apollo_battery.c, hakios.apollo_backlight.c) */
	wr(channel + OBSERVER_COMMAND, COMMAND_READ | (uint32_t)reg << 4 | (count - 1));
	do {
		status = rd(channel + OBSERVER_STATUS);
	} while (!(status & STATUS_DONE) && mach_absolute_time() - start < interval);
	data = rd(channel + OBSERVER_DATA0);
	lck_spin_unlock(&apollo_spmi_bus_lock);

	bus.reads++;
	if (!(status & STATUS_DONE) || (status & (STATUS_DENIED | STATUS_FAILURE | STATUS_DROPPED))) {
		bus.failed++;
		return !(status & STATUS_DONE) ? "no answer from the arbiter" : (status & STATUS_DENIED) ? "the arbiter refuses" :
		       (status & STATUS_FAILURE) ? "the transfer failed" : "the transfer was dropped";
	}
	for (uint32_t i = 0; i < count; i++) {
		bytes[i] = (uint8_t)(data >> (8 * i));
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
		say("apollo buttons: %s is channel %u\n", p->name, p->channel);
	} else {
		say("apollo buttons: no channel is %s\n", p->name);
	}
	return p->found;
}

/* ---- the keys ----------------------------------------------------------------------------------------- */

#define REG_TYPE                0x04
#define REG_SUBTYPE             0x05
#define REG_RT_STATUS           0x10
#define PON_TYPE                0x01
#define PON_KPDPWR              0x01            /* gen1/gen2 power-on block: power in bit 0 ... */
#define PON_RESIN               0x02            /* ... and volume down in bit 1 */
#define REG_INT_LATCHED         0x18            /* the interrupts latched since they were last cleared (read only here) */
#define REG_PULL_CTL            0x70            /* the power-on block's pull-ups: */
#define PON_RESIN_PULL_UP       0x01            /* volume down's ... */
#define PON_KPDPWR_PULL_UP      0x02            /* ... and power's */
#define GPIO_TYPE               0x10
#define GPIO_LEVEL              0x01
#define GPIO_REG_MODE           0x40            /* [1:0] 0 input */
#define GPIO_REG_ENABLE         0x46            /* [7] */
#define POLL_MS                 16
#define MINUTE_TICKS            (60 * 1000 / POLL_MS)

static struct key {
	const char *name;
	uint32_t record;                /* HAKIOS_BUTTON_* */
	bool usable, live, last, down;
	uint32_t presses;
} power = { .name = "power", .record = HAKIOS_BUTTON_LOCK },
    volume_up = { .name = "volume up", .record = HAKIOS_BUTTON_VOLUME_UP },
    volume_down = { .name = "volume down", .record = HAKIOS_BUTTON_VOLUME_DOWN };

/*
 * One look at a key: a change counts once it has read the same twice (16 ms apart), and a key that was held when the
 * driver started is believed only after it has been seen released.
 */
static void
key_look(struct key *k, bool pressed)
{
	bool steady = pressed == k->last;

	k->last = pressed;
	if (!k->usable || !steady) {
		return;
	}
	if (!k->live) {
		k->live = !pressed;
		return;
	}
	if (pressed == k->down) {
		return;
	}
	k->down = pressed;
	if (pressed) {
		k->presses++;
	}
	say("apollo buttons: %s %s\n", k->name, pressed ? "pressed" : "released");
	hakios_touch_post(pressed ? HAKIOS_BUTTON_DOWN : HAKIOS_BUTTON_UP, k->record, 0, 0);
}

static void
apollo_buttons_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint8_t id[2], status = 0, gpio[7];
	uint32_t channels;
	const char *problem;

	arb_core = ml_io_map(ARB_CORE_PHYS, ARB_CORE_SIZE);
	arb_cnfg = ml_io_map(ARB_CNFG_PHYS, ARB_CNFG_SIZE);
	arb_observer = ml_io_map(ARB_OBSERVER_PHYS, ARB_OBSERVER_SIZE);
	if (arb_core == 0 || arb_cnfg == 0 || arb_observer == 0) {
		say("apollo buttons: cannot map the SPMI arbiter\n");
		goto out;
	}
	channels = rd(arb_core + ARB_FEATURES) & 0x7ff;
	if (channels > ARB_MAX_CHANNELS) {
		channels = ARB_MAX_CHANNELS;
	}
	say("apollo buttons: SPMI arbiter version %#x, %u channels\n", rd(arb_core + ARB_VERSION), channels);

	/* power and volume down: a power-on block, of any subtype (the PM8150's says 4) */
	if (find_channel(&pon, channels)) {
		problem = spmi_read(&pon, REG_TYPE, id, 2);
		if (problem == NULL && id[0] == PON_TYPE) {
			problem = spmi_read(&pon, REG_RT_STATUS, &status, 1);
		}
		if (problem == NULL && id[0] == PON_TYPE) {
			uint8_t pull = 0xff;

			/* (Linux qpnp-power-on: PULL_CTL at +0x70, bit 0 RESIN's pull-up, bit 1 KPDPWR's; Android sets both, qcom,pull-up) */
			(void)spmi_read(&pon, REG_PULL_CTL, &pull, 1);
			say("apollo buttons: power-on block: type %#x subtype %#x, status %#x (power %s, volume down %s), pull-ups %#x "
			    "(volume down's %s, power's %s)\n", id[0], id[1], status, (status & PON_KPDPWR) ? "held" : "released",
			    (status & PON_RESIN) ? "held" : "released", pull, (pull & PON_RESIN_PULL_UP) ? "on" : "OFF",
			    (pull & PON_KPDPWR_PULL_UP) ? "on" : "OFF");
			power.usable = volume_down.usable = true;
			power.last = status & PON_KPDPWR;
			volume_down.last = status & PON_RESIN;
		} else {
			say("apollo buttons: %s: %s\n", pon.name, problem != NULL ? problem : "not a power-on block");
		}
	}
	/* volume up: an enabled input */
	if (find_channel(&pin, channels)) {
		problem = spmi_read(&pin, REG_TYPE, id, 2);
		if (problem == NULL) {
			problem = spmi_read(&pin, GPIO_REG_MODE, gpio, 4);
		}
		if (problem == NULL) {
			problem = spmi_read(&pin, GPIO_REG_ENABLE, &gpio[6], 1);
		}
		if (problem == NULL) {
			problem = spmi_read(&pin, REG_RT_STATUS, &status, 1);
		}
		if (problem == NULL && id[0] == GPIO_TYPE && (gpio[0] & 3) == 0 && (gpio[6] & 0x80)) {
			say("apollo buttons: volume up's pin: an enabled input, level %s\n", (status & GPIO_LEVEL) ? "high (released)" :
			    "low (held)");
			volume_up.usable = true;
			volume_up.last = !(status & GPIO_LEVEL);
		} else {
			say("apollo buttons: volume up's pin: %s (type %#x, mode %#x, enable %#x)\n",
			    problem != NULL ? problem : "not an enabled input", id[0], gpio[0], gpio[6]);
		}
	}
	if (!power.usable && !volume_up.usable) {
		say("apollo buttons: no key can be read\n");
		goto out;
	}
	power.live = power.usable && !power.last;
	volume_down.live = volume_down.usable && !volume_down.last;
	volume_up.live = volume_up.usable && !volume_up.last;
	say("apollo buttons: watching%s%s%s every %u ms\n", power.usable ? " power" : "", volume_up.usable ? " volume-up" : "",
	    volume_down.usable ? " volume-down" : "", POLL_MS);

	for (unsigned tick = 1, seen_status = ~0u, seen_latched = ~0u, told = 0;; tick++) {
		if (power.usable && spmi_read(&pon, REG_RT_STATUS, &status, 1) == NULL) {
			uint8_t latched = 0;

			key_look(&power, status & PON_KPDPWR);
			key_look(&volume_down, status & PON_RESIN);
			/*
			 * Volume down is not seen on the phone (27 October: 6 presses of volume up, none of volume down, with RESIN's
			 * pull-up on): every change of the whole status and of the latched interrupts is said, so that a press shows
			 * wherever it is. At most 64 lines.
			 */
			(void)spmi_read(&pon, REG_INT_LATCHED, &latched, 1);
			if ((status != seen_status || latched != seen_latched) && told < 64) {
				say("apollo buttons: power-on block status %#x, latched %#x\n", status, latched);
				seen_status = status;
				seen_latched = latched;
				told++;
			}
		}
		if (volume_up.usable && spmi_read(&pin, REG_RT_STATUS, &status, 1) == NULL) {
			key_look(&volume_up, !(status & GPIO_LEVEL));
		}
		if (tick % MINUTE_TICKS == 0) {
			say("apollo buttons: %u presses of power, %u of volume up, %u of volume down; %llu reads, %llu failed\n",
			    power.presses, volume_up.presses, volume_down.presses, bus.reads, bus.failed);
		}
		if (bus.failed >= 1000 && bus.failed * 2 > bus.reads) {
			say("apollo buttons: more than half of %llu reads have failed: giving up\n", bus.reads);
			break;
		}
		delay_for_interval(POLL_MS, NSEC_PER_MSEC);
	}
out:
	thread_terminate(current_thread());
}

void
apollo_buttons_init(void)
{
	uint32_t enabled = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("apollo_buttons", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	if (kernel_thread_start(apollo_buttons_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_set_thread_name(thread, "apollo buttons");
		thread_deallocate(thread);
	}
}
