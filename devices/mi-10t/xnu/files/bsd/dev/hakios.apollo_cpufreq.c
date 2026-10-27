/*
 * The Xiaomi Mi 10T's (apollo, SM8250) processor at its top speed, with boot-arg apollo_cpufreq=1
 * (bsd/dev/hakios.apollo_input.c). Everything this kernel draws, it draws on the processor (there is no GPU driver for the
 * Adreno 650 yet), so the speed the firmware left the cores at is what the user interface feels like.
 *
 * The phone's device tree: qcom,cpufreq-hw-epss with three frequency domains at 0x18591000, 0x18592000 and
 * 0x18593000 -- cores 0 to 3, 4 to 6, and 7. As Linux drivers/cpufreq/qcom-cpufreq-hw.c (its EPSS layout): the table
 * of frequencies at 0x100, a word a row ([31:30] source, 0 being the fixed 300 MHz; [7:0] multiple of 19.2 MHz), the
 * table ending where a row repeats; and the row wanted, written to 0x320. The firmware then sets the clock and the
 * voltage itself.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <vm/memory_types.h>
#include <kern/kalloc.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.apollo_input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

/* into the loader's log as well: the kernel's own message buffer keeps only the end of a long run */
extern void loader_log_putc(char c);

static int say(const char *format, ...) __printflike(1, 2);

static int
say(const char *format, ...)
{
	char line[200];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	return printf("%s", line);
}

#define EPSS_PHYS               0x18591000UL
#define EPSS_DOMAIN_SIZE        0x1000UL
#define EPSS_DOMAINS            3
#define EPSS_FREQ_TABLE         0x100
#define EPSS_TABLE_ROWS         40
#define EPSS_PERF_STATE         0x320
#define XO_KHZ                  19200
#define ALTERNATE_KHZ           300000

/* ---- the memory's bandwidth (boot-arg apollo_ddr=1, with apollo_cpufreq=1) -------------------------------------
 *
 * Every frame is drawn by the processor, and drawing is mostly copying: the memory bus's clock matters as much as
 * the cores'. Left alone it stays where the boot chain put it. A vote for it goes to RPMh as Linux's interconnect
 * driver makes one (drivers/interconnect/qcom/bcm-voter.c): the BCMs of the memory controller (MC0) and its shared
 * cache (SH0) at their top, and ACV with the application processor's bit -- the same three hakios.cpufreq.c votes
 * for with ddr=2, through TCS 1 of the sequencer (TCS 0 is hakios.apollo_adsp.c's; this runs while the system starts,
 * before hakios.apollo_pcie.c's thread uses TCS 1). How fast a core reads, writes and copies is measured before and after.
 */
extern vm_offset_t ml_io_map_unmappable(vm_offset_t phys_addr, vm_size_t size, unsigned int flags);

#define CMD_DB_PHYS             0x80860000UL
#define CMD_DB_SIZE             0x20000UL
#define RSC_PHYS                0x18220000UL
#define RSC_SIZE                0x10000UL
#define DDR_TCS                 1
#define TCS(reg)                (rsc + 0xd00 + 672 * DDR_TCS + (reg))
#define BCM_COMMIT              (1u << 30)
#define BCM_VALID               (1u << 29)
#define BCM_VOTE(ab, ib)        (BCM_COMMIT | BCM_VALID | ((uint32_t)(ab) & 0x3fff) << 14 | ((uint32_t)(ib) & 0x3fff))
#define BCM_ACV_APPS            8
#define DDR_TOP                 0x3fff

struct cmd_db_slave {
	uint16_t id, header_offset, data_offset, count, version, reserved[3];
};
struct cmd_db_header {
	uint32_t version;
	uint8_t magic[4];
	struct cmd_db_slave slave[8];
	uint32_t checksum, reserved;
	uint8_t data[];
};
struct cmd_db_entry {
	char id[8];
	uint32_t priority[2];
	uint32_t address;
	uint16_t length, offset;
};

static uint32_t
cmd_db_address(const char *name)
{
	static const struct cmd_db_header *db;
	char id[8] = { 0 };

	if (db == NULL) {
		db = (const struct cmd_db_header *)ml_io_map_unmappable(CMD_DB_PHYS, CMD_DB_SIZE, VM_WIMG_WCOMB);
	}
	if (db == NULL || db->magic[0] != 0xdb || db->magic[1] != 0x30 || db->magic[2] != 0x03 || db->magic[3] != 0x0c) {
		return 0;
	}
	strlcpy(id, name, sizeof(id));
	for (unsigned s = 0; s < 8; s++) {
		const struct cmd_db_entry *entry = (const struct cmd_db_entry *)(db->data + db->slave[s].header_offset);

		if (db->slave[s].id == 0 || db->slave[s].count > 512) {
			continue;
		}
		for (unsigned i = 0; i < db->slave[s].count; i++) {
			if (memcmp(entry[i].id, id, sizeof(id)) == 0) {
				return entry[i].address;
			}
		}
	}
	return 0;
}

/* One request through TCS 1, done when every command says completed (hakios.apollo_pcie.c: TCS 1's IRQ bit never sets) */
static bool
ddr_vote(uint32_t mc0, uint32_t sh0, uint32_t acv, uint32_t vote)
{
	vm_offset_t rsc = ml_io_map(RSC_PHYS, RSC_SIZE);
	uint32_t address[3] = { mc0, sh0, acv }, data[3] = { BCM_VOTE(vote, vote), BCM_VOTE(vote, vote), BCM_VOTE(0, BCM_ACV_APPS) };
	uint64_t deadline;

	if (rsc == 0 || *(volatile uint32_t *)TCS(0x18) == 0) {
		return false;                   /* (busy, or not mapped) */
	}
	*(volatile uint32_t *)(rsc + 0xd08) = 1u << DDR_TCS;
	for (unsigned c = 0; c < 3; c++) {
		*(volatile uint32_t *)TCS(0x30 + 20 * c) = 0x10008 | 0x100;     /* a 4-byte write, with a response */
		*(volatile uint32_t *)TCS(0x34 + 20 * c) = address[c];
		*(volatile uint32_t *)TCS(0x38 + 20 * c) = data[c];
	}
	*(volatile uint32_t *)TCS(0x10) = 7;
	*(volatile uint32_t *)TCS(0x1c) = 7;
	__asm__ volatile ("dsb sy" ::: "memory");
	*(volatile uint32_t *)TCS(0x14) &= ~(1u << 24);
	*(volatile uint32_t *)TCS(0x14) |= 1u << 16;
	*(volatile uint32_t *)TCS(0x14) |= 1u << 24;
	clock_interval_to_deadline(100, NSEC_PER_MSEC, &deadline);
	for (;;) {
		bool all = true;

		for (unsigned c = 0; c < 3; c++) {
			all = all && (*(volatile uint32_t *)TCS(0x3c + 20 * c) & (1u << 16));
		}
		if (all && *(volatile uint32_t *)TCS(0x18) != 0) {
			break;
		}
		if (mach_absolute_time() > deadline) {
			*(volatile uint32_t *)TCS(0x14) &= ~((1u << 24) | (1u << 16));
			*(volatile uint32_t *)TCS(0x1c) = 0;
			return false;
		}
		delay_for_interval(50, NSEC_PER_USEC);
	}
	*(volatile uint32_t *)TCS(0x14) &= ~((1u << 24) | (1u << 16));
	*(volatile uint32_t *)TCS(0x1c) = 0;
	return true;
}

/*
 * A vote for one bus clock manager (BCM) of RPMh by its cmd-db name, for the other apollo drivers (hakios.apollo_haptic.c: the
 * QUP wrapper 1's core clock, "QUP1", which no one but Android's geni-se driver votes for). Through TCS 1 as above;
 * false if RPMh has no such name or does not take the vote.
 */
bool apollo_rpmh_bcm_vote(const char *name, uint32_t ab, uint32_t ib);
void apollo_cmd_db_dump(void);

/* every name cmd-db has, by its slave (for finding what a resource is called on this phone) */
void
apollo_cmd_db_dump(void)
{
	const struct cmd_db_header *db;
	char line[300];

	(void)cmd_db_address("");                       /* (maps it) */
	db = (const struct cmd_db_header *)ml_io_map_unmappable(CMD_DB_PHYS, CMD_DB_SIZE, VM_WIMG_WCOMB);
	for (unsigned s = 0; db != NULL && s < 8; s++) {
		const struct cmd_db_entry *entry = (const struct cmd_db_entry *)(db->data + db->slave[s].header_offset);
		int at = snprintf(line, sizeof(line), "apollo rpmh: slave %u (id %u, %u names):", s, db->slave[s].id, db->slave[s].count);

		if (db->slave[s].id == 0 || db->slave[s].count > 512) {
			continue;
		}
		for (unsigned i = 0; i < db->slave[s].count; i++) {
			if (at > (int)sizeof(line) - 24) {
				say("%s\n", line);
				at = snprintf(line, sizeof(line), "apollo rpmh: ...");
			}
			at += snprintf(line + at, sizeof(line) - (size_t)at, " %.8s=%#x", entry[i].id, entry[i].address);
		}
		say("%s\n", line);
	}
}


bool
apollo_rpmh_bcm_vote(const char *name, uint32_t ab, uint32_t ib)
{
	vm_offset_t rsc = ml_io_map(RSC_PHYS, RSC_SIZE);
	uint32_t address = cmd_db_address(name);
	uint64_t deadline;

	if (address == 0 || rsc == 0 || *(volatile uint32_t *)TCS(0x18) == 0) {
		return false;
	}
	*(volatile uint32_t *)(rsc + 0xd08) = 1u << DDR_TCS;
	*(volatile uint32_t *)TCS(0x30) = 0x10008 | 0x100;     /* a 4-byte write, with a response */
	*(volatile uint32_t *)TCS(0x34) = address;
	*(volatile uint32_t *)TCS(0x38) = BCM_VOTE(ab, ib);
	*(volatile uint32_t *)TCS(0x10) = 1;
	*(volatile uint32_t *)TCS(0x1c) = 1;
	__asm__ volatile ("dsb sy" ::: "memory");
	*(volatile uint32_t *)TCS(0x14) &= ~(1u << 24);
	*(volatile uint32_t *)TCS(0x14) |= 1u << 16;
	*(volatile uint32_t *)TCS(0x14) |= 1u << 24;
	clock_interval_to_deadline(100, NSEC_PER_MSEC, &deadline);
	while (!((*(volatile uint32_t *)TCS(0x3c) & (1u << 16)) && *(volatile uint32_t *)TCS(0x18) != 0)) {
		if (mach_absolute_time() > deadline) {
			*(volatile uint32_t *)TCS(0x14) &= ~((1u << 24) | (1u << 16));
			*(volatile uint32_t *)TCS(0x1c) = 0;
			return false;
		}
		delay_for_interval(50, NSEC_PER_USEC);
	}
	*(volatile uint32_t *)TCS(0x14) &= ~((1u << 24) | (1u << 16));
	*(volatile uint32_t *)TCS(0x1c) = 0;
	return true;
}

/* MB a second that one core writes, reads and copies, 32 MB at a time (far more than any cache) */
static void
memory_speed(const char *when)
{
	const vm_size_t size = 32 << 20;
	uint64_t *buf = kalloc_data(2 * size, Z_WAITOK), t0, t[3], ns[3], sum = 0;

	if (buf == NULL) {
		return;
	}
	t0 = mach_absolute_time();
	for (vm_size_t i = 0; i < size / 8; i++) {
		buf[i] = i;
	}
	t[0] = mach_absolute_time() - t0;
	t0 = mach_absolute_time();
	for (vm_size_t i = 0; i < size / 8; i++) {
		sum += ((volatile uint64_t *)buf)[i];
	}
	t[1] = mach_absolute_time() - t0;
	t0 = mach_absolute_time();
	memcpy((uint8_t *)buf + size, buf, size);
	t[2] = mach_absolute_time() - t0;
	for (int k = 0; k < 3; k++) {
		absolutetime_to_nanoseconds(t[k], &ns[k]);
		ns[k] = ns[k] ? ns[k] : 1;
	}
	say("apollo ddr: %s: one core writes %llu MB/s, reads %llu MB/s, copies %llu MB/s%s\n", when,
	    32000000000ull / ns[0], 32000000000ull / ns[1], 32000000000ull / ns[2], sum == 1 ? "" : "");
	kfree_data(buf, 2 * size);
}

static void
memory_bandwidth(void)
{
	uint32_t enabled = 0, mc0, sh0, acv;

	if (!PE_parse_boot_argn("apollo_ddr", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	mc0 = cmd_db_address("MC0");
	sh0 = cmd_db_address("SH0");
	acv = cmd_db_address("ACV");
	if (mc0 == 0 || sh0 == 0 || acv == 0) {
		say("apollo ddr: RPMh has no MC0 (%#x), SH0 (%#x) or ACV (%#x): no vote\n", mc0, sh0, acv);
		return;
	}
	memory_speed("as found");
	if (!ddr_vote(mc0, sh0, acv, DDR_TOP)) {
		say("apollo ddr: RPMh did not take the vote\n");
		return;
	}
	delay_for_interval(20, NSEC_PER_MSEC);
	say("apollo ddr: the memory's bandwidth voted to the top (MC0 %#x, SH0 %#x, ACV %#x)\n", mc0, sh0, acv);
	memory_speed("voted to the top");
}

/* ---- heat (boot-arg apollo_thermal, default 1 with apollo_cpufreq=1) ---------------------------------------------
 *
 * Nothing else here lowers the cores' speed, and a phone held at its top speed gets hot. Once a second the SoC's
 * temperature sensors are read -- the phone's device tree, thermal-zones: tsens@c222000 (its readings at 0xc263000)
 * sensors 1 to 4 the small cores, 7 to 14 the big ones, 15 the GPU; tsens@c223000 (0xc265000) sensor 3 the memory and
 * 8 the GPU again -- and the hottest of them decides: from THERMAL_HOT on the big cores' groups go down a row of
 * their table each second (the small cores too from THERMAL_HOTTER), and under THERMAL_COOL back up a row, never
 * over the top. Android's own limit for the big cores begins at 75 degrees (cpu-1-N-step, cpufreq-1N-config). The
 * sensors' status words (tsens v2, as hakios.adreno.c reads its phone's): the reading in the low 12 bits, signed,
 * in tenths of a degree, and bit 21 when there is one.
 */
#define TSENS0_TM_PHYS          0x0c263000UL
#define TSENS1_TM_PHYS          0x0c265000UL
#define TSENS_STATUS(n)         (0xa0 + 4 * (n))
#define TSENS_VALID             (1u << 21)
#define THERMAL_HOT             700             /* tenths of a degree */
#define THERMAL_HOTTER          800
#define THERMAL_COOL            620

static vm_offset_t epss[EPSS_DOMAINS];
static uint32_t top_row[EPSS_DOMAINS], row_now[EPSS_DOMAINS], row_khz[EPSS_DOMAINS][EPSS_TABLE_ROWS];

/* a sensor's reading in tenths of a degree, or -2730 if it has none */
static int32_t
sensor(vm_offset_t tm, uint32_t n)
{
	for (unsigned again = 0; again < 16; again++) {
		uint32_t status = *(volatile uint32_t *)(tm + TSENS_STATUS(n));

		if (status & TSENS_VALID) {
			int32_t tenths = (int32_t)(status << 20) >> 20;

			/* (a reading no phone has -- a layout not as believed -- slows nothing down) */
			return tenths > 1500 || tenths < -400 ? -2730 : tenths;
		}
	}
	return -2730;
}

static void
set_row(uint32_t domain, uint32_t row)
{
	if (epss[domain] == 0 || row == row_now[domain]) {
		return;
	}
	*(volatile uint32_t *)(epss[domain] + EPSS_PERF_STATE) = row;
	row_now[domain] = row;
}

/*
 * While the display is off (apollo_cpufreq_screen(), from apollo_backlight_power()), every core group runs at its lowest
 * row: nothing is on the screen to hurry for, and the phone neither drains nor warms. Back on, they go straight to their
 * top rows again (unless the heat says otherwise, a second later).
 */
static volatile bool screen_off;

void
apollo_cpufreq_screen(bool on)
{
	screen_off = !on;
	/*
	 * Back on: the top rows at once, not at the thermal thread's next second -- the first frames after the screen
	 * comes back are drawn (and copied to the panel by the processors) right away, and at the screen-off rows they came
	 * out torn (25 October). The thermal thread takes over from there (and says so) on its next pass.
	 */
	if (on) {
		for (uint32_t domain = 0; domain < EPSS_DOMAINS; domain++) {
			set_row(domain, top_row[domain]);
		}
	}
}

static void
apollo_thermal_thread(__unused void *parameter, __unused wait_result_t result)
{
	bool was_off = false;

	static const uint8_t cpu_sensors[] = { 1, 2, 3, 4, 7, 8, 9, 10, 11, 12, 13, 14 };
	vm_offset_t tm0 = ml_io_map(TSENS0_TM_PHYS, 0x1000), tm1 = ml_io_map(TSENS1_TM_PHYS, 0x1000);
	int32_t hottest_seen = -2730;

	for (uint32_t tick = 0;; tick++) {
		int32_t cpu = -2730, gpu, ddr, hottest;

		for (unsigned i = 0; i < sizeof(cpu_sensors); i++) {
			int32_t t = sensor(tm0, cpu_sensors[i]);

			cpu = t > cpu ? t : cpu;
		}
		gpu = sensor(tm0, 15);
		gpu = sensor(tm1, 8) > gpu ? sensor(tm1, 8) : gpu;
		ddr = sensor(tm1, 3);
		hottest = cpu > gpu ? cpu : gpu;
		hottest = ddr > hottest ? ddr : hottest;
		hottest_seen = hottest > hottest_seen ? hottest : hottest_seen;

		if (screen_off != was_off) {
			was_off = screen_off;
			for (uint32_t domain = 0; domain < EPSS_DOMAINS; domain++) {
				set_row(domain, was_off ? 0 : top_row[domain]);
			}
			say("apollo thermal: the display is %s: the cores at %u, %u, %u MHz\n", was_off ? "off" : "on",
			    row_khz[0][row_now[0]] / 1000, row_khz[1][row_now[1]] / 1000, row_khz[2][row_now[2]] / 1000);
		}
		if (was_off) {
			/* the lowest rows stay */
		} else if (hottest >= THERMAL_HOT) {
			for (uint32_t domain = 1; domain < EPSS_DOMAINS; domain++) {
				set_row(domain, row_now[domain] > 0 ? row_now[domain] - 1 : 0);
			}
			if (hottest >= THERMAL_HOTTER) {
				set_row(0, row_now[0] > 0 ? row_now[0] - 1 : 0);
			}
		} else if (hottest < THERMAL_COOL) {
			for (uint32_t domain = 0; domain < EPSS_DOMAINS; domain++) {
				set_row(domain, row_now[domain] < top_row[domain] ? row_now[domain] + 1 : top_row[domain]);
			}
		}
		if (tick % 30 == 0) {
			say("apollo thermal: cores %d.%d, GPU %d.%d, memory %d.%d degrees (hottest so far %d.%d); speeds %u, %u, %u MHz\n",
			    cpu / 10, cpu % 10 < 0 ? -(cpu % 10) : cpu % 10, gpu / 10, gpu % 10 < 0 ? -(gpu % 10) : gpu % 10,
			    ddr / 10, ddr % 10 < 0 ? -(ddr % 10) : ddr % 10, hottest_seen / 10, hottest_seen % 10,
			    row_khz[0][row_now[0]] / 1000, row_khz[1][row_now[1]] / 1000, row_khz[2][row_now[2]] / 1000);
		}
		delay_for_interval(1000, NSEC_PER_MSEC);
	}
}

void
apollo_cpufreq_init(void)
{
	static const char *const group[EPSS_DOMAINS] = { "cores 0-3", "cores 4-6", "core 7" };
	uint32_t enabled = 0, thermal = 1;
	thread_t thread;

	if (!PE_parse_boot_argn("apollo_cpufreq", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	for (uint32_t domain = 0; domain < EPSS_DOMAINS; domain++) {
		vm_offset_t base = ml_io_map(EPSS_PHYS + domain * EPSS_DOMAIN_SIZE, EPSS_DOMAIN_SIZE);
		uint32_t top = 0, top_khz = 0, was;

		if (base == 0) {
			say("apollo cpufreq: cannot map domain %u\n", domain);
			continue;
		}
		for (uint32_t row = 0; row < EPSS_TABLE_ROWS; row++) {
			uint32_t entry = *(volatile uint32_t *)(base + EPSS_FREQ_TABLE + 4 * row);
			uint32_t khz = (entry >> 30) ? XO_KHZ * (entry & 0xff) : ALTERNATE_KHZ;

			if (row > 0 && khz <= top_khz) {
				break;
			}
			top = row;
			top_khz = khz;
			row_khz[domain][row] = khz;
		}
		epss[domain] = base;
		top_row[domain] = row_now[domain] = top;
		was = *(volatile uint32_t *)(base + EPSS_PERF_STATE);
		*(volatile uint32_t *)(base + EPSS_PERF_STATE) = top;
		delay_for_interval(20, NSEC_PER_MSEC);
		say("apollo cpufreq: %s: row %u -> %u of the table, %u MHz\n", group[domain], was, top, top_khz / 1000);
	}
	memory_bandwidth();
	PE_parse_boot_argn("apollo_thermal", &thermal, sizeof(thermal));
	if (thermal && kernel_thread_start(apollo_thermal_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
		say("apollo thermal: the cores slow down from %d degrees, and speed up again under %d\n", THERMAL_HOT / 10,
		    THERMAL_COOL / 10);
	}
}
