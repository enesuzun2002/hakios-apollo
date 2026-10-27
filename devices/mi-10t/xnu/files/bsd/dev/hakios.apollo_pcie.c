/*
 * The Wi-Fi of the Xiaomi Mi 10T (apollo), step W1: its QCA6390 brought onto the SoC's first PCIe link, and seen
 * there (vendor 0x17cb, device 0x1101). Boot-arg apollo_wifi=N (N >= 1), on device=apollo only (bsd/dev/hakios.apollo_input.c).
 *
 * Neither the phone's UEFI nor the loader touches PCIe: everything is done here, in the order Linux does it
 * (drivers/pci/controller/dwc/pcie-qcom.c, ops 1_9_0 = 2_7_0; drivers/phy/qualcomm/phy-qcom-qmp-pcie.c;
 * drivers/power/sequencing/pwrseq-qcom-wcn.c), with the numbers of the phone's own device tree:
 *
 *   power   RPMh, the application processor's sequencer, TCS 1 (TCS 0 is hakios.apollo_adsp.c's): the QCA6390's rails as
 *           qcom,cnss-qca6390 asks them -- smpa6 0.95 V (aon, dig), smpa4 1.8 V (io), smpa5 1.9 V (rfa1), smpc8
 *           1.35 V (rfa2), smpf2 0.95 V (the PMU's, qcom,cmd_db_name) -- and PCIe's (qcom,pcie@1c00000): ldoa5
 *           0.88 V and ldoa9 1.2 V in high-power mode, cx.lvl at level 0x100; then WLAN_EN (GPIO 20) high, 100 ms
 *           after the rails, and 60 ms for the chip to wake
 *   clocks  the pcie_0 power domain (GDSCR 0x16b004), the six pcie_0 branches by vote (0x52008 bits 0-5), aux from
 *           XO at 19.2 MHz, the PHY's refgen from GPLL0 at 100 MHz, the TBU clocks, the Wi-Fi clock reference
 *   pins    CLKREQ (GPIO 80, function pci_e0), PERST# (GPIO 79, low until the link is to train), WAKE (GPIO 81)
 *   PHY     the PHY's reset (GCC 0x6c01c), the 99 writes of the tree's qcom,phy-sequence at 0x1c06000, and the PCS
 *           saying it is ready (0x814 bit 6 clear)
 *   link    the controller's reset (GCC 0x6b000), PARF set up as a root complex, PERST# let go after 100 ms,
 *           the link trained (LTSSM), and the device's first configuration word read through the iATU
 *
 * Nothing here is touched before what feeds it runs: a register of an unclocked block hangs the bus.
 * apollo_wifi_quiesce() (osfmk/arm64/loader_return.c) puts PERST# and WLAN_EN down again on any way back.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <vm/memory_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.apollo_input.h>
#include <sys/hakios.apollo_wifi.h>
#include <sys/errno.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_offset_t ml_io_map_unmappable(vm_offset_t phys_addr, vm_size_t size, unsigned int flags);
extern kern_return_t thread_terminate(thread_t thread);
extern uint64_t kvtophys(vm_offset_t va);
/* osfmk/arm64/gic_v3_irq.c, as hakios.lx.c declares them */
extern kern_return_t gic_v3_spi_attach(uint32_t intid, bool edge, void (*handler)(void *context, uint32_t intid), void *context);
extern void gic_v3_spi_mask(uint32_t intid);
extern void gic_v3_spi_unmask(uint32_t intid);
extern void gic_v3_spi_detach(uint32_t intid);
extern uint64_t gic_v3_spi_count(uint32_t intid);

/* into the loader's log as well (as the other apollo drivers): a phone that resets keeps only that */
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
	__asm__ volatile ("dsb sy" ::: "memory");
}

static void
sleep_us(uint32_t us)
{
	delay_for_interval(us, NSEC_PER_USEC);
}

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

static uint64_t
now_us(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / NSEC_PER_USEC;
}

/* ---- pins (TLMM: one page per pin, all four on the north tile) ------------------------------------- */

#define TLMM_NORTH_PHYS         0x0f900000UL
#define PIN_WLAN_EN             20
#define PIN_PERST               79
#define PIN_CLKREQ              80
#define PIN_WAKE                81
#define PIN_FUNCTION_PCI_E0     1
#define PULL_NONE               0
#define PULL_DOWN               1
#define PULL_UP                 3

static vm_offset_t pin_wlan_en, pins_pcie;      /* pin 20's page; pins 79 to 81's three */

static vm_offset_t
pin(unsigned n)
{
	return n == PIN_WLAN_EN ? pin_wlan_en : pins_pcie + 0x1000UL * (n - PIN_PERST);
}

static void
pin_setup(unsigned n, unsigned function, unsigned pull, unsigned ma, bool output, bool high)
{
	if (output) {
		wr(pin(n) + 4, high ? 2 : 0);   /* the level first */
	}
	wr(pin(n), pull | function << 2 | (ma / 2 - 1) << 6 | (output ? 1u << 9 : 0));
}

static void
pin_set(unsigned n, bool high)
{
	wr(pin(n) + 4, high ? 2 : 0);
}

/* ---- RPMh: the command database, and TCS 1 of the application processor's sequencer ---------------- */

#define CMD_DB_PHYS             0x80860000UL
#define CMD_DB_SIZE             0x20000UL
#define RSC_PHYS                0x18220000UL    /* rsc@18200000, DRV 2 */
#define RSC_SIZE                0x10000UL
#define RSC_TCS_BASE            0xd00
#define RSC_IRQ_STATUS          (RSC_TCS_BASE + 0x04)
#define RSC_IRQ_CLEAR           (RSC_TCS_BASE + 0x08)
#define RSC_TCS(n)              (RSC_TCS_BASE + 672 * (n))
#define TCS_WAIT_FOR_DONE       0x10
#define TCS_CONTROL             0x14
#define TCS_STATUS              0x18
#define TCS_ENABLED             0x1c
#define TCS_COMMAND(c)          (0x30 + 20 * (c))
#define TCS_ACTIVE_MODE         (1u << 16)
#define TCS_TRIGGER             (1u << 24)
#define MSGID_WRITE_WAITED      (8u | 1u << 8 | 1u << 16)
#define WIFI_TCS                1               /* the second TCS for requests made at once; hakios.apollo_adsp.c has the first */
#define VRM_VOLTAGE             0x0             /* millivolts */
#define VRM_ENABLE              0x4
#define VRM_MODE                0x8
#define LDO_MODE_HPM            7

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

static const struct cmd_db_header *cmd_db;
static vm_offset_t rsc;

/* A resource's address in RPMh (0: not there), and its level table (ARC resources) */
static uint32_t
cmd_db_find(const char *name, const uint8_t **levels, uint32_t *length)
{
	char id[8] = { 0 };

	if (cmd_db == NULL) {
		cmd_db = (const struct cmd_db_header *)ml_io_map_unmappable(CMD_DB_PHYS, CMD_DB_SIZE, VM_WIMG_WCOMB);
	}
	if (cmd_db == NULL || !(cmd_db->magic[0] == 0xdb && cmd_db->magic[1] == 0x30 && cmd_db->magic[2] == 0x03 &&
	    cmd_db->magic[3] == 0x0c)) {
		return 0;
	}
	strlcpy(id, name, sizeof(id));
	for (unsigned s = 0; s < 8; s++) {
		const struct cmd_db_slave *slave = &cmd_db->slave[s];
		const struct cmd_db_entry *entry = (const struct cmd_db_entry *)(cmd_db->data + slave->header_offset);

		if (slave->id == 0 || slave->count > 512) {
			continue;
		}
		for (unsigned i = 0; i < slave->count; i++) {
			if (memcmp(entry[i].id, id, sizeof(id)) != 0) {
				continue;
			}
			if (levels != NULL) {
				*levels = cmd_db->data + slave->data_offset + entry[i].offset;
				*length = entry[i].length;
			}
			return entry[i].address;
		}
	}
	return 0;
}

/* What RPMh's command database has of regulators (for the log, when a rail is not found) */
static void
cmd_db_list(void)
{
	char line[280];
	int at = snprintf(line, sizeof(line), "apollo wifi: RPMh's regulators:");

	for (unsigned s = 0; s < 8 && cmd_db != NULL; s++) {
		const struct cmd_db_slave *slave = &cmd_db->slave[s];
		const struct cmd_db_entry *entry = (const struct cmd_db_entry *)(cmd_db->data + slave->header_offset);

		if (slave->id == 0 || slave->count > 512) {
			continue;
		}
		for (unsigned i = 0; i < slave->count; i++) {
			if (memcmp(entry[i].id, "smp", 3) != 0 && memcmp(entry[i].id, "ldo", 3) != 0 && memcmp(entry[i].id, "bob", 3) != 0) {
				continue;
			}
			if (at > (int)sizeof(line) - 12) {
				say("%s\n", line);
				at = snprintf(line, sizeof(line), "apollo wifi: ...");
			}
			at += snprintf(line + at, sizeof(line) - (size_t)at, " %.8s", entry[i].id);
		}
	}
	say("%s\n", line);
}

/*
 * One request of up to 16 writes, made at once (active), through TCS 1; true when RPMh says done. TCS 1 is shared with
 * apollo_rpmh_bcm_vote() (hakios.apollo_cpufreq.c): the whole request, its wait included, under apollo_rpmh_tcs1_lock.
 */
static bool
tcs_send(const uint32_t *address, const uint32_t *value, unsigned count, const char *what)
{
	vm_offset_t tcs = rsc + RSC_TCS(WIFI_TCS);
	uint32_t mask = (count >= 32 ? ~0u : (1u << count) - 1), done = 0;
	uint64_t began;

	lck_mtx_lock(&apollo_rpmh_tcs1_lock);
	if (rd(tcs + TCS_STATUS) == 0) {
		lck_mtx_unlock(&apollo_rpmh_tcs1_lock);
		say("apollo wifi: RPMh's TCS %u is busy (control %#x): %s not asked\n", WIFI_TCS, rd(tcs + TCS_CONTROL), what);
		return false;
	}
	wr(rsc + RSC_IRQ_CLEAR, 1u << WIFI_TCS);
	for (unsigned c = 0; c < count; c++) {
		wr(tcs + TCS_COMMAND(c), MSGID_WRITE_WAITED);
		wr(tcs + TCS_COMMAND(c) + 4, address[c]);
		wr(tcs + TCS_COMMAND(c) + 8, value[c]);
	}
	wr(tcs + TCS_WAIT_FOR_DONE, mask);
	wr(tcs + TCS_ENABLED, mask);
	wr(tcs + TCS_CONTROL, rd(tcs + TCS_CONTROL) & ~TCS_TRIGGER);
	wr(tcs + TCS_CONTROL, rd(tcs + TCS_CONTROL) | TCS_ACTIVE_MODE);
	wr(tcs + TCS_CONTROL, rd(tcs + TCS_CONTROL) | TCS_TRIGGER);
	/*
	 * Done: every command says completed (bit 16; Linux rpmh-rsc's CMD_STATUS_COMPL). The TCS's own bit in the IRQ
	 * status is not to be waited for: the boot loader has enabled that interrupt for TCS 0 only, and TCS 1's never
	 * sets (seen on the phone: every command 0x10101, the IRQ status bit still clear after 100 ms).
	 */
	began = now_us();
	while (now_us() - began < 100000) {
		bool all = true;

		for (unsigned c = 0; c < count && all; c++) {
			all = (rd(tcs + TCS_COMMAND(c) + 12) & (1u << 16)) != 0;
		}
		/* (and the TCS idle again, a little later: a command's status from the request before must not count) */
		all = all && rd(tcs + TCS_STATUS) != 0 && now_us() - began >= 100;
		if (all || (rd(rsc + RSC_IRQ_STATUS) & (1u << WIFI_TCS))) {
			done = 1;
			break;
		}
		sleep_us(50);
	}
	if (!done) {
		/* what each command says: [16] it was sent, [8] completed, [0] ... (Linux rpmh-rsc: CMD_STATUS_ISSUED/COMPL) */
		char line[260];
		int at = snprintf(line, sizeof(line), "apollo wifi: TCS %u status %#x control %#x; the commands:", WIFI_TCS,
		    rd(tcs + TCS_STATUS), rd(tcs + TCS_CONTROL));

		for (unsigned c = 0; c < count && at < (int)sizeof(line) - 24; c++) {
			at += snprintf(line + at, sizeof(line) - (size_t)at, " %#x=%#x:%#x", address[c], value[c],
			    rd(tcs + TCS_COMMAND(c) + 12));
		}
		say("%s\n", line);
	}
	wr(rsc + RSC_IRQ_CLEAR, 1u << WIFI_TCS);
	wr(tcs + TCS_CONTROL, rd(tcs + TCS_CONTROL) & ~(TCS_TRIGGER | TCS_ACTIVE_MODE));
	wr(tcs + TCS_ENABLED, 0);
	lck_mtx_unlock(&apollo_rpmh_tcs1_lock);
	say("apollo wifi: RPMh %s %s (%u writes, %llu us)\n", done ? "has done" : "HAS NOT SAID DONE to", what, count,
	    now_us() - began);
	return done != 0;
}

/* The same writes one at a time, when together they are not done: which of them RPMh does not take */
static bool
tcs_send_each(const uint32_t *address, const uint32_t *value, unsigned count, const char *what)
{
	unsigned failed = 0;

	if (tcs_send(address, value, count, what)) {
		return true;
	}
	for (unsigned c = 0; c < count; c++) {
		char one[48];

		snprintf(one, sizeof(one), "%#x = %u alone", address[c], value[c]);
		failed += !tcs_send(&address[c], &value[c], 1, one);
	}
	return failed == 0;
}

static const struct {
	const char *name;
	uint32_t mv;
	bool hpm, optional;
} rails[] = {
	/*
	 * The radio's rails in PWM (mode 7, pmic5 SMPS "fast"): the firmware crashed (watchdog) the moment it powered the
	 * radio, in mission and in calibration mode alike (25 October). smpc8's device-tree node starts it in low-power mode
	 * (qcom,init-mode 1, LPM, until 200 mA are asked for: qcom,mode-threshold-currents), and on Android other processors'
	 * votes lift it; here nobody else votes.
	 */
	{ "smpa6", 950, true },         /* vdd-wlan-aon, vdd-wlan-dig */
	{ "smpa4", 1800, false },       /* vdd-wlan-io */
	{ "smpa5", 1900, true },        /* vdd-wlan-rfa1 */
	{ "smpc8", 1350, true },        /* vdd-wlan-rfa2 */
	{ "smpf2", 950, false, true },  /* not a supply of cnss's (vdd-wlan-* are smpa6/smpa4/smpa5/smpc8): only qcom,cmd_db_name, the CPR's PMIC address; this phone's cmd-db has no smpf* (so no CPR voltage either: cnss2's cnss_update_cpr_info() needs it) */
	{ "ldoa5", 880, true },         /* PCIe's vreg-0p9, 73.5 mA */
	{ "ldoa9", 1200, true },        /* PCIe's vreg-1p8 (1.2 V on this phone), 16 mA */
};
#define RAILS (sizeof(rails) / sizeof(rails[0]))

static bool
power_on(void)
{
	uint32_t address[16], value[16], base[RAILS], cx, length = 0;
	const uint8_t *levels = NULL;
	unsigned n = 0, position = ~0u;

	for (unsigned r = 0; r < RAILS; r++) {
		base[r] = cmd_db_find(rails[r].name, NULL, NULL);
		if (base[r] != 0) {
			say("apollo wifi: cmd-db: %s found at %#x\n", rails[r].name, base[r]);
		} else {
			say("apollo wifi: cmd-db: %s NOT FOUND\n", rails[r].name);
		}
		if (base[r] == 0) {
			say("apollo wifi: RPMh has no %s%s\n", rails[r].name, rails[r].optional ? ": left out" : "");
			cmd_db_list();
			if (!rails[r].optional) {
				return false;
			}
		}
	}
	cx = cmd_db_find("cx.lvl", &levels, &length);
	if (cx == 0 || levels == NULL) {
		say("apollo wifi: RPMh has no cx.lvl\n");
		return false;
	}
	for (unsigned l = 0; l < length / 2 && l < 16; l++) {
		if ((uint32_t)(levels[2 * l] | levels[2 * l + 1] << 8) >= 0x100) {
			position = l;
			break;
		}
	}
	if (position == ~0u) {
		/* (a vote for position 0 would be CX off, not a level for the chip: nothing is asked) */
		say("apollo wifi: cx.lvl has no level of 0x100 or more among its first %u: no CX vote, no power\n", length / 2 < 16 ? length / 2 : 16);
		return false;
	}
	/* first the voltages and the modes, then the enables and the CX level */
	for (unsigned r = 0; r < RAILS; r++) {
		if (base[r] == 0) {
			continue;
		}
		address[n] = base[r] + VRM_VOLTAGE, value[n++] = rails[r].mv;
		if (rails[r].hpm) {
			address[n] = base[r] + VRM_MODE, value[n++] = LDO_MODE_HPM;
		}
	}
	if (!tcs_send_each(address, value, n, "the voltages of the Wi-Fi's and PCIe's rails")) {
		return false;
	}
	n = 0;
	for (unsigned r = 0; r < RAILS; r++) {
		if (base[r] != 0) {
			address[n] = base[r] + VRM_ENABLE, value[n++] = 1;
		}
	}
	address[n] = cx, value[n++] = position;
	if (!tcs_send_each(address, value, n, "the rails on, and CX at level 0x100")) {
		return false;
	}
	for (unsigned r = 0; r < RAILS; r++) {
		if (base[r] == 0) {
			continue;
		}
		say("apollo wifi: %s at %#x: %u mV%s\n", rails[r].name, base[r], rails[r].mv, rails[r].hpm ? ", high-power mode" : "");
	}
	say("apollo wifi: cx.lvl at %#x: position %u\n", cx, position);
	return true;
}

/* ---- clocks and the power domain (GCC, kona: gcc-kona.c) ------------------------------------------- */

#define GCC_PHYS                0x00100000UL
#define GCC_SIZE                0x00091000UL
#define GCC_VOTE                0x52008         /* APCS_GCC_BRANCH_ENA_VOTE_1 */
#define VOTE_PCIE0              0x3fu           /* slv_axi 0, mstr_axi 1, cfg_ahb 2, aux 3, pipe 4, slv_q2a 5 */
#define PCIE0_GDSCR             0x6b004         /* bit 0 collapse, bit 31 power on */
#define PCIE0_BCR               0x6b000
#define PCIE0_PHY_BCR           0x6c01c
#define PCIE0_CFG_AHB_CBCR      0x6b024         /* bit 31: off */
#define PCIE0_MSTR_AXI_CBCR     0x6b01c
#define PCIE0_SLV_AXI_CBCR      0x6b014
#define PCIE0_AUX_CBCR          0x6b028
#define PCIE0_AUX_RCG           0x6b038         /* cmd; cfg at +4 */
#define REFGEN_RCG              0x6f014
#define REFGEN_CBCR             0x6f02c
#define TBU_CBCR                0x9000c         /* gcc_aggre_noc_pcie_tbu_clk */
#define SF_TBU_CBCR             0x8d058         /* gcc_ddrss_pcie_sf_tbu_clk */
#define WIFI_CLKREF             0x8c004         /* gcc_pcie_wifi_clkref_en */
#define CLK_OFF                 (1u << 31)

static vm_offset_t gcc;

static bool
rcg_set(uint32_t rcg, uint32_t cfg, const char *what)
{
	wr(gcc + rcg + 4, (rd(gcc + rcg + 4) & ~0x371fu) | cfg);
	wr(gcc + rcg, rd(gcc + rcg) | 1);
	for (int i = 0; i < 500 && (rd(gcc + rcg) & 1); i++) {
		sleep_us(2);
	}
	if (rd(gcc + rcg) & 1) {
		say("apollo wifi: %s does not take its new rate (cmd %#x)\n", what, rd(gcc + rcg));
		return false;
	}
	return true;
}

static bool
branch_on(uint32_t cbcr, const char *what)
{
	wr(gcc + cbcr, rd(gcc + cbcr) | 1);
	for (int i = 0; i < 200 && (rd(gcc + cbcr) & CLK_OFF); i++) {
		sleep_us(5);
	}
	if (rd(gcc + cbcr) & CLK_OFF) {
		say("apollo wifi: %s stays off (%#x)\n", what, rd(gcc + cbcr));
		return false;
	}
	return true;
}

static bool
clocks_on(void)
{
	/* the power domain first: what is in it is unclocked and unreachable until it is on */
	wr(gcc + PCIE0_GDSCR, rd(gcc + PCIE0_GDSCR) & ~1u);
	for (int i = 0; i < 100 && !(rd(gcc + PCIE0_GDSCR) & (1u << 31)); i++) {
		sleep_us(10);
	}
	if (!(rd(gcc + PCIE0_GDSCR) & (1u << 31))) {
		say("apollo wifi: the pcie_0 power domain does not come on (GDSCR %#x)\n", rd(gcc + PCIE0_GDSCR));
		return false;
	}
	if (!rcg_set(PCIE0_AUX_RCG, 0 << 8 | 1, "pcie_0 aux (XO, 19.2 MHz)") ||
	    !rcg_set(REFGEN_RCG, 1 << 8 | 11, "the PHY's refgen (GPLL0 / 6, 100 MHz)")) {
		return false;
	}
	apollo_gcc_vote(gcc + GCC_VOTE, VOTE_PCIE0);    /* (shared with the touch, haptic and amplifier drivers: under its lock) */
	if (!branch_on(TBU_CBCR, "the aggregate NoC's PCIe TBU clock") || !branch_on(SF_TBU_CBCR, "DDRSS's PCIe TBU clock") ||
	    !branch_on(REFGEN_CBCR, "the PHY's refgen clock") || !branch_on(WIFI_CLKREF, "the Wi-Fi's clock reference")) {
		return false;
	}
	for (int i = 0; i < 200; i++) {
		if (!((rd(gcc + PCIE0_CFG_AHB_CBCR) | rd(gcc + PCIE0_MSTR_AXI_CBCR) | rd(gcc + PCIE0_AUX_CBCR)) & CLK_OFF)) {
			say("apollo wifi: the pcie_0 power domain and clocks are on (GDSCR %#x; cfg %#x, mstr %#x, slv %#x, aux %#x)\n",
			    rd(gcc + PCIE0_GDSCR), rd(gcc + PCIE0_CFG_AHB_CBCR), rd(gcc + PCIE0_MSTR_AXI_CBCR),
			    rd(gcc + PCIE0_SLV_AXI_CBCR), rd(gcc + PCIE0_AUX_CBCR));
			return true;
		}
		sleep_us(5);
	}
	say("apollo wifi: the pcie_0 clocks stay off: cfg %#x, mstr %#x, aux %#x\n", rd(gcc + PCIE0_CFG_AHB_CBCR),
	    rd(gcc + PCIE0_MSTR_AXI_CBCR), rd(gcc + PCIE0_AUX_CBCR));
	return false;
}

static void
bcr_pulse(uint32_t bcr)
{
	wr(gcc + bcr, rd(gcc + bcr) | 1);
	sleep_ms(1);
	wr(gcc + bcr, rd(gcc + bcr) & ~1u);
	sleep_ms(1);
}

/* ---- the PHY (QMP, gen3 x1): the tree's qcom,phy-sequence --------------------------------------------- */

#define PHY_PHYS                0x01c06000UL
#define PHY_SIZE                0x1000UL
#define PCS_STATUS1             0x814           /* bit 6: PHYSTATUS, clear when ready */

static vm_offset_t phy;

static const uint16_t phy_sequence[][2] = {     /* offset, value: /soc/qcom,pcie@1c00000 qcom,phy-sequence (no delays) */
	{ 0x840, 0x03 }, { 0x094, 0x08 }, { 0x154, 0x34 }, { 0x16c, 0x08 }, { 0x058, 0x0f }, { 0x0a4, 0x42 }, { 0x110, 0x24 },
	{ 0x11c, 0x03 }, { 0x118, 0xb4 }, { 0x10c, 0x02 }, { 0x1bc, 0x11 }, { 0x0bc, 0x82 }, { 0x0d4, 0x03 }, { 0x0d0, 0x55 },
	{ 0x0cc, 0x55 }, { 0x0b0, 0x1a }, { 0x0ac, 0x0a }, { 0x0c4, 0x68 }, { 0x0e0, 0x02 }, { 0x0dc, 0xaa }, { 0x0d8, 0xab },
	{ 0x0b8, 0x34 }, { 0x0b4, 0x14 }, { 0x158, 0x01 }, { 0x074, 0x06 }, { 0x07c, 0x16 }, { 0x084, 0x36 }, { 0x078, 0x06 },
	{ 0x080, 0x16 }, { 0x088, 0x36 }, { 0x1b0, 0x1e }, { 0x1ac, 0xca }, { 0x1b8, 0x18 }, { 0x1b4, 0xa2 }, { 0x050, 0x07 },
	{ 0x010, 0x01 }, { 0x01c, 0x31 }, { 0x020, 0x01 }, { 0x024, 0xde }, { 0x028, 0x07 }, { 0x030, 0x4c }, { 0x034, 0x06 },
	{ 0x29c, 0x12 }, { 0x284, 0x35 }, { 0x23c, 0x11 }, { 0x51c, 0x03 }, { 0x518, 0x1c }, { 0x524, 0x1e }, { 0x4e8, 0x00 },
	{ 0x4ec, 0x0e }, { 0x4f0, 0x4a }, { 0x4f4, 0x0f }, { 0x5b4, 0x04 }, { 0x434, 0x7f }, { 0x444, 0x70 }, { 0x510, 0x17 },
	{ 0x4d4, 0x04 }, { 0x4d8, 0x07 }, { 0x598, 0xd4 }, { 0x59c, 0x54 }, { 0x5a0, 0xdb }, { 0x5a4, 0x3b }, { 0x5a8, 0x31 },
	{ 0x584, 0x24 }, { 0x588, 0xe4 }, { 0x58c, 0xec }, { 0x590, 0x3b }, { 0x594, 0x36 }, { 0x570, 0x3f }, { 0x574, 0x3f },
	{ 0x578, 0xff }, { 0x57c, 0x7f }, { 0x580, 0x14 }, { 0x4fc, 0x00 }, { 0x4f8, 0xc0 }, { 0x460, 0x30 }, { 0x464, 0x00 },
	{ 0x5bc, 0x0c }, { 0x4dc, 0x1b }, { 0x408, 0x0c }, { 0x414, 0x03 }, { 0x5b8, 0x30 }, { 0x9a4, 0x01 }, { 0xc90, 0x00 },
	{ 0xc40, 0x01 }, { 0xc48, 0x01 }, { 0xc50, 0x00 }, { 0xcb4, 0x33 }, { 0xcbc, 0x00 }, { 0xce0, 0x58 }, { 0xca4, 0x0f },
	{ 0x048, 0x90 }, { 0xc1c, 0xc1 }, { 0x988, 0x77 }, { 0x998, 0x0b }, { 0x8dc, 0x0d }, { 0x9ec, 0x12 }, { 0x800, 0x00 },
	{ 0x844, 0x03 },
};

static bool
phy_on(void)
{
	bcr_pulse(PCIE0_PHY_BCR);
	for (unsigned i = 0; i < sizeof(phy_sequence) / sizeof(phy_sequence[0]); i++) {
		wr(phy + phy_sequence[i][0], phy_sequence[i][1]);
	}
	sleep_ms(1);
	for (int i = 0; i < 50; i++) {
		if (!(rd(phy + PCS_STATUS1) & (1u << 6))) {
			say("apollo wifi: the PHY is ready (%u writes; PCS status %#x)\n",
			    (unsigned)(sizeof(phy_sequence) / sizeof(phy_sequence[0])), rd(phy + PCS_STATUS1));
			return true;
		}
		sleep_us(200);
	}
	say("apollo wifi: the PHY is not ready (PCS status %#x)\n", rd(phy + PCS_STATUS1));
	return false;
}

/* ---- the controller (DesignWare, with Qualcomm's PARF around it) ------------------------------------ */

#define PARF_PHYS               0x01c00000UL
#define PARF_SIZE               0x3000UL
#define PARF_SYS_CTRL           0x000
#define PARF_PM_CTRL            0x020
#define PARF_PHY_CTRL           0x040
#define PARF_MHI_CLOCK_RESET    0x174
#define PARF_AXI_MSTR_WR_HALT   0x1a8
#define PARF_LTSSM              0x1b0
#define PARF_DBI_BASE           0x350           /* and 0x354, the high word */
#define PARF_SLV_ADDR_SPACE     0x358           /* and 0x35c */
#define PARF_ATU_BASE           0x634           /* and 0x638 */
#define PARF_DEVICE_TYPE        0x1000
/*
 * The controller tags each requester's memory traffic with a stream from this table, a hash (crc8, polynomial 7, of the
 * big-endian bus/device/function) to "bdf << 16 | (stream - qcom,smmu-sid-base) << 8 | next" (Linux's pcie-qcom.c,
 * qcom_pcie_config_sid_1_9_0). The tree's iommu-map: 0x000 to 0x1c00 (the root port), 0x100 (the QCA6390) to 0x1c01.
 * Left empty, the chip's reads went out as 0x1c00, not through W2b's context, and its boot ROM gave up (error 0xef128100).
 */
#define PARF_BDF_TO_SID(n)      (0x2000 + 4 * (n))
#define BDF_TO_SID_ENTRIES      256
#define WIFI_BDF                0x100
#define WIFI_BDF_HASH           0x15            /* crc8 of { 0x01, 0x00 } */
#define DEVICE_TYPE_RC          0x4

#define DBI_PHYS                0x60000000UL    /* dm_core */
#define ATU_PHYS                0x60001000UL    /* iatu */
#define CONF_PHYS               0x60100000UL    /* conf: the device's configuration space, through iATU region 0 */
#define SLV_ADDR_SPACE_SIZE     0x4000000u      /* qcom,slv-addr-space-size */
#define DBI_PORT_DEBUG1         0x72c           /* bit 4 link up, bit 29 link in training */
#define DBI_LINK_CTRL_STATUS    0x080           /* the root port's PCIe capability (at 0x70): link status in the high half */
#define ATU_CTRL1               0x00
#define ATU_CTRL2               0x04
#define ATU_LOWER_BASE          0x08
#define ATU_UPPER_BASE          0x0c
#define ATU_LIMIT               0x10
#define ATU_LOWER_TARGET        0x14
#define ATU_UPPER_TARGET        0x18
#define ATU_TYPE_CFG0           0x4
#define ATU_ENABLE              (1u << 31)

static vm_offset_t parf, dbi, atu, conf;

/*
 * Where the controller's own registers (DBI) and its address translation (iATU) answer. On this generation the
 * DBI base is an offset in PCIe0's window (0x60000000), as Linux writes it for ops 2_3_3 (PARF_DBI_BASE_ADDR = 0):
 * writing the absolute address put DBI outside the window, and it read all ones (seen on the phone). Tried in that
 * order, and the other way if DBI does not answer; the same for the iATU (offset 0x1000, or absolute).
 */
static bool
dbi_answers(void)
{
	return rd(dbi) != 0xffffffffu;
}

static bool
atu_answers(void)
{
	return rd(atu + ATU_LIMIT) != 0xffffffffu;
}

static void
controller_setup(void)
{
	say("apollo wifi: PARF as found: DBI base %#x:%#x, iATU base %#x:%#x, address space %#x, DBI reads %#x\n",
	    rd(parf + PARF_DBI_BASE + 4), rd(parf + PARF_DBI_BASE), rd(parf + PARF_ATU_BASE + 4), rd(parf + PARF_ATU_BASE),
	    rd(parf + PARF_SLV_ADDR_SPACE), rd(dbi));
	wr(parf + PARF_DEVICE_TYPE, DEVICE_TYPE_RC);
	wr(parf + PARF_PHY_CTRL, rd(parf + PARF_PHY_CTRL) & ~1u);
	wr(parf + PARF_SLV_ADDR_SPACE, SLV_ADDR_SPACE_SIZE);
	wr(parf + PARF_SLV_ADDR_SPACE + 4, 0);
	wr(parf + PARF_DBI_BASE, 0);
	wr(parf + PARF_DBI_BASE + 4, 0);
	if (!dbi_answers()) {
		wr(parf + PARF_DBI_BASE, (uint32_t)DBI_PHYS);
	}
	wr(parf + PARF_ATU_BASE, (uint32_t)(ATU_PHYS - DBI_PHYS));
	wr(parf + PARF_ATU_BASE + 4, 0);
	if (!atu_answers()) {
		wr(parf + PARF_ATU_BASE, (uint32_t)ATU_PHYS);
	}
	say("apollo wifi: DBI base %#x (DBI reads %#x), iATU base %#x (its region 0 limit reads %#x)\n", rd(parf + PARF_DBI_BASE),
	    rd(dbi), rd(parf + PARF_ATU_BASE), rd(atu + ATU_LIMIT));
	wr(parf + PARF_SYS_CTRL, rd(parf + PARF_SYS_CTRL) & ~(1u << 29));
	wr(parf + PARF_MHI_CLOCK_RESET, rd(parf + PARF_MHI_CLOCK_RESET) | 1u << 4);
	wr(parf + PARF_PM_CTRL, rd(parf + PARF_PM_CTRL) & ~(1u << 5));
	wr(parf + PARF_AXI_MSTR_WR_HALT, rd(parf + PARF_AXI_MSTR_WR_HALT) | 1u << 31);
}

static bool
link_up(void)
{
	uint32_t debug = rd(dbi + DBI_PORT_DEBUG1);

	return (debug & (1u << 4)) && !(debug & (1u << 29));
}

static bool
link_on(void)
{
	uint32_t id, status;

	say("apollo wifi: the root port is %04x:%04x; PERST# goes high in 100 ms\n", rd(dbi) & 0xffff, rd(dbi) >> 16);
	/* the root port: bus 0, the device behind it on bus 1, memory and bus mastering on */
	wr(dbi + 0x18, 0x00010100);
	wr(dbi + 0x04, rd(dbi + 0x04) | 0x6);
	sleep_ms(100);
	pin_set(PIN_PERST, true);
	sleep_ms(1);
	wr(parf + PARF_LTSSM, rd(parf + PARF_LTSSM) | 1u << 8);
	for (int i = 0; i < 100 && !link_up(); i++) {
		sleep_ms(10);
	}
	status = rd(dbi + DBI_LINK_CTRL_STATUS) >> 16;
	if (!link_up()) {
		say("apollo wifi: THE LINK DID NOT COME UP (port debug %#x, LTSSM %#x, link status %#x)\n",
		    rd(dbi + DBI_PORT_DEBUG1), rd(parf + PARF_LTSSM), status);
		return false;
	}
	say("apollo wifi: the link is up: gen %u, x%u (port debug %#x)\n", status & 0xf, (status >> 4) & 0x3f,
	    rd(dbi + DBI_PORT_DEBUG1));
	/* iATU region 0, outbound: configuration requests of type 0 to bus 1, device 0 */
	wr(atu + ATU_LOWER_BASE, (uint32_t)CONF_PHYS);
	wr(atu + ATU_UPPER_BASE, 0);
	wr(atu + ATU_LIMIT, (uint32_t)CONF_PHYS + 0xfff);
	wr(atu + ATU_LOWER_TARGET, 1u << 24);
	wr(atu + ATU_UPPER_TARGET, 0);
	wr(atu + ATU_CTRL1, ATU_TYPE_CFG0);
	wr(atu + ATU_CTRL2, ATU_ENABLE);
	for (int i = 0; i < 10 && !(rd(atu + ATU_CTRL2) & ATU_ENABLE); i++) {
		sleep_us(10);
	}
	id = rd(conf);
	say("apollo wifi: the device on the link is %04x:%04x%s (class %#x)\n", id & 0xffff, id >> 16,
	    id == 0x110117cb ? ": THE QCA6390" : "", rd(conf + 0x08) >> 8);
	return id == 0x110117cb;
}

/* ---- step W2a: the chip's BAR, and its MHI registers read (no DMA yet) -------------------------------- */

/*
 * The tree's ranges: PCIe0's memory window is 0x60300000, 0x3d00000 bytes, the same addresses on both sides. BAR0 of
 * the QCA6390 (64-bit: ath11k's pci.c maps it whole) goes at its start, aligned to its size; iATU region 1 sends that
 * window's accesses out as memory requests; the root port forwards it (memory base/limit); the chip decodes it.
 * Then the MHI block at the BAR's start (Linux drivers/bus/mhi/common.h) and its BHI block say what the chip waits for.
 */
#define MEM_WINDOW              0x60300000UL
#define MEM_WINDOW_SIZE         0x03d00000UL
#define ATU_REGION(n)           (0x200u * (n))
#define ATU_TYPE_MEM            0x0
#define MHI_VER                 0x08
#define MHI_CFG                 0x10            /* [7:0] channels? [23:16] event rings, [31:24] hw event rings (MHICFG) */
#define MHI_BHI_OFF             0x28
#define MHI_BHIE_OFF            0x2c
#define MHI_CTRL                0x38
#define MHI_STATUS              0x48            /* [15:8] MHI state, bit 2 system error, bit 0 ready */
#define BHI_EXECENV             0x28
#define BHI_STATUS              0x2c
#define BHI_ERRCODE             0x30

static vm_offset_t bar;
static vm_offset_t bar_mapped;          /* what of BAR0 is mapped (the window needs 1 MB) */

static const char *
execution_environment(uint32_t ee)
{
	static const char *const names[] = { "PBL (the boot ROM: waiting for its first image)", "SBL", "AMSS (the firmware runs)",
		                             "RDDM (a crash dump)", "WFW", "PTHRU", "EDL", "FP", "UEFI" };

	return ee < sizeof(names) / sizeof(names[0]) ? names[ee] : "unknown";
}

static bool
mhi_look(void)
{
	uint32_t size, low, mhi_ver, mhi_cfg, bhi, bhie, status, ee;

	/* BAR0's size: all ones written, what reads back */
	low = rd(conf + 0x10);
	wr(conf + 0x10, 0xffffffff);
	size = ~(rd(conf + 0x10) & ~0xfu) + 1;
	wr(conf + 0x10, low);
	if (size == 0 || size > MEM_WINDOW_SIZE || (low & 0x1)) {
		say("apollo wifi: W2a: BAR0 is not a memory BAR of a size this takes (reads %#x, size %#x)\n", low, size);
		return false;
	}
	uint32_t at = (uint32_t)((MEM_WINDOW + size - 1) & ~(uint64_t)(size - 1));
	wr(conf + 0x10, at | (low & 0xf));
	if ((low & 0x6) == 0x4) {
		wr(conf + 0x14, 0);             /* the high word of a 64-bit BAR */
	}
	/* iATU region 1: the memory window, out as memory requests to the same addresses */
	wr(atu + ATU_REGION(1) + ATU_LOWER_BASE, (uint32_t)MEM_WINDOW);
	wr(atu + ATU_REGION(1) + ATU_UPPER_BASE, 0);
	wr(atu + ATU_REGION(1) + ATU_LIMIT, (uint32_t)(MEM_WINDOW + MEM_WINDOW_SIZE - 1));
	wr(atu + ATU_REGION(1) + ATU_LOWER_TARGET, (uint32_t)MEM_WINDOW);
	wr(atu + ATU_REGION(1) + ATU_UPPER_TARGET, 0);
	wr(atu + ATU_REGION(1) + ATU_CTRL1, ATU_TYPE_MEM);
	wr(atu + ATU_REGION(1) + ATU_CTRL2, ATU_ENABLE);
	/* the root port forwards the window (memory base and limit, in 1 MB units), the chip decodes memory, masters */
	wr(dbi + 0x20, (uint32_t)(((MEM_WINDOW + MEM_WINDOW_SIZE - 1) >> 16) & 0xfff0) << 16 | (uint32_t)((MEM_WINDOW >> 16) & 0xfff0));
	wr(conf + 0x04, rd(conf + 0x04) | 0x6);
	bar_mapped = size < 0x100000 ? size : 0x100000;
	bar = ml_io_map(at, bar_mapped);
	if (bar == 0) {
		say("apollo wifi: W2a: cannot map BAR0\n");
		return false;
	}
	mhi_ver = rd(bar + MHI_VER);
	mhi_cfg = rd(bar + MHI_CFG);
	bhi = rd(bar + MHI_BHI_OFF);
	bhie = rd(bar + MHI_BHIE_OFF);
	status = rd(bar + MHI_STATUS);
	say("apollo wifi: W2a: BAR0 at %#x, %u KB (%s); MHI version %#x, config %#x, BHI at +%#x, BHIe at +%#x; MHI state %u%s%s\n",
	    at, size / 1024, (low & 0x6) == 0x4 ? "64-bit" : "32-bit", mhi_ver, mhi_cfg, bhi, bhie, status >> 8 & 0xff,
	    (status & 1) ? ", ready" : "", (status & 4) ? ", SYSTEM ERROR" : "");
	if (bhi == 0 || bhi == 0xffffffffu || bhi >= (size < 0x100000 ? size : 0x100000)) {
		say("apollo wifi: W2a: no BHI block where MHI says (%#x)\n", bhi);
		return false;
	}
	ee = rd(bar + bhi + BHI_EXECENV);
	say("apollo wifi: W2a: the chip's execution environment: %u, %s; BHI status %#x, error %#x\n", ee, execution_environment(ee),
	    rd(bar + bhi + BHI_STATUS), rd(bar + bhi + BHI_ERRCODE));
	return true;
}

/* ---- step W2b: the first stage of the firmware, handed over BHI ---------------------------------------- */

/*
 * The chip's boot ROM (PBL) waits for a first image over BHI (Linux drivers/bus/mhi/host/boot.c, mhi_fw_load_bhi): the
 * image's address as the chip sees it, its size, and a session id in the transmit doorbell. The chip reads it by DMA,
 * through the application processors' SMMU as stream 0x1c01 (/soc/qcom,pcie@1c00000 iommu-map): a context of ours, page
 * tables of ours, the first 0x80000 bytes of amss.bin (ath11k: sbl_size) at 0xa0000000 on, contiguous there though its
 * pages are not (qcom,iommu-dma-addr-pool of cnss). The method is hakios.apollo_sound.c's, for stream 0x1801.
 */
#define SMMU_PHYS               0x15000000UL
#define SMMU_SIZE               0x100000UL
#define SMMU_SCR0               0x000
#define SMMU_IDR0               0x020
#define SMMU_IDR1               0x024
#define SMMU_SGFSR              0x048
#define SMMU_SGFSYNR1           0x054
#define SMMU_SMR(n)             (0x800 + 4 * (n))       /* bit 31 valid, [30:16] mask, [14:0] stream */
#define SMMU_S2CR(n)            (0xc00 + 4 * (n))       /* [17:16] 0 translate 1 bypass 2 fault, [7:0] context */
#define SMMU_S2CR_FAULT         (2u << 16)
#define SMMU_CBAR(n)            (4 * (n))
#define SMMU_CBAR_STAGE_1       (1u << 16 | 3u << 8 | 0xfu << 12)
#define SMMU_CBA2R(n)           (0x800 + 4 * (n))
#define SMMU_CBA2R_64_BIT       1
#define SMMU_CB_SCTLR           0x00
#define SMMU_CB_TCR2            0x10
#define SMMU_CB_TTBR0           0x20
#define SMMU_CB_TTBR1           0x28
#define SMMU_CB_TCR             0x30
#define SMMU_CB_MAIR0           0x38
#define SMMU_CB_MAIR1           0x3c
#define SMMU_CB_FSR             0x58
#define SMMU_CB_FAR             0x60
#define SMMU_CB_FSYNR0          0x68
#define SMMU_CB_TLBIASID        0x610
#define SMMU_CB_TLBSYNC         0x7f0
#define SMMU_CB_TLBSTATUS       0x7f4
#define SMMU_SCTLR_ON           (1u << 0 | 1u << 1 | 1u << 2 | 1u << 5 | 1u << 8 | 1u << 12)
#define SMMU_TCR_39_BITS        (25u | 1u << 8 | 1u << 10 | 3u << 12 | 1u << 23)
#define SMMU_TCR2               (1u | 1u << 4 | 7u << 15)
#define SMMU_MAIR0              0xf404ff44
#define SMMU_FSR_ALL            0xc00001fe
#define PTE_ADDRESS             0x0000fffffffff000ull
#define WIFI_PTE                (3ull | 1ull << 6 | 2ull << 8 | 1ull << 10 | 1ull << 11)       /* as hakios.apollo_sound.c's */

#define WIFI_STREAM             0x1c01
#define WIFI_ADDRESSES          0xa0000000ull
#define FIRST_STAGE             0x80000u
#define WIFI_POOL_SIZE          0x10000000ull   /* qcom,iommu-dma-addr-pool of cnss: 0xa0000000, size 0x10000000 */
#define WIFI_L3_TABLES          128             /* the whole pool: 128 third-level tables of 2 MB each, added as mappings need them (apollo_wifi_dma_map: >= 96 MB for ath11k) */

#define BHI_IMGADDR_LOW         0x08
#define BHI_IMGADDR_HIGH        0x0c
#define BHI_IMGSIZE             0x10
#define BHI_IMGTXDB             0x18
#define BHI_INTVEC              0x20
#define BHI_ERRDBG1             0x34
#define BHI_ERRDBG2             0x38
#define BHI_ERRDBG3             0x3c
#define BHI_SERIALNU            0x40
#define BHI_STATUS_SUCCESS      2u
#define BHI_STATUS_ERROR        3u

static struct {
	volatile bool on;               /* the context or the match register may be set: apollo_wifi_quiesce() takes them down */
	uint32_t page, pages, match, context, old_smr, old_s2cr;
	vm_offset_t base;               /* the SMMU's megabyte */
	vm_offset_t tables;             /* three of 4K: the root, one of the second level, one of the third */
	vm_offset_t buffer;             /* FIRST_STAGE bytes of the image, never freed (the chip may read it until PERST# is down) */
	uint64_t next;                  /* W2c: the next free address of the Wi-Fi's (wsmmu_map() hands them out from FIRST_STAGE on) */
	vm_offset_t l3[WIFI_L3_TABLES]; /* W2c: the third-level tables, one per 2 MB from WIFI_ADDRESSES on; [0] is in tables, 0: none yet */
} wsmmu;
static vm_offset_t bhi_base;

static vm_offset_t
wsmmu_context(uint32_t n)
{
	return wsmmu.base + (wsmmu.pages + n) * wsmmu.page;
}

/* A free match register and a free context (with apollo_smmu_lock held) */
/*
 * W2b's first runs reset the whole phone without a word (25 October): a pause of 3 s after each step's announcement, so
 * that the loader has the line before the next step is taken (boot-arg apollo_wifi_pause=MS, 0: none).
 */
static uint32_t w2b_pause_ms = 3000;

static void
w2b_hold(const char *next)
{
	if (w2b_pause_ms) {
		say("apollo wifi: W2b: next: %s (in %u ms)\n", next, w2b_pause_ms);
		delay_for_interval(w2b_pause_ms, 1000 * NSEC_PER_USEC);
	}
}

static bool
wsmmu_find(void)
{
	bool used[256] = { false };
	uint32_t scr0 = rd(wsmmu.base + SMMU_SCR0), idr0 = rd(wsmmu.base + SMMU_IDR0), idr1 = rd(wsmmu.base + SMMU_IDR1);
	uint32_t matches = idr0 & 0xff, contexts = idr1 & 0xff;

	if (scr0 & 1u << 3) {
		say("apollo wifi: W2b: this SMMU numbers its streams with 16 bits, which nothing here is written for\n");
		return false;
	}
	wsmmu.page = idr1 >> 31 ? 0x10000 : 0x1000;
	wsmmu.pages = 2u << (idr1 >> 28 & 7);
	if (2 * wsmmu.pages * wsmmu.page != SMMU_SIZE) {
		say("apollo wifi: W2b: the SMMU says it has 2 x %u pages of %#x bytes, not a megabyte\n", wsmmu.pages, wsmmu.page);
		return false;
	}
	wsmmu.match = ~0u;
	for (uint32_t n = 0; n < matches && SMMU_SMR(n) < 0xc00; n++) {
		uint32_t smr = rd(wsmmu.base + SMMU_SMR(n)), s2cr = rd(wsmmu.base + SMMU_S2CR(n));
		uint32_t stream = smr & 0x7fff, mask = smr >> 16 & 0x7fff;

		if (smr >> 31) {
			used[s2cr & 0xff] = true;
			if (((stream ^ WIFI_STREAM) & ~mask) == 0) {
				say("apollo wifi: W2b: match register %u covers the Wi-Fi's stream already: stream %#x mask %#x, s2cr %#x: it is "
				    "somebody's, and is left alone\n", n, stream, mask, s2cr);
				return false;
			}
		} else if (wsmmu.match == ~0u) {
			wsmmu.match = n;
			wsmmu.old_smr = smr;
			wsmmu.old_s2cr = s2cr;
		}
	}
	/* a context nobody's match register uses, and that is off (one switched on without a stream is somebody's) */
	for (wsmmu.context = 0; wsmmu.context < contexts; wsmmu.context++) {
		if (!used[wsmmu.context] && !(rd(wsmmu_context(wsmmu.context) + SMMU_CB_SCTLR) & 1)) {
			break;
		}
	}
	if (wsmmu.match == ~0u || wsmmu.context >= contexts) {
		say("apollo wifi: W2b: the SMMU has no match register or no context left\n");
		return false;
	}
	say("apollo wifi: W2b: free in the SMMU: match register %u (smr %#x, s2cr %#x) and context %u, at %#lx\n", wsmmu.match,
	    wsmmu.old_smr, wsmmu.old_s2cr, wsmmu.context, SMMU_PHYS + (wsmmu.pages + wsmmu.context) * wsmmu.page);
	w2b_hold("the context's own registers are written");
	return true;
}

/* The SMMU's context off and the stream's match register invalid, S2CR to fault. No lock, no sleep, no log: a panic's way too. */
static void
wsmmu_off(void)
{
	if (!wsmmu.on) {
		return;
	}
	wsmmu.on = false;
	wr(wsmmu.base + SMMU_SMR(wsmmu.match), 0);
	wr(wsmmu.base + SMMU_S2CR(wsmmu.match), SMMU_S2CR_FAULT);
	wr(wsmmu_context(wsmmu.context) + SMMU_CB_SCTLR, 0);
}

/* The context's fault status and the SMMU's, told once */
static void
wsmmu_faults(bool *told)
{
	uint32_t status, global;

	if (!wsmmu.on || *told) {
		return;
	}
	status = rd(wsmmu_context(wsmmu.context) + SMMU_CB_FSR);
	global = rd(wsmmu.base + SMMU_SGFSR);
	if ((status & SMMU_FSR_ALL) == 0 && global == 0) {
		return;
	}
	*told = true;
	say("apollo wifi: W2b: THE SMMU HAS REFUSED A REQUEST: context %u: status %#x, address %#llx, syndrome %#x; the SMMU as a "
	    "whole: status %#x, stream %#x\n", wsmmu.context, status,
	    (uint64_t)rd(wsmmu_context(wsmmu.context) + SMMU_CB_FAR + 4) << 32 | rd(wsmmu_context(wsmmu.context) + SMMU_CB_FAR),
	    rd(wsmmu_context(wsmmu.context) + SMMU_CB_FSYNR0), global, rd(wsmmu.base + SMMU_SGFSYNR1));
}

/* FIRST_STAGE bytes of buffer behind the Wi-Fi's addresses from WIFI_ADDRESSES on, through a context of ours */
static bool
wsmmu_on(void)
{
	uint64_t *root, *second, *third, table_root;
	uint32_t smr, s2cr, sees = 1u << 31 | WIFI_STREAM, context;
	vm_offset_t cb;
	bool ok = false;

	if (kmem_alloc(kernel_map, &wsmmu.tables, 3 * 4096, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		say("apollo wifi: W2b: no memory for the page tables\n");
		return false;
	}
	lck_mtx_lock(&apollo_smmu_lock);
	if (!wsmmu_find()) {
		goto out;
	}
	smr = SMMU_SMR(wsmmu.match);
	s2cr = SMMU_S2CR(wsmmu.match);
	context = wsmmu.context;
	cb = wsmmu_context(context);
	root = (uint64_t *)wsmmu.tables;
	second = (uint64_t *)(wsmmu.tables + 4096);
	third = (uint64_t *)(wsmmu.tables + 8192);
	wsmmu.next = WIFI_ADDRESSES + FIRST_STAGE;
	wsmmu.l3[0] = wsmmu.tables + 8192;
	root[WIFI_ADDRESSES >> 30 & 0x1ff] = kvtophys((vm_offset_t)second) | 3;
	second[WIFI_ADDRESSES >> 21 & 0x1ff] = kvtophys((vm_offset_t)third) | 3;
	for (unsigned page = 0; page < FIRST_STAGE / 4096; page++) {
		third[(WIFI_ADDRESSES >> 12 & 0x1ff) + page] = kvtophys(wsmmu.buffer + 4096 * page) | WIFI_PTE;
	}
	for (vm_offset_t line = wsmmu.tables; line < wsmmu.tables + 3 * 4096; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__asm__ volatile ("dsb sy");
	table_root = kvtophys(wsmmu.tables) | (uint64_t)context << 48;
	say("apollo wifi: W2b: THE SMMU IS ABOUT TO BE WRITTEN: context %u gets page tables of ours (root %#llx) for the %u KB of the first "
	    "stage at %#llx, and match register %u sends stream %#x through it\n", context, table_root & PTE_ADDRESS, FIRST_STAGE / 1024,
	    WIFI_ADDRESSES, wsmmu.match, WIFI_STREAM);
	w2b_hold("the context's registers, then the global match registers (s2cr, smr: the hypervisor's to grant)");
	wr(cb + SMMU_CB_SCTLR, 0);
	wr(wsmmu.base + wsmmu.page + SMMU_CBA2R(context), SMMU_CBA2R_64_BIT);
	wr(wsmmu.base + wsmmu.page + SMMU_CBAR(context), SMMU_CBAR_STAGE_1);
	wr(cb + SMMU_CB_TCR2, SMMU_TCR2);
	wr(cb + SMMU_CB_TCR, SMMU_TCR_39_BITS);
	wr(cb + SMMU_CB_TTBR0, (uint32_t)table_root);
	wr(cb + SMMU_CB_TTBR0 + 4, (uint32_t)(table_root >> 32));
	wr(cb + SMMU_CB_TTBR1, 0);
	wr(cb + SMMU_CB_TTBR1 + 4, 0);
	wr(cb + SMMU_CB_MAIR0, SMMU_MAIR0);
	wr(cb + SMMU_CB_MAIR1, 0);
	wr(cb + SMMU_CB_FSR, SMMU_FSR_ALL);
	wr(cb + SMMU_CB_TLBIASID, context);
	wr(cb + SMMU_CB_TLBSYNC, 1);
	for (unsigned ms = 0; rd(cb + SMMU_CB_TLBSTATUS) & 1; ms++) {
		if (ms == 100) {
			say("apollo wifi: W2b: the SMMU does not finish forgetting old translations\n");
			goto out;
		}
		sleep_ms(1);
	}
	wr(cb + SMMU_CB_SCTLR, SMMU_SCTLR_ON);
	wsmmu.on = true;
	wr(wsmmu.base + s2cr, context);
	wr(wsmmu.base + smr, sees);
	if (rd(wsmmu.base + smr) != sees || (rd(wsmmu.base + s2cr) & 0x300ff) != context || !(rd(cb + SMMU_CB_SCTLR) & 1) ||
	    rd(cb + SMMU_CB_TTBR0) != (uint32_t)table_root) {
		say("apollo wifi: W2b: THE SMMU DID NOT TAKE WHAT WAS WRITTEN: stream %#x is not going through context %u (smr %#x, s2cr "
		    "%#x, sctlr %#x)\n", WIFI_STREAM, context, rd(wsmmu.base + smr), rd(wsmmu.base + s2cr), rd(cb + SMMU_CB_SCTLR));
		goto out;
	}
	say("apollo wifi: W2b: THE WI-FI STREAM %#x GOES THROUGH CONTEXT %u of the SMMU, with our page tables\n", WIFI_STREAM, context);
	w2b_hold("the chip may master the bus (its PCI command register)");
	ok = true;
out:
	if (!ok) {
		wsmmu_off();
	}
	lck_mtx_unlock(&apollo_smmu_lock);
	return ok;
}

/* ---- the chip's preparation before MHI power-up (ath11k_pci_power_up), and the diagnostics ------------------ */

/*
 * What Linux does for a QCA6390 before the first stage goes over BHI: ath11k_pci_power_up() (drivers/net/wireless/ath/ath11k/pci.c) is
 *   ath11k_pci_sw_reset(power_on = true)   mdelay(100); ltssm; clear interrupts; wlaon power control; fix_l1ss (hw_params .fix_l1ss =
 *                                          true for "qca6390 hw2.0", core.c); mhi clear vector; debug registers; SoC global reset;
 *                                          MHICTRL reset
 *   ath11k_pci_aspm_disable()
 *   ath11k_pci_msi_enable()                (the MSI enable is moved here from W2c: first_stage() does it, after the SMMU is on)
 *   ath11k_mhi_start()                     mhi_prepare_for_power_up() (BHIe RX registers cleared), mhi_sync_power_up() =
 *                                          mhi_async_power_up() (drivers/bus/mhi/host/pm.c): BHI INTVEC, execution environment, state,
 *                                          SYS_ERR reset, then mhi_fw_load_handler() = first_stage()
 * The chip's own registers above 0x80000 are reached through a window in the BAR (pcic.h, pci.c ath11k_pci_window_{read,write}32(),
 * hw_params .static_window_map = false for QCA6390): window register at +0x310c, bit 30 enable, the value is offset[24:19]; the data is
 * at +0x80000 + offset[18:0]. Offsets below 0x80000 are direct. force wake (PCIE_SOC_WAKE_PCIE_LOCAL_REG) is not part of the power-up
 * (ath11k_pci_power_down() only), and the wakeup/release hooks are used once ATH11K_FLAG_DEVICE_INIT_DONE is set: not here.
 */
#define CHIP_WINDOW_ENABLE      0x40000000u     /* ATH11K_PCI_WINDOW_ENABLE_BIT (pcic.h) */
#define CHIP_WINDOW_REG         0x310c          /* ATH11K_PCI_WINDOW_REG_ADDRESS */
#define CHIP_WINDOW_START       0x80000u        /* ATH11K_PCI_WINDOW_START */
#define CHIP_WINDOW_RANGE       0x7ffffu        /* ATH11K_PCI_WINDOW_RANGE_MASK GENMASK(18, 0) */
#define CHIP_WINDOW_VALUE(o)    ((o) >> 19 & 0x3fu)     /* ATH11K_PCI_WINDOW_VALUE_MASK GENMASK(24, 19) */
#define CHIP_PARF_LTSSM         0x1e081b0u      /* PCIE_PCIE_PARF_LTSSM (pci.h) */
#define CHIP_LTSSM_VALUE        0x111u          /* PARM_LTSSM_VALUE */
#define CHIP_GCC_HOT_RST        0x1e402bcu      /* GCC_GCC_PCIE_HOT_RST */
#define CHIP_GCC_HOT_RST_VAL    0x10u           /* GCC_GCC_PCIE_HOT_RST_VAL */
#define CHIP_INT_ALL_CLEAR      0x1e08228u      /* PCIE_PCIE_INT_ALL_CLEAR */
#define CHIP_INT_CLEAR_ALL      0xffffffffu     /* PCIE_INT_CLEAR_ALL */
#define CHIP_QFPROM_PWR_CTRL    0x1f8031cu      /* WLAON_QFPROM_PWR_CTRL_REG */
#define CHIP_QFPROM_VDD4BLOW    0x4u            /* QFPROM_PWR_CTRL_VDD4BLOW_MASK */
#define CHIP_SYSCLK_EN_SEL      0x1e0c0acu      /* qca6390_regs .pcie_qserdes_sysclk_en_sel (hw.c) */
#define CHIP_SYSCLK_EN_SEL_VAL  0x10u           /* PCIE_QSERDES_COM_SYSCLK_EN_SEL_VAL, mask 0xffffffff */
#define CHIP_OSC_DTCT_CONFIG1   0x1e0c628u      /* qca6390_regs .pcie_pcs_osc_dtct_config_base (hw.c) */
#define CHIP_OSC_DTCT_CONFIG2   (CHIP_OSC_DTCT_CONFIG1 + 0x4)   /* PCIE_PCS_OSC_DTCT_CONFIG2_REG */
#define CHIP_OSC_DTCT_CONFIG4   (CHIP_OSC_DTCT_CONFIG1 + 0xc)   /* PCIE_PCS_OSC_DTCT_CONFIG4_REG */
#define CHIP_OSC_DTCT_MASK      0x000000ffu     /* PCIE_PCS_OSC_DTCT_CONFIG_MSK (the values: 0x02, 0x52, 0xff) */
#define CHIP_TXVECDB            0x360u          /* PCIE_TXVECDB (ath11k mhi.h) */
#define CHIP_TXVECSTATUS        0x368u          /* PCIE_TXVECSTATUS */
#define CHIP_RXVECDB            0x394u          /* PCIE_RXVECDB */
#define CHIP_RXVECSTATUS        0x39cu          /* PCIE_RXVECSTATUS */
#define CHIP_Q6_COOKIE          0x1f80500u      /* PCIE_Q6_COOKIE_ADDR */
#define CHIP_WARM_SW_ENTRY      0x1f80504u      /* WLAON_WARM_SW_ENTRY */
#define CHIP_RESET_CAUSE        0x1f8060cu      /* WLAON_SOC_RESET_CAUSE_REG: a read-clear register */
#define CHIP_GLOBAL_RESET       0x3008u         /* PCIE_SOC_GLOBAL_RESET */
#define CHIP_GLOBAL_RESET_V     1u              /* PCIE_SOC_GLOBAL_RESET_V */
#define CHIP_SOC_WAKE           0x3004u         /* PCIE_SOC_WAKE_PCIE_LOCAL_REG (read in the diagnostics only) */
#define CHIP_TCSR_HW_VERSION    0x0224u         /* TCSR_SOC_HW_VERSION (pci.c), read by ath11k_pci_probe() */
#define BHIE_RXVECADDR_LOW      0x60            /* BHIE_RXVECADDR_LOW_OFFS .. BHIE_RXVECSTATUS_OFFS (0x78): mhi_prepare_for_power_up() clears them */
#define BHIE_RXVECSTATUS        0x78
#define MHICTRL_RESET           (1u << 1)       /* as in W2c's list below */
#define MHISTATUS_STATE_SHIFT   8
#define MHI_EE_AMSS             2
#define MHI_STATE_SYS_ERR       0xff            /* MHI_STATE_SYS_ERR, include/linux/mhi.h */
#define MHI_EE_PBL              0               /* MHI_IN_PBL(): PBL, PTHRU, EDL; MHI_POWER_UP_CAPABLE() adds AMSS (host/internal.h) */
#define MHI_EE_PTHRU            5               /* MHI_EE_LIST order: PBL SBL AMSS RDDM WFW PTHRU EDL (names[] above) */
#define MHI_EE_EDL              6
#define PCI_CAP_EXP             0x10

static uint32_t chip_prep;              /* boot-arg apollo_wifi_prep: the preparation above (and MSI before BHI) is done */
static uint32_t chip_window;            /* ath11k_pci->register_window: the window last selected */

static void wifi_final_dump(const char *why);
static void wifi_rddm_collect(const char *why);
static bool w2c_rddm_prepare(void);
static bool w2c_msi_setup(void);

static bool
chip_window_ok(void)
{
	return bar_mapped >= 2 * CHIP_WINDOW_START;
}

/* ath11k_pci_select_window(): the window register written when the window changes (then read back, to flush it) */
static void
chip_select_window(uint32_t offset)
{
	uint32_t window = CHIP_WINDOW_VALUE(offset);

	if (window != chip_window) {
		wr(bar + CHIP_WINDOW_REG, CHIP_WINDOW_ENABLE | window);
		(void)rd(bar + CHIP_WINDOW_REG);
		chip_window = window;
	}
}

/* ath11k_pcic_read32() (__ath11k_pcic_read32(): direct below ATH11K_PCI_WINDOW_START, ath11k_pci_window_read32() above) */
static uint32_t
chip_rd(uint32_t offset)
{
	if (offset < CHIP_WINDOW_START) {
		return rd(bar + offset);
	}
	if (!chip_window_ok()) {
		return ~0u;
	}
	chip_select_window(offset);
	return rd(bar + CHIP_WINDOW_START + (offset & CHIP_WINDOW_RANGE));
}

static void
chip_wr(uint32_t offset, uint32_t value)
{
	if (offset < CHIP_WINDOW_START) {
		wr(bar + offset, value);
		return;
	}
	if (!chip_window_ok()) {
		return;
	}
	chip_select_window(offset);
	wr(bar + CHIP_WINDOW_START + (offset & CHIP_WINDOW_RANGE), value);
}

/* ath11k_pci_restore_window(): the window register's content is cleared by a global reset; written again from the cache */
static void
chip_restore_window(void)
{
	wr(bar + CHIP_WINDOW_REG, CHIP_WINDOW_ENABLE | chip_window);
	(void)rd(bar + CHIP_WINDOW_REG);
}

/* ath11k_pci_set_link_reg(): up to ten tries until the masked bits read back as asked */
static bool
chip_set_reg(uint32_t offset, uint32_t value, uint32_t mask, const char *name)
{
	uint32_t v = chip_rd(offset);

	if ((v & mask) == value) {
		say("apollo wifi: prep: %s (%#x) already reads %#x\n", name, offset, v);
		return true;
	}
	for (int i = 0; i < 10; i++) {
		chip_wr(offset, (v & ~mask) | value);
		v = chip_rd(offset);
		if ((v & mask) == value) {
			say("apollo wifi: prep: %s (%#x) set to %#x (try %d)\n", name, offset, v, i + 1);
			return true;
		}
		sleep_ms(2);
	}
	say("apollo wifi: prep: failed to set %s (%#x): %#x != %#x\n", name, offset, v & mask, value);
	return false;
}

/* ath11k_pci_fix_l1ss() */
static void
chip_fix_l1ss(void)
{
	chip_set_reg(CHIP_SYSCLK_EN_SEL, CHIP_SYSCLK_EN_SEL_VAL, 0xffffffffu, "QSERDES SYSCLK_EN_SEL");
	chip_set_reg(CHIP_OSC_DTCT_CONFIG1, 0x02, CHIP_OSC_DTCT_MASK, "PCS OSC_DTCT_CONFIG1");
	chip_set_reg(CHIP_OSC_DTCT_CONFIG2, 0x52, CHIP_OSC_DTCT_MASK, "PCS OSC_DTCT_CONFIG2");
	chip_set_reg(CHIP_OSC_DTCT_CONFIG4, 0xff, CHIP_OSC_DTCT_MASK, "PCS OSC_DTCT_CONFIG4");
}

/* ath11k_pci_enable_ltssm() */
static void
chip_enable_ltssm(void)
{
	uint32_t v = chip_rd(CHIP_PARF_LTSSM);

	/* "PCIE link seems very unstable after the Hot Reset" */
	for (int i = 0; v != CHIP_LTSSM_VALUE && i < 5; i++) {
		if (v == ~0u) {
			sleep_ms(5);
		}
		chip_wr(CHIP_PARF_LTSSM, CHIP_LTSSM_VALUE);
		v = chip_rd(CHIP_PARF_LTSSM);
	}
	say("apollo wifi: prep: ltssm %#x (PARF_LTSSM %#x)\n", v, CHIP_PARF_LTSSM);
	v = chip_rd(CHIP_GCC_HOT_RST);
	v |= CHIP_GCC_HOT_RST_VAL;
	chip_wr(CHIP_GCC_HOT_RST, v);
	v = chip_rd(CHIP_GCC_HOT_RST);
	say("apollo wifi: prep: pcie_hot_rst %#x (GCC_PCIE_HOT_RST %#x)\n", v, CHIP_GCC_HOT_RST);
	sleep_ms(5);
}

/* ath11k_pci_clear_dbg_registers() */
static void
chip_clear_dbg_registers(void)
{
	uint32_t cookie, warm, cause;

	cookie = chip_rd(CHIP_Q6_COOKIE);               /* read cookie */
	warm = chip_rd(CHIP_WARM_SW_ENTRY);
	sleep_ms(10);
	chip_wr(CHIP_WARM_SW_ENTRY, 0);                 /* "to prevent Q6 from continuing warm path and entering dead loop" */
	sleep_ms(10);
	say("apollo wifi: prep: pcie_q6_cookie_addr %#x, wlaon_warm_sw_entry %#x, then %#x\n", cookie, warm, chip_rd(CHIP_WARM_SW_ENTRY));
	cause = chip_rd(CHIP_RESET_CAUSE);              /* "a read clear register. clear the register to prevent Q6 from entering wrong code path" */
	say("apollo wifi: prep: soc reset cause %u\n", cause);
}

/* ath11k_pci_soc_global_reset(): the chip goes back to PBL */
static void
chip_soc_global_reset(void)
{
	uint32_t val = chip_rd(CHIP_GLOBAL_RESET);

	val |= CHIP_GLOBAL_RESET_V;
	chip_wr(CHIP_GLOBAL_RESET, val);
	(void)chip_rd(CHIP_GLOBAL_RESET);               /* flush the posted write */
	sleep_ms(10);                                   /* Linux: "exact time to sleep is uncertain" */
	val &= ~CHIP_GLOBAL_RESET_V;                    /* "toggle V bit back otherwise stuck in reset status" */
	chip_wr(CHIP_GLOBAL_RESET, val);
	(void)chip_rd(CHIP_GLOBAL_RESET);
	sleep_ms(10);
	val = chip_rd(CHIP_GLOBAL_RESET);
	if (val == 0xffffffffu) {
		say("apollo wifi: prep: link down error during global reset\n");
	}
	chip_restore_window();
	say("apollo wifi: prep: global reset done (PCIE_SOC_GLOBAL_RESET reads %#x)\n", val);
}

/* the root of the chip's configuration space as W1 and W2a left it, and after the reset */
struct cfg_snap {
	uint32_t id, command, bar0, bar1;
	bool link;
};

static void
cfg_look(const char *when, struct cfg_snap *s)
{
	s->id = rd(conf);
	s->command = rd(conf + 0x04) & 0xffff;
	s->bar0 = rd(conf + 0x10);
	s->bar1 = rd(conf + 0x14);
	s->link = link_up();
	say("apollo wifi: prep: %s: PCI id %#x, command %#x, BAR0 %#x:%#x, link %s (port debug %#x)\n", when, s->id, s->command, s->bar1, s->bar0,
	    s->link ? "up" : "DOWN", rd(dbi + DBI_PORT_DEBUG1));
}

/* the offset of a capability in the chip's configuration space, 0: none */
static uint32_t
chip_capability(uint32_t id)
{
	uint32_t cap = rd(conf + 0x34) & 0xfc;

	for (unsigned hops = 0; cap != 0 && hops < 48; hops++) {
		uint32_t word = rd(conf + cap);

		if ((word & 0xff) == id) {
			return cap;
		}
		cap = word >> 8 & 0xfc;
	}
	return 0;
}

/* ath11k_pci_aspm_disable(): L0s and L1 off in the chip's link control (bits 1:0 of PCI_EXP_LNKCTL; Linux keeps the old value to restore) */
static void
chip_aspm_disable(void)
{
	uint32_t cap = chip_capability(PCI_CAP_EXP), v;

	if (cap == 0) {
		say("apollo wifi: prep: no PCI Express capability in the chip's list: ASPM left alone\n");
		return;
	}
	v = rd(conf + cap + 0x10);
	say("apollo wifi: prep: link_ctl %#x (PCIe capability at conf+%#x): L0s %u L1 %u; both off\n", v & 0xffff, cap, v & 1, v >> 1 & 1);
	/* the high half is the link status, whose bits 14 and 15 are write-1-to-clear: written as zeros */
	wr(conf + cap + 0x10, (v & 0xfffcu));
	say("apollo wifi: prep: link_ctl now %#x\n", rd(conf + cap + 0x10) & 0xffff);
}

/* ath11k_pci_power_up()'s part before the BHI load: see the comment above. false: the chip or its link is not there. */
static bool
chip_prepare(void)
{
	struct cfg_snap before, after;
	uint32_t v, bhi, bhie, ee, state, hw;
	uint64_t began;

	if (!chip_window_ok()) {
		say("apollo wifi: prep: BAR0 is mapped for only %#lx bytes, the window needs %#x: no preparation\n", (unsigned long)bar_mapped,
		    2 * CHIP_WINDOW_START);
		return false;
	}
	cfg_look("before", &before);
	hw = rd(bar + CHIP_TCSR_HW_VERSION);
	bhi = rd(bar + MHI_BHI_OFF);                    /* (read once; all ones from a chip that does not answer: no read through it) */
	say("apollo wifi: prep: TCSR_SOC_HW_VERSION %#x (major %u, minor %u); before: MHISTATUS %#x, MHICTRL %#x, BHI at +%#x, BHI execution environment %#x\n", hw,
	    hw >> 8 & 0xf, hw & 0xff, rd(bar + MHI_STATUS), rd(bar + MHI_CTRL), bhi,
	    bhi != 0 && bhi != ~0u && (uint64_t)bhi + BHI_EXECENV + 4 <= bar_mapped ? rd(bar + bhi + BHI_EXECENV) : ~0u);
	w2b_hold("the chip-internal registers: the BAR window register, then the chip's PARF LTSSM, GCC hot reset, interrupt clear, WLAON power control, "
	    "QSERDES/PCS L1SS registers, BHIe/MHI vector registers, WLAON warm entry");
	chip_window = 0;                                /* ath11k_pci_power_up(): ab_pci->register_window = 0 */
	chip_restore_window();                          /* (the window register itself is not written by Linux here; it is by its restore, as after the reset) */
	/* ath11k_pci_sw_reset(power_on = true) */
	sleep_ms(100);                                  /* mdelay(100) */
	chip_enable_ltssm();
	chip_wr(CHIP_INT_ALL_CLEAR, CHIP_INT_CLEAR_ALL);        /* ath11k_pci_clear_all_intrs(): the WAR for the PCIe hot reset's interrupt */
	v = chip_rd(CHIP_QFPROM_PWR_CTRL);              /* ath11k_pci_set_wlaon_pwr_ctrl() */
	v &= ~CHIP_QFPROM_VDD4BLOW;
	chip_wr(CHIP_QFPROM_PWR_CTRL, v);
	say("apollo wifi: prep: wlaon qfprom power control %#x (cleared VDD4BLOW), interrupts cleared\n", v);
	chip_fix_l1ss();                                /* hw_params.fix_l1ss */
	/* ath11k_mhi_clear_vector(): the BHIe vector registers (below the window: direct) */
	chip_wr(CHIP_TXVECDB, 0);
	chip_wr(CHIP_TXVECSTATUS, 0);
	chip_wr(CHIP_RXVECDB, 0);
	chip_wr(CHIP_RXVECSTATUS, 0);
	chip_clear_dbg_registers();
	w2b_hold("the SoC global reset (PCIE_SOC_GLOBAL_RESET): the chip restarts in PBL");
	chip_soc_global_reset();
	/* ath11k_mhi_set_mhictrl_reset(): "After SOC_GLOBAL_RESET, MHISTATUS may still have SYSERR bit set and thus need to set MHICTRL_RESET" */
	v = rd(bar + MHI_STATUS);
	say("apollo wifi: prep: mhistatus %#x after the global reset; MHICTRL_RESET written\n", v);
	wr(bar + MHI_CTRL, MHICTRL_RESET);
	sleep_ms(10);
	/* the link, the configuration space and the BAR as they were? (Linux only warns when the reset read comes back all ones) */
	for (began = now_us(); now_us() - began < 1000000 && !link_up(); ) {
		sleep_ms(10);
	}
	cfg_look("after the global reset", &after);
	if (!after.link || after.id != 0x110117cb) {
		say("apollo wifi: prep: THE LINK OR THE CHIP IS GONE after the global reset\n");
		return false;
	}
	if (after.bar0 != before.bar0 || after.bar1 != before.bar1) {
		say("apollo wifi: prep: BAR0 changed in the reset (%#x:%#x was %#x:%#x): written again\n", after.bar1, after.bar0, before.bar1, before.bar0);
		wr(conf + 0x10, before.bar0);
		wr(conf + 0x14, before.bar1);
	}
	if ((after.command & 0x6) != 0x6) {
		say("apollo wifi: prep: the command register lost memory/master (%#x): written again\n", after.command);
		wr(conf + 0x04, (rd(conf + 0x04) & 0xffff) | 0x6);
	}
	/* the chip's MMIO answers again? */
	for (began = now_us(); now_us() - began < 2000000; sleep_ms(10)) {
		v = rd(bar + MHI_VER);
		if (v != ~0u && v != 0) {
			break;
		}
	}
	say("apollo wifi: prep: MHI version %#x %llu ms after the reset's end; BAR0 reads back %#x\n", v, (now_us() - began) / 1000, rd(conf + 0x10));
	chip_aspm_disable();                            /* ath11k_pci_aspm_disable() */
	/* mhi_prepare_for_power_up(): BHI and BHIe offsets, the BHIe RX registers cleared ("POR values are undefined"); the RDDM table that follows is not ported */
	bhi = rd(bar + MHI_BHI_OFF);
	bhie = rd(bar + MHI_BHIE_OFF);
	if (bhi == 0 || bhi >= bar_mapped || bhie == 0 || bhie + BHIE_RXVECSTATUS + 4 > bar_mapped) {
		say("apollo wifi: prep: BHI offset %#x or BHIe offset %#x out of range\n", bhi, bhie);
		return false;
	}
	for (uint32_t o = BHIE_RXVECADDR_LOW; o <= BHIE_RXVECSTATUS; o += 4) {
		wr(bar + bhie + o, 0);
	}
	/* mhi_async_power_up() */
	wr(bar + bhi + BHI_INTVEC, 0);                  /* Setup BHI INTVEC */
	ee = rd(bar + bhi + BHI_EXECENV);
	state = rd(bar + MHI_STATUS) >> MHISTATUS_STATE_SHIFT & 0xff;
	say("apollo wifi: prep: power on with execution environment %u (%s), MHI state %#x, BHI at +%#x, BHIe at +%#x\n", ee, execution_environment(ee), state,
	    bhi, bhie);
	if (!(ee == MHI_EE_PBL || ee == MHI_EE_PTHRU || ee == MHI_EE_EDL || ee == MHI_EE_AMSS)) {       /* MHI_POWER_UP_CAPABLE() */
		say("apollo wifi: prep: %s is not a valid execution environment for power on\n", execution_environment(ee));
		return false;
	}
	if (state == MHI_STATE_SYS_ERR) {
		/* mhi_set_mhi_state(RESET): MHICTRL.RESET = 1, polled until the chip clears it (25 ms steps), then INTVEC again */
		wr(bar + MHI_CTRL, rd(bar + MHI_CTRL) | MHICTRL_RESET);
		for (began = now_us(); now_us() - began < 10000000 && (rd(bar + MHI_CTRL) & MHICTRL_RESET); ) {
			sleep_ms(25);
		}
		say("apollo wifi: prep: SYS_ERR: MHI reset %s after %llu ms (MHICTRL %#x)\n", (rd(bar + MHI_CTRL) & MHICTRL_RESET) ? "NOT cleared" : "cleared",
		    (now_us() - began) / 1000, rd(bar + MHI_CTRL));
		wr(bar + bhi + BHI_INTVEC, 0);          /* "device cleares INTVEC as part of RESET processing, re-program it" */
	}
	say("apollo wifi: prep: done: MHISTATUS %#x, MHICTRL %#x, BHI status %#x\n", rd(bar + MHI_STATUS), rd(bar + MHI_CTRL), rd(bar + bhi + BHI_STATUS));
	return true;
}

/*
 * The diagnostics: one line when anything of these changed. The BHI and MHI registers are read each time (W2b/W2c polls), the chip's own
 * (windowed) ones at most every 100 ms and only with the preparation on. Not read here: WLAON_SOC_RESET_CAUSE (it clears on read), and
 * ath11k has no register for the SR status or the interrupt status of the chip's PCIe: nothing is guessed.
 */
#define DIAG_DIRECT     9
#define DIAG_WINDOWED   9
static struct {
	uint32_t direct[DIAG_DIRECT], windowed[DIAG_WINDOWED];
	uint64_t windowed_at;
	bool any;
} diag;

static void
diag_read(uint32_t *direct, uint32_t *windowed)
{
	direct[0] = rd(bhi_base + BHI_ERRCODE);
	direct[1] = rd(bhi_base + BHI_ERRDBG1);
	direct[2] = rd(bhi_base + BHI_ERRDBG2);
	direct[3] = rd(bhi_base + BHI_ERRDBG3);
	direct[4] = rd(bhi_base + BHI_EXECENV);
	direct[5] = rd(bar + MHI_STATUS);
	direct[6] = rd(bar + MHI_CTRL);
	direct[7] = rd(bhi_base + BHI_STATUS);
	direct[8] = rd(conf);                   /* the chip's PCI id: all ones when its link is gone */
	if (windowed != NULL) {
		windowed[0] = chip_rd(CHIP_WARM_SW_ENTRY);
		windowed[1] = chip_rd(CHIP_Q6_COOKIE);
		windowed[2] = chip_rd(CHIP_PARF_LTSSM);
		windowed[3] = chip_rd(CHIP_GCC_HOT_RST);
		windowed[4] = chip_rd(CHIP_SYSCLK_EN_SEL);
		windowed[5] = chip_rd(CHIP_OSC_DTCT_CONFIG1);
		windowed[6] = chip_rd(CHIP_QFPROM_PWR_CTRL);
		windowed[7] = chip_rd(CHIP_GLOBAL_RESET);
		windowed[8] = chip_rd(CHIP_SOC_WAKE);
	}
}

static void
diag_tell(const char *phase, uint64_t began, const uint32_t *direct, const uint32_t *windowed)
{
	say("apollo wifi: diag: %s +%llu ms: BHI error %#x, debug %#x %#x %#x, EE %#x, MHISTATUS %#x, MHICTRL %#x, BHI status %#x, PCI id %#x, link %s\n",
	    phase, (now_us() - began) / 1000, direct[0], direct[1], direct[2], direct[3], direct[4], direct[5], direct[6], direct[7], direct[8],
	    link_up() ? "up" : "DOWN");
	if (windowed != NULL) {
		say("apollo wifi: diag: %s +%llu ms: warm_sw_entry %#x, q6 cookie %#x, PARF_LTSSM %#x, hot_rst %#x, sysclk_en_sel %#x, osc_dtct1 %#x, "
		    "qfprom_pwr %#x, global_reset %#x, soc_wake %#x\n", phase, (now_us() - began) / 1000, windowed[0], windowed[1], windowed[2], windowed[3],
		    windowed[4], windowed[5], windowed[6], windowed[7], windowed[8]);
	}
}

/* log once per change */
static void
diag_poll(const char *phase, uint64_t began)
{
	uint32_t direct[DIAG_DIRECT], windowed[DIAG_WINDOWED];
	bool with_window = chip_prep != 0 && chip_window_ok() && (now_us() - diag.windowed_at >= 100000 || !diag.any);
	bool changed;

	if (bhi_base == 0) {
		return;
	}
	diag_read(direct, with_window ? windowed : NULL);
	changed = !diag.any || memcmp(direct, diag.direct, sizeof(direct)) != 0;
	if (with_window) {
		diag.windowed_at = now_us();
		changed = changed || !diag.any || memcmp(windowed, diag.windowed, sizeof(windowed)) != 0;
	}
	if (changed) {
		diag_tell(phase, began, direct, with_window ? windowed : NULL);
		memcpy(diag.direct, direct, sizeof(direct));
		if (with_window) {
			memcpy(diag.windowed, windowed, sizeof(windowed));
		}
		diag.any = true;
	}
}

extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);

/* The first stage over BHI, and the chip's answer */
static bool
first_stage(void)
{
	const uint8_t *image;
	uint64_t size = 0, began;
	uint32_t command, offset, session, ee, mhi, status = 0, last_ee, last_mhi, last_status, unreadable = 0;
	bool told = false, success = false;
	int disk = -1;

	if (chip_prep && !chip_prepare()) {
		say("apollo wifi: W2b: the chip's preparation failed\n");
		wifi_final_dump("after a failed preparation");
		return false;
	}
	offset = rd(bar + MHI_BHI_OFF);
	if (offset == 0 || offset == ~0u || (uint64_t)offset + BHI_SERIALNU + 4 > bar_mapped) {   /* (BHI's registers go up to SERIALNU) */
		say("apollo wifi: W2b: no BHI block where MHI says (%#x)\n", offset);
		return false;
	}
	bhi_base = bar + offset;
	image = hakios_image_file("fw", "amss.bin", &size, &disk);
	if (image == NULL || size < FIRST_STAGE) {
		say("apollo wifi: W2b: no fw/amss.bin of at least %#x bytes in the small image (%llu bytes)\n", FIRST_STAGE, size);
		return false;
	}
	say("apollo wifi: W2b: fw/amss.bin: %llu bytes in the image of /dev/md%d; the first %#x go to the chip\n", size, disk, FIRST_STAGE);
	if (kmem_alloc(kernel_map, &wsmmu.buffer, FIRST_STAGE, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		wsmmu.buffer = 0;
		say("apollo wifi: W2b: no memory for the first stage\n");
		return false;
	}
	memcpy((void *)wsmmu.buffer, image, FIRST_STAGE);
	for (vm_offset_t line = wsmmu.buffer; line < wsmmu.buffer + FIRST_STAGE; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__asm__ volatile ("dsb sy");
	wsmmu.base = ml_io_map(SMMU_PHYS, SMMU_SIZE);
	if (wsmmu.base == 0) {
		say("apollo wifi: W2b: cannot map the SMMU\n");
		return false;
	}
	if (!wsmmu_on()) {
		return false;
	}
	/* the chip's stream: 0x1c01 for bus 1, device 0, function 0 (the root port's 0x1c00 is a zero entry, as Linux writes it) */
	say("apollo wifi: W2b: the controller's stream table had %#x at the chip's slot %#x; now %#x\n",
	    rd(parf + PARF_BDF_TO_SID(WIFI_BDF_HASH)), WIFI_BDF_HASH, WIFI_BDF << 16 | (WIFI_STREAM - 0x1c00) << 8);
	for (uint32_t n = 0; n < BDF_TO_SID_ENTRIES; n++) {
		wr(parf + PARF_BDF_TO_SID(n), 0);
	}
	wr(parf + PARF_BDF_TO_SID(WIFI_BDF_HASH), WIFI_BDF << 16 | (WIFI_STREAM - 0x1c00) << 8);
	__asm__ volatile ("dsb sy");
	/* the chip decodes memory and masters the bus */
	wr(conf + 0x04, rd(conf + 0x04) | 0x6);
	command = rd(conf + 0x04);
	ee = rd(bhi_base + BHI_EXECENV);
	mhi = rd(bar + MHI_STATUS);
	say("apollo wifi: W2b: PCI command %#x; before: execution environment %u (%s), MHI state %u%s%s, MHICTRL %#x, BHI status %#x, serial "
	    "number %#x\n", command & 0xffff, ee, execution_environment(ee), mhi >> 8 & 0xff, (mhi & 1) ? ", ready" : "",
	    (mhi & 4) ? ", SYSTEM ERROR" : "", rd(bar + MHI_CTRL), rd(bhi_base + BHI_STATUS), rd(bhi_base + BHI_SERIALNU));
	if (chip_prep) {
		/* ath11k_pci_msi_enable() comes before ath11k_mhi_start(): the chip's MSI is on before the BHI load */
		w2c_msi_setup();                /* (its failure is not W2b's: the polls go on) */
	}
	session = ((uint32_t)mach_absolute_time() & 0x3fffffff) | 1;
	say("apollo wifi: W2b: BHI: image at %#llx, %#x bytes, session %#x\n", WIFI_ADDRESSES, FIRST_STAGE, session);
	w2b_hold("the BHI registers and the doorbell: the chip reads the image");
	/* Linux's order (mhi_fw_load_bhi) */
	wr(bhi_base + BHI_STATUS, 0);
	wr(bhi_base + BHI_IMGADDR_HIGH, (uint32_t)(WIFI_ADDRESSES >> 32));
	wr(bhi_base + BHI_IMGADDR_LOW, (uint32_t)WIFI_ADDRESSES);
	wr(bhi_base + BHI_IMGSIZE, FIRST_STAGE);
	wr(bhi_base + BHI_INTVEC, 0);
	wr(bhi_base + BHI_IMGTXDB, session);
	last_ee = ee;
	last_mhi = mhi;
	last_status = 0x12345678;
	began = now_us();
	while (now_us() - began < 20000000) {
		sleep_ms(10);
		status = rd(bhi_base + BHI_STATUS);
		ee = rd(bhi_base + BHI_EXECENV);
		mhi = rd(bar + MHI_STATUS);
		if (ee != last_ee || mhi != last_mhi || status != last_status) {
			say("apollo wifi: W2b: %llu ms: BHI status %#x, execution environment %u (%s), MHI state %u%s%s\n",
			    (now_us() - began) / 1000, status, ee, execution_environment(ee), mhi >> 8 & 0xff, (mhi & 1) ? ", ready" : "",
			    (mhi & 4) ? ", SYSTEM ERROR" : "");
			last_ee = ee;
			last_mhi = mhi;
			last_status = status;
		}
		wsmmu_faults(&told);
		diag_poll("BHI load", began);
		/* all ones: the chip did not answer the read (Linux's MHI takes it for a link down), not an error code */
		if (status == ~0u) {
			unreadable++;
			continue;
		}
		if ((status >> 30) == BHI_STATUS_SUCCESS || (status >> 30) == BHI_STATUS_ERROR) {
			break;
		}
	}
	if (unreadable != 0) {
		say("apollo wifi: W2b: %u reads of the BHI status came back all ones; last status %#x, error %#x, execution environment %#x, "
		    "MHI status %#x, PCI id %#x\n", unreadable, status, rd(bhi_base + BHI_ERRCODE), rd(bhi_base + BHI_EXECENV),
		    rd(bar + MHI_STATUS), rd(conf + 0x00));
	}
	wsmmu_faults(&told);
	if ((status >> 30) == BHI_STATUS_ERROR) {
		say("apollo wifi: W2b: THE CHIP REFUSED THE FIRST STAGE: BHI status %#x, error %#x, debug %#x %#x %#x\n", status,
		    rd(bhi_base + BHI_ERRCODE), rd(bhi_base + BHI_ERRDBG1), rd(bhi_base + BHI_ERRDBG2), rd(bhi_base + BHI_ERRDBG3));
		return false;
	}
	if ((status >> 30) != BHI_STATUS_SUCCESS) {
		say("apollo wifi: W2b: THE CHIP DID NOT ANSWER in 20 s: BHI status %#x, error %#x, execution environment %u\n", status,
		    rd(bhi_base + BHI_ERRCODE), rd(bhi_base + BHI_EXECENV));
		wifi_final_dump("W2b: no answer");
		return false;
	}
	success = true;
	say("apollo wifi: W2b: the chip took the first stage after %llu ms: BHI status %#x\n", (now_us() - began) / 1000, status);
	/* where the chip goes from there: PBL toward SBL */
	began = now_us();
	diag.any = false;
	diag_poll("after BHI", began);
	while (now_us() - began < 5000000) {
		sleep_ms(10);
		ee = rd(bhi_base + BHI_EXECENV);
		mhi = rd(bar + MHI_STATUS);
		if (ee != last_ee || mhi != last_mhi) {
			say("apollo wifi: W2b: +%llu ms: execution environment %u (%s), MHI state %u%s%s\n", (now_us() - began) / 1000, ee,
			    execution_environment(ee), mhi >> 8 & 0xff, (mhi & 1) ? ", ready" : "", (mhi & 4) ? ", SYSTEM ERROR" : "");
			last_ee = ee;
			last_mhi = mhi;
		}
		wsmmu_faults(&told);
		diag_poll("after BHI", began);
	}
	say("apollo wifi: W2b: after 5 s: execution environment %u (%s), MHI state %u, BHI status %#x\n", last_ee,
	    execution_environment(last_ee), last_mhi >> 8 & 0xff, rd(bhi_base + BHI_STATUS));
	if (success) {
		say("apollo wifi: W2b DONE: the QCA6390 took the first stage of its firmware over BHI\n");
	}
	return success;
}

/* ---- step W2c: from PBL through SBL to mission mode (MHI and BHIe) ------------------------------------- */

/*
 * The part of Linux's MHI host that gets the chip there (drivers/bus/mhi/host/{pm,boot,init,main}.c, with ath11k's
 * QCA6390 configuration in drivers/net/wireless/ath/ath11k/mhi.c), polling, no interrupts:
 *   mhi_ready_state_transition(): MHICTRL.RESET clear and MHISTATUS.READY set, then mhi_init_mmio() (the context
 *   arrays and the rings, as the chip sees them), the event rings' write pointers and doorbells, MHI state M0;
 *   the chip goes PBL to SBL; mhi_download_amss_image() (mhi_fw_load_bhie()): all of amss.bin in 512 KB segments
 *   (ath11k: seg_len) and a vector table of { address, size } for them, handed over the BHIe block;
 *   then SBL to AMSS (execution environment 2).
 * Everything the chip reads or writes is behind the Wi-Fi's SMMU context, from where W2b's mapping ends on.
 */
/* register offsets: drivers/bus/mhi/common.h (MHI block, from the BAR's start) */
#define MHI_CHDBOFF             0x18            /* CHDBOFF */
#define MHI_ERDBOFF             0x20            /* ERDBOFF: where the event ring doorbells are, 8 bytes each */
#define MHI_CCABAP_LOWER        0x58            /* CCABAP_LOWER, and CCABAP_HIGHER at 0x5c: the channel contexts */
#define MHI_ECABAP_LOWER        0x60            /* ECABAP_LOWER, and 0x64: the event contexts */
#define MHI_CRCBAP_LOWER        0x68            /* CRCBAP_LOWER, and 0x6c: the command context */
#define MHI_CTRLBASE_LOWER      0x80            /* MHICTRLBASE_LOWER, and 0x84 */
#define MHI_CTRLLIMIT_LOWER     0x88            /* MHICTRLLIMIT_LOWER, and 0x8c */
#define MHI_DATABASE_LOWER      0x98            /* MHIDATABASE_LOWER, and 0x9c */
#define MHI_DATALIMIT_LOWER     0xa0            /* MHIDATALIMIT_LOWER, and 0xa4 */
#define MHICFG_NER_SHIFT        16              /* MHICFG_NER_MASK GENMASK(23, 16): event rings */
#define MHICFG_NHWER_SHIFT      24              /* MHICFG_NHWER_MASK GENMASK(31, 24): hardware event rings */
#define MHICTRL_RESET           (1u << 1)       /* MHICTRL_RESET_MASK BIT(1) */
#define MHICTRL_STATE_SHIFT     8               /* MHICTRL_MHISTATE_MASK GENMASK(15, 8) */
#define MHISTATUS_STATE_SHIFT   8               /* MHISTATUS_MHISTATE_MASK GENMASK(15, 8) */
#define MHISTATUS_SYSERR        (1u << 2)       /* MHISTATUS_SYSERR_MASK */
#define MHISTATUS_READY         (1u << 0)       /* MHISTATUS_READY_MASK */
#define MHI_STATE_READY         1               /* enum mhi_state, include/linux/mhi.h */
#define MHI_STATE_M0            2
#define MHI_STATE_M1            3               /* enum mhi_state (include/linux/mhi.h): M1 3, M2 4, SYS_ERR 0xff */
#define MHI_STATE_M2            4
#define MHI_STATE_SYS_ERR       0xff
#define MHI_EE_SBL              1               /* MHI_EE_LIST order, drivers/bus/mhi/host/internal.h: PBL SBL AMSS RDDM ... */
#define MHI_EE_AMSS             2
#define MHI_EE_RDDM             3
/* the BHIe block (from BHIEOFF), drivers/bus/mhi/common.h */
#define BHIE_TXVECADDR_LOW      0x2c            /* BHIE_TXVECADDR_LOW_OFFS */
#define BHIE_TXVECADDR_HIGH     0x30            /* BHIE_TXVECADDR_HIGH_OFFS */
#define BHIE_TXVECSIZE          0x34            /* BHIE_TXVECSIZE_OFFS */
#define BHIE_TXVECDB            0x3c            /* BHIE_TXVECDB_OFFS: sequence id in [29:0] */
#define BHIE_TXVECSTATUS        0x44            /* BHIE_TXVECSTATUS_OFFS: sequence id [29:0], status [31:30] */
#define BHIE_SEQ_MASK           0x3fffffffu     /* BHIE_TXVECDB_SEQNUM_BMSK, BHIE_TXVECSTATUS_SEQNUM_BMSK */
#define BHIE_STATUS_XFER_COMPL  2u              /* BHIE_TXVECSTATUS_STATUS_XFER_COMPL */
#define BHIE_STATUS_ERROR       3u              /* BHIE_TXVECSTATUS_STATUS_ERROR */
/* the configuration: ath11k_mhi_config_qca6390 and ath11k_mhi_register() */
#define W2C_SEGMENT             0x80000u        /* mhi_ctrl->seg_len = SZ_512K, sbl_size too */
#define W2C_CHANNELS            128             /* .max_channels */
#define W2C_EVENT_RINGS         2               /* .num_events: ath11k_mhi_events_qca6390 */
#define W2C_ER0_ELEMENTS        32              /* event ring 0 (MHI_ER_CTRL, irq 1, no moderation) */
#define W2C_ER1_ELEMENTS        256             /* event ring 1 (irq 2, moderation 1 ms) */
#define W2C_CMD_ELEMENTS        128             /* CMD_EL_PER_RING, internal.h */
#define W2C_ELEMENT             16              /* sizeof(struct mhi_ring_element) */
#define W2C_ER_TYPE_VALID       1u              /* MHI_ER_TYPE_VALID, internal.h */
#define W2C_BRST_DISABLE        2u              /* MHI_DB_BRST_DISABLE, include/linux/mhi.h */
#define W2C_TO_DEVICE           1u              /* MHI_CH_TYPE_OUTBOUND = DMA_TO_DEVICE */
#define W2C_FROM_DEVICE         2u              /* MHI_CH_TYPE_INBOUND = DMA_FROM_DEVICE */
#define W2C_WAIT_US             20000000ull
/* MSI: /soc/qcom,pcie@1c00000 msi-parent -> /soc/qcom,pcie0_msi@17a10040 (compatible "qcom,pci-msi"): reg = GICD_SETSPI_NSR, interrupts =
 * GIC_SPI 0x300, 0x301, ... edge rising, so vector n is GIC INTID 32 + 0x300 + n; the downstream driver gives the chip the doorbell as
 * the address and the INTID as the data. The chip's write is DMA: it goes through the Wi-Fi's SMMU context like the rest. */
#define W2C_MSI_DOORBELL        0x17a10000ull   /* the 4K page of GICD_SETSPI_NSR (0x17a10040) */
#define W2C_MSI_SETSPI          0x40            /* the register's offset in that page */
#define W2C_MSI_INTID           800             /* 32 + 0x300: the data of vector 0 (32-aligned, as a multi-message MSI of 32 needs) */
#define W2C_MSI_VECTORS         3               /* ath11k_msi_config (pci.c), "MHI": 3 vectors from base 0: state/BHI, event ring 0, event ring 1 */
#define W2C_MSI_WANT_LOG2       5               /* all 32 of the chip's (ath11k: 32 in total), or what its MMC allows */
#define PTE_DEVICE              (2ull << 2)     /* AttrIdx 2 = byte 2 of SMMU_MAIR0 (0x04: Device-nGnRE); WIFI_PTE has index 0 (0x44: normal, non-cacheable) */
#define PCI_CAP_MSI             0x05
/* struct mhi_ring_element words and packet types (common.h): ptr in [0..1], dword0 in [2], dword1 in [3] */
#define W2C_PKT_CMD_COMPLETION  0x21            /* MHI_PKT_TYPE_CMD_COMPLETION_EVENT */
#define W2C_PKT_STATE_CHANGE    0x20            /* MHI_PKT_TYPE_STATE_CHANGE_EVENT */
#define W2C_PKT_EE              0x40            /* MHI_PKT_TYPE_EE_EVENT */
#define W2C_PKT_TX_EVENT        0x22            /* MHI_PKT_TYPE_TX_EVENT (common.h): a transfer completed */
#define W2C_EV_CC_SUCCESS       0x1             /* enum mhi_ev_ccs (common.h) */
#define W2C_EV_CC_EOT           0x2
#define W2C_EV_CC_OVERFLOW      0x3
#define W2C_EV_CC_EOB           0x4
#define W2C_CMD_RESET_CHAN      16              /* enum mhi_cmd_type (common.h) */
#define W2C_CMD_STOP_CHAN       17
#define W2C_CMD_START_CHAN      18
#define W2C_TRE_TYPE_TRANSFER   2               /* MHI_TRE_TYPE_TRANSFER */
#define MHI_CRDB_LOWER          0x70            /* CRDB_LOWER (common.h): the command ring's doorbell, and CRDB_HIGHER at 0x74 */
#define MHI_DEV_WAKE_DB         127             /* host/internal.h: the channel doorbell number of the wake doorbell */
#define W2C_EVENT_LOG_MOST      200
#define IPCR_UL                 20              /* the IPCR channels (below) */
#define IPCR_DL                 21
/* our layout of the control pages (4K each; Linux aligns each ring to its length, 4K does for all of these) */
#define W2C_CTRL_PAGES          8
#define W2C_OFF_CHAN            0x0000          /* 128 channel contexts of 44 bytes: 5632 bytes */
#define W2C_OFF_EVENT           0x2000          /* the event contexts */
#define W2C_OFF_CMD             0x3000          /* the command context */
#define W2C_OFF_CMD_RING        0x4000
#define W2C_OFF_ER0             0x5000
#define W2C_OFF_ER1             0x6000
#define W2C_OFF_VECTOR          0x7000          /* the vector table (struct bhi_vec_entry: 8 bytes address, 8 bytes size, per segment) */

/* struct mhi_chan_ctxt, mhi_event_ctxt, mhi_cmd_ctxt (common.h): three words and four 64-bit values: 44 bytes */
struct w2c_context {
	uint32_t word0, word1, word2;
	uint64_t rbase, rlen, rp, wp;
} __attribute__((packed, aligned(4)));

static struct {
	vm_offset_t ctrl, image;        /* never freed (the chip may use them until PERST# is down); ctrl is the UNCACHED alias (ctrl_alloc) */
	uint64_t ctrl_iova, image_iova;
	uint32_t segments;
	vm_offset_t bhie;
	uint32_t ee, mhi;               /* the last read */
	uint64_t rp[W2C_EVENT_RINGS], wp[W2C_EVENT_RINGS];      /* the host's pointers (mhi_ring rp, wp), as the chip's addresses */
	uint32_t erdb;                  /* ERDBOFF: the event ring doorbells */
	uint32_t chdb;                  /* CHDBOFF: the channel doorbells (8 bytes each, the wake doorbell 127 among them) */
	uint64_t msi_address;           /* the address the chip's MSI capability got (the doorbell's IOVA + 0x40) */
	volatile bool rings;            /* the event rings are armed (doorbells rung once): they may be consumed */
	unsigned dumped;
	bool told, outside;
	volatile uint32_t msi_cap;      /* the MSI capability's offset in configuration space, once it is written (0: not): quiesce clears its enable */
	volatile bool msi_attached;     /* the three INTIDs are attached */
	volatile bool msi_on;           /* the chip's MSI is enabled: w2c_wait() sleeps on the interrupt */
	volatile bool msi_cap_skip;     /* apollo_wifi_quiesce(): the capability is not touched (the link goes down) */
	uint32_t seen[W2C_MSI_VECTORS]; /* the counts last told */
} w2c;

static volatile uint32_t w2c_hits[W2C_MSI_VECTORS];     /* the interrupts that arrived, per vector */
static uint32_t w2c_wake;                               /* the event the handler wakes (its address) */

static LCK_GRP_DECLARE(wifi_lck_grp, "apollo-wifi");
static LCK_MTX_DECLARE(wifi_lock, &wifi_lck_grp);       /* apollo_wifi_power_up()/power_down(): one at a time */
static LCK_MTX_DECLARE(wifi_map_lock, &wifi_lck_grp);   /* the Wi-Fi's page tables (wsmmu_map_as) */
static LCK_MTX_DECLARE(wifi_ev_lock, &wifi_lck_grp);    /* the event rings (w2c_events) and the IPCR receive ring */
static LCK_SPIN_DECLARE(wifi_chip_lock, &wifi_lck_grp); /* apollo_wifi_read32/write32: the window register and the access after it */
static LCK_SPIN_DECLARE(wifi_wake_lock, &wifi_lck_grp); /* apollo_wifi_wake (Linux's mhi_cntrl->wlock) */
static volatile bool wifi_dead;                         /* the firmware crashed (SYS_ERR / RDDM seen): nothing but the RDDM collection touches the chip any more */
static volatile bool w2c_m2;                            /* the host put MHI in M2 (mhi_pm_m1_transition) and no M0 event came yet */
static volatile bool wifi_amss;                         /* the chip runs its firmware (AMSS) with MHI in M0: chip_rd/wr and the doorbells may be used */
/*
 * The link is up and PERST# high: BAR0 may be touched. Set by W1 once the link is up, cleared by apollo_wifi_quiesce() before PERST# drops.
 * The writers of the IPCR threads' paths (mhi_write64, the doorbells, apollo_wifi_read32/write32, the RDDM collection) check it, so that
 * a thread still on its way out after power down does not touch a BAR whose link is gone (that hangs the bus).
 */
static volatile bool wifi_link_alive;

static bool ipcr_event(unsigned ring, uint64_t index, const volatile uint32_t *tre);
static void mhi_wake_hold(void);
static bool mhi_wake_is_held(void);
static void mhi_wake_reassert(void);
static bool ipcr_cmd_pending(void);
static bool ipcr_up(void);
static bool ipcr_stopping(void);
void apollo_wifi_wake(int on);
static void wifi_mark_dead(const char *why);

static void
cache_flush(vm_offset_t address, size_t bytes)
{
	for (vm_offset_t line = address & ~(vm_offset_t)63; line < address + bytes; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__asm__ volatile ("dsb sy");
}

/*
 * Memory the chip and the CPU both write (the MHI contexts and rings): one physical piece (kmem_alloc_contig), written back and
 * invalidated once, then mapped a second time without the CPU's cache (VM_WIMG_WCOMB: normal non-cacheable, as hakios.lx.c's
 * lx_os_radio_memory does); the cached view is never used again. A 44-byte context shares its 64-byte line with fields the chip
 * writes (rp, chstate): through this alias a CPU store can not write a stale line back over them. Returns the alias, 0 on failure.
 */
static vm_offset_t
ctrl_alloc(vm_size_t size, uint64_t *physical)
{
	vm_offset_t cached = 0;

	if (kmem_alloc_contig(kernel_map, &cached, size, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		return 0;
	}
	bzero((void *)cached, size);
	cache_flush(cached, size);
	*physical = kvtophys(cached);
	return ml_io_map_unmappable(*physical, size, VM_WIMG_WCOMB);
}

/*
 * Map the pages of a kernel buffer (not contiguous in memory) at the next free addresses of the Wi-Fi's context, adding
 * third-level tables under the same second-level table as needed; the address of the first goes to *iova.
 */
static bool
wsmmu_map_as_locked(vm_offset_t buffer, uint64_t physical, uint32_t bytes, uint64_t attributes, uint64_t *iova)
{
	uint32_t first_table = ~0u, last_table = 0;
	uint64_t *second = (uint64_t *)(wsmmu.tables + 4096), at = wsmmu.next;
	uint32_t pages = (bytes + 4095) / 4096;
	vm_offset_t cb = wsmmu_context(wsmmu.context);

	if (!wsmmu.on || at + 4096ull * pages > WIFI_ADDRESSES + (uint64_t)WIFI_L3_TABLES * 0x200000) {
		say("apollo wifi: W2c: no room for %u pages at %#llx in the Wi-Fi's addresses\n", pages, at);
		return false;
	}
	for (uint32_t page = 0; page < pages; page++) {
		uint64_t address = at + 4096ull * page;
		uint32_t table = (uint32_t)((address - WIFI_ADDRESSES) >> 21);

		if (wsmmu.l3[table] == 0 && kmem_alloc(kernel_map, &wsmmu.l3[table], 4096, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
		    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
			wsmmu.l3[table] = 0;
			say("apollo wifi: W2c: no memory for a page table\n");
			return false;
		}
		if (first_table == ~0u) {
			first_table = table;
		}
		last_table = table;
		second[address >> 21 & 0x1ff] = kvtophys(wsmmu.l3[table]) | 3;
		((uint64_t *)wsmmu.l3[table])[address >> 12 & 0x1ff] =
		    (physical ? physical + 4096ull * page : kvtophys(buffer + 4096ull * page)) | attributes;
	}
	cache_flush((vm_offset_t)second, 4096);
	for (uint32_t table = first_table; first_table != ~0u && table <= last_table; table++) {      /* only the tables this call touched */
		cache_flush(wsmmu.l3[table], 4096);
	}
	wsmmu.next = at + 4096ull * pages;
	wr(cb + SMMU_CB_TLBIASID, wsmmu.context);
	wr(cb + SMMU_CB_TLBSYNC, 1);
	for (unsigned ms = 0; rd(cb + SMMU_CB_TLBSTATUS) & 1; ms++) {
		if (ms == 100) {
			say("apollo wifi: W2c: the SMMU does not finish forgetting old translations\n");
			return false;
		}
		sleep_ms(1);
	}
	*iova = at;
	return true;
}

/* one at a time: the Wi-Fi's page tables are written from W2c's thread and from apollo_wifi_dma_map()'s callers */
static bool
wsmmu_map_as(vm_offset_t buffer, uint64_t physical, uint32_t bytes, uint64_t attributes, uint64_t *iova)
{
	bool ok;

	lck_mtx_lock(&wifi_map_lock);
	ok = wsmmu_map_as_locked(buffer, physical, bytes, attributes, iova);
	lck_mtx_unlock(&wifi_map_lock);
	return ok;
}

/* pages of a kernel buffer, as normal memory (MAIR index 0) */
static bool
wsmmu_map(vm_offset_t buffer, uint32_t bytes, uint64_t *iova)
{
	return wsmmu_map_as(buffer, 0, bytes, WIFI_PTE, iova);
}

static void
mhi_write64(vm_offset_t low_register, uint64_t value)
{
	if (!wifi_link_alive) {
		return;                         /* (power down has taken the link away: no BAR write) */
	}
	__asm__ volatile ("dsb sy");            /* the (uncached) ring and context stores are visible before the doorbell */
	wr(bar + low_register + 4, (uint32_t)(value >> 32));
	wr(bar + low_register, (uint32_t)value);
}

/* In interrupt context: count it, wake W2c (the 10 ms timeout of w2c_wait() covers a wakeup that comes before the wait) */
static void
w2c_msi_arrived(__unused void *context, uint32_t intid)
{
	if (intid >= W2C_MSI_INTID && intid < W2C_MSI_INTID + W2C_MSI_VECTORS) {
		__atomic_fetch_add(&w2c_hits[intid - W2C_MSI_INTID], 1, __ATOMIC_RELAXED);
	}
	thread_wakeup((event_t)&w2c_wake);
}

/* Wait for an interrupt, 10 ms at most (the polls of W2c do not depend on one arriving) */
static void
w2c_wait(void)
{
	if (!w2c.msi_on) {
		sleep_ms(10);
		return;
	}
	assert_wait_timeout((event_t)&w2c_wake, THREAD_UNINT, 10, NSEC_PER_MSEC);
	thread_block(THREAD_CONTINUE_NULL);
}

/*
 * The chip's MSI: the doorbell's page in the Wi-Fi's SMMU context (device memory), INTIDs 800 to 802 attached, then the MSI capability
 * (conf + the list from 0x34) written: address = the mapped doorbell + 0x40, data = 800, MME = what MMC allows (up to 32), enabled; INTx
 * off in the command register. Not fatal when it fails: W2c polls as before.
 */
static bool
w2c_msi_setup(void)
{
	uint32_t cap = rd(conf + 0x34) & 0xfc, word = 0, control, mmc, vectors, data_at, command;
	uint64_t iova = 0, address;
	bool wide;

	for (unsigned hops = 0; cap != 0 && hops < 48; hops++) {
		word = rd(conf + cap);
		if ((word & 0xff) == PCI_CAP_MSI) {
			break;
		}
		cap = word >> 8 & 0xfc;
	}
	if (cap == 0 || (word & 0xff) != PCI_CAP_MSI) {
		say("apollo wifi: W2c: no MSI capability in the chip's list (it starts at %#x): polling only\n", rd(conf + 0x34) & 0xfc);
		return false;
	}
	control = word >> 16;
	mmc = control >> 1 & 7;
	wide = (control & 0x80) != 0;
	vectors = 1u << mmc;
	say("apollo wifi: W2c: MSI capability at conf+%#x: control %#x (%s, up to %u vectors%s, %s), address %#x\n", cap, control,
	    (control & 1) ? "enabled" : "disabled", vectors, (control & 0x100) ? ", per-vector masks" : "", wide ? "64-bit" : "32-bit address", rd(conf + cap + 4));
	if (vectors < W2C_MSI_VECTORS) {
		say("apollo wifi: W2c: the chip allows %u vectors, MHI needs %u: polling only\n", vectors, W2C_MSI_VECTORS);
		return false;
	}
	if (mmc > W2C_MSI_WANT_LOG2) {
		mmc = W2C_MSI_WANT_LOG2;
		vectors = 1u << mmc;
	}
	if (!wsmmu_map_as(0, W2C_MSI_DOORBELL, 4096, WIFI_PTE | PTE_DEVICE, &iova)) {
		return false;
	}
	address = iova + W2C_MSI_SETSPI;
	if (!wide && (address >> 32)) {
		say("apollo wifi: W2c: the chip's MSI address is 32 bits, the doorbell's address %#llx is not: polling only\n", address);
		return false;
	}
	say("apollo wifi: W2c: the MSI doorbell (%#llx) is mapped at %#llx (device memory); the chip gets address %#llx, data %u (INTID of vector 0), %u vectors\n",
	    W2C_MSI_DOORBELL + W2C_MSI_SETSPI, iova, address, W2C_MSI_INTID, vectors);
	w2b_hold("the GIC distributor takes INTIDs 800 to 802 (edge), then the chip's MSI capability and command register are written");
	for (unsigned n = 0; n < W2C_MSI_VECTORS; n++) {
		kern_return_t kr = gic_v3_spi_attach(W2C_MSI_INTID + n, true, w2c_msi_arrived, NULL);

		if (kr != KERN_SUCCESS) {
			say("apollo wifi: W2c: INTID %u is not available (%d): polling only\n", W2C_MSI_INTID + n, kr);
			for (unsigned m = 0; m < n; m++) {
				gic_v3_spi_detach(W2C_MSI_INTID + m);
			}
			return false;
		}
	}
	w2c.msi_attached = true;
	w2c.msi_address = address;
	data_at = cap + (wide ? 0xc : 0x8);
	wr(conf + cap + 4, (uint32_t)address);
	if (wide) {
		wr(conf + cap + 8, (uint32_t)(address >> 32));
	}
	wr(conf + data_at, W2C_MSI_INTID);
	if (control & 0x100) {
		wr(conf + data_at + 4, 0);      /* the mask bits (per-vector masking): all vectors open */
	}
	command = rd(conf + 0x04);
	wr(conf + 0x04, (command & 0xffff) | 1u << 10);         /* INTx disable (the status half is write-1-to-clear: left out) */
	control = (control & ~(7u << 4)) | mmc << 4 | 1;        /* MME and enable */
	w2c.msi_cap = cap;
	wr(conf + cap, (word & 0xffff) | control << 16);
	w2c.msi_on = true;
	say("apollo wifi: W2c: MSI enabled: control %#x, address %#x %#x, data %#x, command %#x\n", rd(conf + cap) >> 16, rd(conf + cap + 4),
	    wide ? rd(conf + cap + 8) : 0, rd(conf + data_at) & 0xffff, rd(conf + 0x04) & 0xffff);
	return true;
}

/* The three INTIDs masked and the chip's MSI disabled; no lock, no sleep, no log: a panic's way too (quiesce) */
static void
w2c_msi_mask(void)
{
	uint32_t cap = w2c.msi_cap;

	if (w2c.msi_attached) {
		for (unsigned n = 0; n < W2C_MSI_VECTORS; n++) {
			gic_v3_spi_mask(W2C_MSI_INTID + n);
		}
	}
	if (cap != 0 && conf != 0 && !w2c.msi_cap_skip) {
		wr(conf + cap, rd(conf + cap) & ~(1u << 16));
	}
	w2c.msi_on = false;
}

/* A failed W2c: masked, and the INTIDs given back */
static void
w2c_msi_stop(void)
{
	w2c_msi_mask();
	if (w2c.msi_attached) {
		w2c.msi_attached = false;
		for (unsigned n = 0; n < W2C_MSI_VECTORS; n++) {
			gic_v3_spi_detach(W2C_MSI_INTID + n);
		}
	}
}

/* One element, decoded as Linux's mhi_process_ctrl_ev_ring() does (drivers/bus/mhi/host/main.c) */
static void
w2c_event_log(unsigned ring, uint64_t index, const volatile uint32_t *tre)
{
	uint32_t type = tre[3] >> 16 & 0xff, code = tre[2] >> 24 & 0xff;       /* MHI_TRE_GET_EV_TYPE, _STATE, _EXECENV, _CODE */
	uint64_t ptr = (uint64_t)tre[1] << 32 | tre[0];

	/* STATE_CHANGE and EE events are always told (the power states matter); the other kinds stop at W2C_EVENT_LOG_MOST */
	if (type != W2C_PKT_STATE_CHANGE && type != W2C_PKT_EE) {
		if (w2c.dumped >= W2C_EVENT_LOG_MOST) {
			return;
		}
		if (++w2c.dumped == W2C_EVENT_LOG_MOST) {
			say("apollo wifi: W2c: (the event log of the other kinds stops after this one)\n");
		}
	}
	switch (type) {
	case W2C_PKT_STATE_CHANGE:
		say("apollo wifi: W2c: ring %u element %llu: STATE_CHANGE, new MHI state %u\n", ring, index, code);
		break;
	case W2C_PKT_EE:
		say("apollo wifi: W2c: ring %u element %llu: EE, new execution environment %u (%s)\n", ring, index, code, execution_environment(code));
		break;
	case W2C_PKT_CMD_COMPLETION:
		say("apollo wifi: W2c: ring %u element %llu: CMD_COMPLETION, code %u, command element at %#llx\n", ring, index, code, ptr);
		break;
	default:
		say("apollo wifi: W2c: ring %u element %llu: type %#x: %08x %08x %08x %08x\n", ring, index, type, tre[0], tre[1], tre[2], tre[3]);
		break;
	}
}

/* The doorbell of channel (or doorbell number) n; defined with IPCR below */
static void ipcr_channel_db(unsigned n, uint64_t value);

/*
 * mhi_pm_m0_transition() (drivers/bus/mhi/host/pm.c), on the STATE_CHANGE event to M0: wake_get(force), in mission mode every event
 * ring's doorbell (mhi_ring_er_db) and the command ring's if it is not empty, the doorbells of the channels with something on their
 * ring (channel contexts: the chip's rp behind the host's wp), wake_put. wifi_ev_lock is held; no other lock.
 */
static void
w2c_pm_m0(void)
{
	struct w2c_context *chan = (struct w2c_context *)(w2c.ctrl + W2C_OFF_CHAN);
	struct w2c_context *cmd = (struct w2c_context *)(w2c.ctrl + W2C_OFF_CMD);

	w2c_m2 = false;
	apollo_wifi_wake(1);
	if (rd(bhi_base + BHI_EXECENV) == MHI_EE_AMSS) {        /* MHI_IN_MISSION_MODE(ee) */
		for (unsigned ring = 0; ring < W2C_EVENT_RINGS; ring++) {
			mhi_write64(w2c.erdb + 8 * ring, w2c.wp[ring]);
		}
		if (ipcr_cmd_pending()) {               /* mhi_cmd->ring.rp != wp */
			mhi_write64(MHI_CRDB_LOWER, cmd->wp);
		}
	}
	for (unsigned n = IPCR_UL; n <= IPCR_DL; n++) {
		if (ipcr_up() && chan[n].rp != chan[n].wp) {
			ipcr_channel_db(n, chan[n].wp);
		}
	}
	apollo_wifi_wake(0);
}

/*
 * mhi_pm_m1_transition() (pm.c), on the STATE_CHANGE event to M1: Linux sets M2 (mhi_set_mhi_state: MHICTRL MHISTATE) and, when
 * dev_wake or pending packets are there, leaves M2 at once (wake_get(force) + wake_put(force)). Ours does that decision up front:
 * wake held (the permanent assertion of mhi_wake_hold): no M2, the wake doorbell is rung again (wake_get(force)'s doorbell); not
 * held: MHICTRL = M2.
 */
static void
w2c_pm_m1(void)
{
	if (mhi_wake_is_held()) {
		mhi_wake_reassert();
		say("apollo wifi: W2c: M1 with the wake held: wake doorbell rung again, no M2\n");
		return;
	}
	wr(bar + MHI_CTRL, (rd(bar + MHI_CTRL) & ~(0xffu << MHICTRL_STATE_SHIFT)) | MHI_STATE_M2 << MHICTRL_STATE_SHIFT);
	w2c_m2 = true;
	say("apollo wifi: W2c: M1: MHI state M2 requested (no wake held)\n");
}

/*
 * The event rings (ring 0 control, ring 1 data), consumed as Linux's mhi_process_ctrl_ev_ring() does: the chip's rp from the event
 * context, the host's own rp up to it, each element told and recycled (mhi_recycle_ev_ring_element(): wp and rp on by one element,
 * the context's wp written), the doorbell rung with the wp on every pass (mhi_ring_er_db(), unconditional as in mhi_process_ctrl_ev_ring
 * and mhi_pm_m0_transition). The contexts and rings are the uncached alias (ctrl_alloc): no cache maintenance. Nothing is done once the
 * firmware has crashed (wifi_dead): no doorbell goes to a chip in RDDM.
 */
static void
w2c_events(void)
{
	struct w2c_context *contexts = (struct w2c_context *)(w2c.ctrl + W2C_OFF_EVENT);
	static const uint32_t offset[W2C_EVENT_RINGS] = { W2C_OFF_ER0, W2C_OFF_ER1 };
	static const uint32_t elements[W2C_EVENT_RINGS] = { W2C_ER0_ELEMENTS, W2C_ER1_ELEMENTS };

	if (w2c.ctrl == 0 || !w2c.rings || wifi_dead) {
		return;
	}
	lck_mtx_lock(&wifi_ev_lock);
	for (unsigned ring = 0; ring < W2C_EVENT_RINGS && !wifi_dead; ring++) {
		uint64_t base = w2c.ctrl_iova + offset[ring], length = (uint64_t)elements[ring] * W2C_ELEMENT, rp;
		unsigned count = 0;
		volatile uint64_t *context_rp = &contexts[ring].rp, *context_wp = &contexts[ring].wp;

		rp = *context_rp;
		while (rp != w2c.rp[ring]) {
			volatile uint32_t *tre;

			if (rp < base || rp >= base + length || (rp - base) % W2C_ELEMENT || count >= elements[ring]) {
				if (!w2c.outside) {
					w2c.outside = true;
					say("apollo wifi: W2c: event ring %u: the chip's rp %#llx is outside the ring (%#llx, %llu bytes) or the ring is full\n", ring, rp, base, length);
				}
				break;
			}
			tre = (volatile uint32_t *)(w2c.ctrl + offset[ring] + (w2c.rp[ring] - base));
			if (!ipcr_event(ring, (w2c.rp[ring] - base) / W2C_ELEMENT, tre)) {
				w2c_event_log(ring, (w2c.rp[ring] - base) / W2C_ELEMENT, tre);
			}
			/* mhi_recycle_ev_ring_element() */
			w2c.wp[ring] += W2C_ELEMENT;
			if (w2c.wp[ring] >= base + length) {
				w2c.wp[ring] = base;
			}
			*context_wp = w2c.wp[ring];
			if (ring == 0 && (tre[3] >> 16 & 0xff) == W2C_PKT_STATE_CHANGE) {      /* mhi_process_ctrl_ev_ring(): MHI_PKT_TYPE_STATE_CHANGE_EVENT */
				uint32_t state = tre[2] >> 24;

				if (state == MHI_STATE_M0) {
					w2c_pm_m0();
				} else if (state == MHI_STATE_M1) {
					w2c_pm_m1();
				} else if (state == MHI_STATE_SYS_ERR) {
					wifi_mark_dead("STATE_CHANGE event: SYS_ERR");
				}
			} else if (ring == 0 && (tre[3] >> 16 & 0xff) == W2C_PKT_EE && (tre[2] >> 24) == MHI_EE_RDDM) {
				wifi_mark_dead("EE event: RDDM");
			}
			w2c.rp[ring] += W2C_ELEMENT;
			if (w2c.rp[ring] >= base + length) {
				w2c.rp[ring] = base;
			}
			rp = *context_rp;
			count++;
			if (wifi_dead) {
				break;
			}
		}
		if (!wifi_dead) {
			mhi_write64(w2c.erdb + 8 * ring, w2c.wp[ring]);        /* mhi_ring_er_db(): every pass */
		}
	}
	lck_mtx_unlock(&wifi_ev_lock);
}

/* Read the execution environment and the MHI state; log a change (and each interrupt count's); the SMMU's faults and the event rings too */
static void
w2c_poll(uint64_t began, const char *phase)
{
	uint32_t ee = rd(bhi_base + BHI_EXECENV), mhi = rd(bar + MHI_STATUS);

	if (ee != w2c.ee || mhi != w2c.mhi) {
		say("apollo wifi: W2c: %s +%llu ms: execution environment %#x (%s), MHI state %u%s%s (MHISTATUS %#x), BHI status %#x\n", phase,
		    (now_us() - began) / 1000, ee, execution_environment(ee), mhi >> MHISTATUS_STATE_SHIFT & 0xff, (mhi & MHISTATUS_READY) ? ", ready" : "",
		    (mhi & MHISTATUS_SYSERR) ? ", SYSTEM ERROR" : "", mhi, rd(bhi_base + BHI_STATUS));
		w2c.ee = ee;
		w2c.mhi = mhi;
	}
	for (unsigned n = 0; n < W2C_MSI_VECTORS; n++) {
		uint32_t hits = w2c_hits[n];

		if (hits != w2c.seen[n]) {
			say("apollo wifi: W2c: %s +%llu ms: MSI vector %u: %u interrupts (was %u); execution environment %#x, MHI state %u\n", phase,
			    (now_us() - began) / 1000, n, hits, w2c.seen[n], ee, mhi >> MHISTATUS_STATE_SHIFT & 0xff);
			w2c.seen[n] = hits;
		}
	}
	wsmmu_faults(&w2c.told);
	diag_poll(phase, began);
	w2c_events();
}

/* The last word on a step that did not get anywhere: the diagnostics' registers, the MSI hit counts, the SMMU's fault status */
static void
wifi_final_dump(const char *why)
{
	uint32_t direct[DIAG_DIRECT], windowed[DIAG_WINDOWED];
	bool window = chip_prep != 0 && chip_window_ok();
	uint64_t now = now_us();

	say("apollo wifi: FINAL DUMP (%s)\n", why);
	if (bhi_base != 0) {
		diag_read(direct, window ? windowed : NULL);
		diag_tell("final", now, direct, window ? windowed : NULL);
	}
	say("apollo wifi: final: PCI command/status %#x, link %s (port debug %#x, link status %#x)\n", rd(conf + 0x04), link_up() ? "up" : "DOWN",
	    rd(dbi + DBI_PORT_DEBUG1), rd(dbi + DBI_LINK_CTRL_STATUS) >> 16);
	say("apollo wifi: final: MSI: attached %u, on %u, capability at conf+%#x (control %#x); hits per vector %u %u %u; GIC counts %llu %llu %llu\n",
	    w2c.msi_attached, w2c.msi_on, w2c.msi_cap, w2c.msi_cap ? rd(conf + w2c.msi_cap) >> 16 : 0, w2c_hits[0], w2c_hits[1], w2c_hits[2],
	    w2c.msi_attached ? gic_v3_spi_count(W2C_MSI_INTID) : 0, w2c.msi_attached ? gic_v3_spi_count(W2C_MSI_INTID + 1) : 0,
	    w2c.msi_attached ? gic_v3_spi_count(W2C_MSI_INTID + 2) : 0);
	if (wsmmu.on) {
		vm_offset_t cb = wsmmu_context(wsmmu.context);

		say("apollo wifi: final: SMMU: global status %#x (stream %#x); context %u: status %#x, address %#llx, syndrome %#x, sctlr %#x\n",
		    rd(wsmmu.base + SMMU_SGFSR), rd(wsmmu.base + SMMU_SGFSYNR1), wsmmu.context, rd(cb + SMMU_CB_FSR),
		    (uint64_t)rd(cb + SMMU_CB_FAR + 4) << 32 | rd(cb + SMMU_CB_FAR), rd(cb + SMMU_CB_FSYNR0), rd(cb + SMMU_CB_SCTLR));
	}
}

/* ---- RDDM: the firmware's crash dump (BHIe receive vector), drivers/bus/mhi/host/boot.c ---------------------------------- */

/*
 * ath11k_mhi_register(): mhi_ctrl->rddm_size = RDDM_DUMP_SIZE (0x420000) and seg_len = SZ_512K (drivers/net/wireless/ath/ath11k/mhi.c).
 *   mhi_alloc_bhie_table(rddm_size): DIV_ROUND_UP(0x420000, 512 KB) = 9 segments of 512 KB, plus the vector table as the last entry
 *     (16 bytes per segment: 64-bit address, 64-bit size; 9 * 16 = 144 bytes)
 *   mhi_rddm_prepare() (mhi_prepare_for_power_up(), before the chip is powered up): the vector table's address to RXVECADDR_HIGH/LOW, its size
 *     to RXVECSIZE, a sequence id (non-zero, 30 bits) to RXVECDB
 *   mhi_download_rddm_image(): when the chip's EE becomes RDDM, wait for RXVECSTATUS[31:30] == XFER_COMPL (2) (mhi_cntrl->timeout_ms; the
 *     DT's mhi,timeout is 10000)
 * Ours: the nine segments are one buffer (contiguous in the chip's addresses, as the image of W2c); boot-arg apollo_wifi_rddm (default 1).
 */
#define BHIE_RXVECADDR_LOW_OFF  0x60
#define BHIE_RXVECADDR_HIGH_OFF 0x64
#define BHIE_RXVECSIZE_OFF      0x68
#define BHIE_RXVECDB_OFF        0x70
#define BHIE_RXVECSTATUS_OFF    0x78
#define RDDM_DUMP_SIZE          0x420000u
#define RDDM_SEGMENT            0x80000u        /* mhi_ctrl->seg_len = SZ_512K */
#define RDDM_SEGMENTS           ((RDDM_DUMP_SIZE + RDDM_SEGMENT - 1) / RDDM_SEGMENT)
#define RDDM_WAIT_US            10000000ull     /* mhi,timeout */
#define RDDM_RUN_MIN            12              /* printable run, characters */
#define RDDM_LINES              60
#define RDDM_LINE_MAX           200

static uint32_t wifi_rddm = 1;                  /* boot-arg apollo_wifi_rddm */
static struct {
	vm_offset_t buffer, table;              /* RDDM_SEGMENTS * RDDM_SEGMENT bytes; one page with the vector table */
	uint64_t buffer_iova, table_iova;
	bool armed, collected;
} rddm;

/* mhi_alloc_bhie_table() and mhi_rddm_prepare(): true when the receive vector is programmed */
static bool
w2c_rddm_prepare(void)
{
	struct rddm_vec {
		uint64_t address, size;
	} *vector;
	uint32_t offset, sequence;
	vm_offset_t base;

	if (!wifi_rddm) {
		return false;
	}
	offset = rd(bar + MHI_BHIE_OFF);
	if (offset == 0 || offset + BHIE_RXVECSTATUS_OFF + 4 > bar_mapped) {
		say("apollo wifi: RDDM: BHIe offset %#x out of range: no crash dump\n", offset);
		return false;
	}
	base = bar + offset;
	if (rddm.buffer == 0) {
		if (kmem_alloc(kernel_map, &rddm.buffer, (vm_size_t)RDDM_SEGMENTS * RDDM_SEGMENT, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS ||
		    kmem_alloc(kernel_map, &rddm.table, 4096, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
			say("apollo wifi: RDDM: no memory for the dump (%u KB): no crash dump\n", RDDM_SEGMENTS * RDDM_SEGMENT / 1024);
			rddm.buffer = rddm.table = 0;
			return false;
		}
		cache_flush(rddm.buffer, (size_t)RDDM_SEGMENTS * RDDM_SEGMENT);
		if (!wsmmu_map(rddm.buffer, RDDM_SEGMENTS * RDDM_SEGMENT, &rddm.buffer_iova) || !wsmmu_map(rddm.table, 4096, &rddm.table_iova)) {
			say("apollo wifi: RDDM: could not map the dump buffer: no crash dump\n");
			rddm.buffer = rddm.table = 0;
			return false;
		}
		vector = (struct rddm_vec *)rddm.table;
		for (uint32_t n = 0; n < RDDM_SEGMENTS; n++) {
			vector[n].address = rddm.buffer_iova + (uint64_t)n * RDDM_SEGMENT;
			vector[n].size = RDDM_SEGMENT;
		}
		cache_flush(rddm.table, 4096);
	}
	sequence = ((uint32_t)mach_absolute_time() & BHIE_SEQ_MASK) | 1;
	wr(base + BHIE_RXVECADDR_HIGH_OFF, (uint32_t)(rddm.table_iova >> 32));
	wr(base + BHIE_RXVECADDR_LOW_OFF, (uint32_t)rddm.table_iova);
	wr(base + BHIE_RXVECSIZE_OFF, RDDM_SEGMENTS * 16);
	wr(base + BHIE_RXVECDB_OFF, (rd(base + BHIE_RXVECDB_OFF) & ~BHIE_SEQ_MASK) | sequence);   /* mhi_write_reg_field() */
	rddm.armed = true;
	say("apollo wifi: RDDM: receive vector armed at BHIe+%#x: %u segments of %#x (%u KB) at %#llx, vector table (%u bytes) at %#llx, sequence %u\n",
	    BHIE_RXVECADDR_LOW_OFF, RDDM_SEGMENTS, RDDM_SEGMENT, RDDM_SEGMENTS * RDDM_SEGMENT / 1024, rddm.buffer_iova, RDDM_SEGMENTS * 16,
	    rddm.table_iova, sequence);
	return true;
}

static bool
rddm_has(const char *run, size_t length, const char *needle)
{
	size_t n = strlen(needle);

	for (size_t i = 0; i + n <= length; i++) {
		if (memcmp(run + i, needle, n) == 0) {
			return true;
		}
	}
	return false;
}

/*
 * The firmware's own image (fw/amss.bin): a run found there word for word is its read-only data (format strings, file
 * names), not what it wrote when it failed. The first dump (25 October) printed 40 of those and nothing else.
 */
static const uint8_t *rddm_image;
static uint64_t rddm_image_size;

static bool
rddm_in_image(const char *run, size_t length)
{
	if (rddm_image == NULL || length < 8) {
		return false;
	}
	for (uint64_t i = 0; i + length <= rddm_image_size; i++) {
		if (rddm_image[i] == (uint8_t)run[0] && memcmp(rddm_image + i, run, length) == 0) {
			return true;
		}
	}
	return false;
}

/*
 * The dump's printable runs (at least RDDM_RUN_MIN) with ".c" or an error word, and no '%', that are not in the firmware's image.
 * Bounded: at most RDDM_IMAGE_BUDGET comparisons against the image (each one a pass over 4 MB; after that the candidates are
 * printed unchecked), RDDM_SCAN_US of wall-clock time for the whole scan, and a short sleep every RDDM_YIELD bytes scanned (no lock
 * is held: the event thread and W2c's thread both call this with nothing but wifi_lock, a mutex, at most). The 25 October runs that
 * compared every run of the dump against the image froze the kernel.
 */
#define RDDM_IMAGE_BUDGET       300
#define RDDM_SCAN_US            1000000ull
#define RDDM_YIELD              (256u * 1024)

static unsigned
rddm_scan(const uint8_t *dump, size_t size, uint32_t *seen)
{
	char text[RDDM_LINE_MAX + 1];
	size_t i = 0, yielded = 0;
	unsigned printed = 0, compared = 0, unchecked = 0;
	uint64_t began = now_us();

	while (i < size && printed < RDDM_LINES) {
		size_t start = i, length;
		bool dot_c, word;
		uint32_t hash = 5381;

		while (i < size && dump[i] >= 0x20 && dump[i] < 0x7f) {
			i++;
		}
		length = i - start;
		i++;
		if (i - yielded >= RDDM_YIELD) {
			yielded = i;
			sleep_ms(1);
			if (now_us() - began > RDDM_SCAN_US) {
				say("apollo wifi: RDDM: the scan stops at %#zx of %#zx after %llu ms (time cap)\n", i, size, (now_us() - began) / 1000);
				break;
			}
		}
		if (length < RDDM_RUN_MIN) {
			continue;
		}
		if (length > RDDM_LINE_MAX) {
			length = RDDM_LINE_MAX;
		}
		memcpy(text, dump + start, length);
		text[length] = 0;
		dot_c = rddm_has(text, length, ".c");
		word = rddm_has(text, length, "err") || rddm_has(text, length, "ERR") || rddm_has(text, length, "assert") ||
		    rddm_has(text, length, "ASSERT") || rddm_has(text, length, "fatal") || rddm_has(text, length, "FATAL") || rddm_has(text, length, "fail");
		if (!(dot_c || word) || rddm_has(text, length, "%")) {
			continue;
		}
		if (now_us() - began > RDDM_SCAN_US) {
			say("apollo wifi: RDDM: the scan stops at %#zx of %#zx after %llu ms (time cap)\n", start, size, (now_us() - began) / 1000);
			break;
		}
		if (compared < RDDM_IMAGE_BUDGET) {
			compared++;
			if (rddm_in_image(text, length)) {
				continue;
			}
		} else {
			unchecked++;
		}
		for (size_t k = 0; k < length; k++) {
			hash = hash * 33 + (uint8_t)text[k];
		}
		for (unsigned k = 0; k < printed; k++) {
			if (seen[k] == hash) {
				hash = 0;
				break;
			}
		}
		if (hash == 0) {
			continue;
		}
		seen[printed++] = hash;
		say("apollo wifi: RDDM: @%#zx: %s\n", start, text);
	}
	say("apollo wifi: RDDM: scan done in %llu ms: %u image comparisons, %u candidates printed unchecked\n", (now_us() - began) / 1000, compared, unchecked);
	return printed;
}

/* mhi_download_rddm_image() and the dump's crash reason, once: wait for the chip's EE RDDM and the vector's completion, then scan the buffer */
static void
wifi_rddm_collect(const char *why)
{
	uint64_t began = now_us();
	uint32_t ee = ~0u, status = 0, offset = rd(bar + MHI_BHIE_OFF);
	vm_offset_t base = bar + offset;
	const uint8_t *dump = (const uint8_t *)rddm.buffer;
	uint32_t seen[RDDM_LINES + 1];
	size_t size = (size_t)RDDM_SEGMENTS * RDDM_SEGMENT, extent = 0;
	unsigned printed;

	if (!wifi_rddm || !rddm.armed || rddm.collected || offset == 0 || offset + BHIE_RXVECSTATUS_OFF + 4 > bar_mapped) {
		return;
	}
	rddm.collected = true;
	say("apollo wifi: RDDM: collecting (%s)\n", why);
	while (now_us() - began < 5000000ull) {         /* the firmware enters RDDM by itself after its SYS_ERR */
		if (ipcr_stopping() || !wifi_link_alive) {
			say("apollo wifi: RDDM: power down: the collection is given up\n");
			return;
		}
		ee = rd(bhi_base + BHI_EXECENV);
		if (ee == MHI_EE_RDDM || ee == ~0u) {
			break;
		}
		sleep_ms(10);
	}
	say("apollo wifi: RDDM: execution environment %#x (%s) after %llu ms\n", ee, execution_environment(ee), (now_us() - began) / 1000);
	if (ee != MHI_EE_RDDM) {
		return;
	}
	for (began = now_us(); now_us() - began < RDDM_WAIT_US; sleep_ms(10)) {
		if (ipcr_stopping() || !wifi_link_alive) {
			say("apollo wifi: RDDM: power down: the collection is given up\n");
			return;
		}
		status = rd(base + BHIE_RXVECSTATUS_OFF);
		if (status == ~0u || (status >> 30) == BHIE_STATUS_XFER_COMPL || (status >> 30) == BHIE_STATUS_ERROR) {
			break;
		}
	}
	if (ipcr_stopping() || !wifi_link_alive) {
		return;
	}
	say("apollo wifi: RDDM: RXVECSTATUS %#x (status %u, sequence %u) after %llu ms; BHI error %#x, debug %#x %#x\n", status, status >> 30,
	    status & BHIE_SEQ_MASK, (now_us() - began) / 1000, rd(bhi_base + BHI_ERRCODE), rd(bhi_base + BHI_ERRDBG1), rd(bhi_base + BHI_ERRDBG2));
	if ((status >> 30) != BHIE_STATUS_XFER_COMPL) {
		return;
	}
	cache_flush(rddm.buffer, size);
	for (size_t o = size; o > 0; o--) {
		if (dump[o - 1] != 0) {
			extent = o;
			break;
		}
	}
	say("apollo wifi: RDDM: dump buffer %zu bytes (%u segments of %#x) at IOVA %#llx; the last non-zero byte is at %#zx (%zu bytes)\n", size,
	    RDDM_SEGMENTS, RDDM_SEGMENT, rddm.buffer_iova, extent ? extent - 1 : 0, extent);
	say("apollo wifi: RDDM: first bytes: %08x %08x %08x %08x %08x %08x %08x %08x\n", ((const uint32_t *)dump)[0], ((const uint32_t *)dump)[1],
	    ((const uint32_t *)dump)[2], ((const uint32_t *)dump)[3], ((const uint32_t *)dump)[4], ((const uint32_t *)dump)[5],
	    ((const uint32_t *)dump)[6], ((const uint32_t *)dump)[7]);
	{
		int disk = 0;

		rddm_image = hakios_image_file("fw", "amss.bin", &rddm_image_size, &disk);
	}
	printed = rddm_scan(dump, extent, seen);
	say("apollo wifi: RDDM: %u crash-reason candidates printed (printable runs of %u or more characters)\n", printed, RDDM_RUN_MIN);
}

/* amss.bin and the control pages in memory, mapped for the chip; the contexts and rings filled as Linux's mhi_init_dev_ctxt() does */
static bool
w2c_prepare(void)
{
	const uint8_t *image, *data;
	uint64_t size = 0, remaining;
	uint32_t segments;
	struct w2c_context *chan, *event, *cmd;
	struct bhi_vec {
		uint64_t address, size;
	} *vector;
	int disk = -1;
	uint64_t ctrl_physical = 0;

	image = hakios_image_file("fw", "amss.bin", &size, &disk);
	if (image == NULL || size < W2C_SEGMENT) {
		say("apollo wifi: W2c: no fw/amss.bin in the small image (%llu bytes)\n", size);
		return false;
	}
	/* mhi_fw_load_handler(): a second ELF header after the SBL's sbl_size means: the chip gets only what is behind it */
	data = image;
	if (size > W2C_SEGMENT + 4 && memcmp(image + W2C_SEGMENT, "\x7f" "ELF", 4) == 0) {
		data += W2C_SEGMENT;
		size -= W2C_SEGMENT;
		say("apollo wifi: W2c: a second ELF header after the first %#x bytes: only what follows goes over BHIe\n", W2C_SEGMENT);
	}
	segments = (uint32_t)((size + W2C_SEGMENT - 1) / W2C_SEGMENT);
	if (segments * sizeof(struct bhi_vec) > 4096) {
		say("apollo wifi: W2c: %u segments do not fit one page of vector table\n", segments);
		return false;
	}
	w2c.segments = segments;
	if (kmem_alloc(kernel_map, &w2c.image, (vm_size_t)segments * W2C_SEGMENT, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS ||
	    (w2c.ctrl = ctrl_alloc(W2C_CTRL_PAGES * 4096, &ctrl_physical)) == 0) {
		say("apollo wifi: W2c: no memory for the image (%u KB) and the control pages (%u KB in one piece)\n", segments * W2C_SEGMENT / 1024, W2C_CTRL_PAGES * 4);
		w2c.image = w2c.ctrl = 0;
		return false;
	}
	memcpy((void *)w2c.image, data, size);
	cache_flush(w2c.image, (size_t)size);
	if (!wsmmu_map_as(0, ctrl_physical, W2C_CTRL_PAGES * 4096, WIFI_PTE, &w2c.ctrl_iova) || !wsmmu_map(w2c.image, segments * W2C_SEGMENT, &w2c.image_iova)) {
		w2c.ctrl = 0;
		return false;
	}
	/* the vector table: mhi_firmware_copy_bhie(): address and size of each segment (ours are contiguous in the chip's addresses) */
	vector = (struct bhi_vec *)(w2c.ctrl + W2C_OFF_VECTOR);
	remaining = size;
	for (uint32_t n = 0; n < segments; n++) {
		vector[n].address = w2c.image_iova + (uint64_t)n * W2C_SEGMENT;
		vector[n].size = remaining < W2C_SEGMENT ? remaining : W2C_SEGMENT;
		remaining -= vector[n].size;
	}
	/* channel contexts: all zero (disabled) but ath11k's two, IPCR 20 (to the device) and 21 (from it), on event ring 1, burst mode off */
	chan = (struct w2c_context *)(w2c.ctrl + W2C_OFF_CHAN);
	chan[20].word0 = W2C_BRST_DISABLE << 8;
	chan[20].word1 = W2C_TO_DEVICE;
	chan[20].word2 = 1;
	chan[21].word0 = W2C_BRST_DISABLE << 8;
	chan[21].word1 = W2C_FROM_DEVICE;
	chan[21].word2 = 1;
	/* event contexts: intmod (ms) in [31:16], ertype valid, the MSI vector number; ring base, length, rp = wp = base */
	event = (struct w2c_context *)(w2c.ctrl + W2C_OFF_EVENT);
	event[0].word0 = 0u << 16;
	event[0].word1 = W2C_ER_TYPE_VALID;
	event[0].word2 = 1;
	event[0].rbase = event[0].rp = event[0].wp = w2c.ctrl_iova + W2C_OFF_ER0;
	event[0].rlen = W2C_ER0_ELEMENTS * W2C_ELEMENT;
	event[1].word0 = 1u << 16;
	event[1].word1 = W2C_ER_TYPE_VALID;
	event[1].word2 = 2;
	event[1].rbase = event[1].rp = event[1].wp = w2c.ctrl_iova + W2C_OFF_ER1;
	event[1].rlen = W2C_ER1_ELEMENTS * W2C_ELEMENT;
	w2c.rp[0] = event[0].rp;                /* mhi_init_event_ctxt(): rp = wp = base, until the ready transition moves the wp */
	w2c.rp[1] = event[1].rp;
	cmd = (struct w2c_context *)(w2c.ctrl + W2C_OFF_CMD);
	cmd->rbase = cmd->rp = cmd->wp = w2c.ctrl_iova + W2C_OFF_CMD_RING;
	cmd->rlen = W2C_CMD_ELEMENTS * W2C_ELEMENT;
	__asm__ volatile ("dsb sy");            /* (the control pages are the uncached alias: nothing to flush) */
	say("apollo wifi: W2c: amss.bin: %llu bytes in %u segments of %#x at %#llx; control pages (%u KB) at %#llx: channel contexts +%#x, "
	    "event contexts +%#x, command context +%#x, command ring +%#x, event rings +%#x and +%#x, vector table +%#x; next free address %#llx\n",
	    size, segments, W2C_SEGMENT, w2c.image_iova, W2C_CTRL_PAGES * 4, w2c.ctrl_iova, W2C_OFF_CHAN, W2C_OFF_EVENT, W2C_OFF_CMD,
	    W2C_OFF_CMD_RING, W2C_OFF_ER0, W2C_OFF_ER1, W2C_OFF_VECTOR, wsmmu.next);
	return true;
}

/* mhi_ready_state_transition(), mhi_init_mmio(), M0: until the chip's MHI is in M0 */
static bool
w2c_ready_m0(void)
{
	uint64_t began = now_us();
	uint32_t mhi = ~0u, ctrl = ~0u, offset, cfg, ee, window_unreadable = 0;
	uint64_t window_start = WIFI_ADDRESSES, window_stop = WIFI_ADDRESSES + WIFI_POOL_SIZE - 1;

	/* 1. MHISTATUS readable (all ones: the chip's MHI side is down; Linux's "link down" test) */
	while (now_us() - began < W2C_WAIT_US) {
		mhi = rd(bar + MHI_STATUS);
		if (mhi != ~0u) {
			break;
		}
		window_unreadable++;
		sleep_ms(10);
	}
	if (mhi == ~0u) {
		say("apollo wifi: W2c: MHISTATUS unreadable (all ones) for 20 s\n");
		return false;
	}
	say("apollo wifi: W2c: MHISTATUS readable after %llu ms (%u reads all ones): %#x\n", (now_us() - began) / 1000, window_unreadable, mhi);
	wr(bhi_base + BHI_INTVEC, 0);           /* mhi_async_power_up(): the chip clears it when it resets; 0: MHI's own interrupt is MSI vector 0 */
	/* 2. RESET cleared, then READY, by the chip (polled every 10 ms; Linux: 25 ms) */
	w2c.ee = w2c.mhi = 0x12345678;
	for (began = now_us(); now_us() - began < W2C_WAIT_US; w2c_wait()) {
		ctrl = rd(bar + MHI_CTRL);
		mhi = rd(bar + MHI_STATUS);
		w2c_poll(began, "waiting for READY");
		if (ctrl != ~0u && mhi != ~0u && !(ctrl & MHICTRL_RESET) && (mhi & MHISTATUS_READY)) {
			break;
		}
	}
	if (ctrl == ~0u || mhi == ~0u || (ctrl & MHICTRL_RESET) || !(mhi & MHISTATUS_READY)) {
		say("apollo wifi: W2c: THE CHIP DID NOT BECOME READY in 20 s: MHICTRL %#x, MHISTATUS %#x\n", ctrl, mhi);
		wifi_final_dump("W2c: READY not reached");
		return false;
	}
	say("apollo wifi: W2c: MHI READY after %llu ms (MHICTRL %#x, MHISTATUS %#x)\n", (now_us() - began) / 1000, ctrl, mhi);
	if (!w2c_prepare()) {
		return false;
	}
	w2c_rddm_prepare();                     /* mhi_prepare_for_power_up(): before the chip runs on (not fatal when it fails) */
	/* mhi_init_mmio(): the register values come from the contexts and from the window */
	offset = rd(bar + MHI_ERDBOFF);
	if (offset == 0 || offset == ~0u || (uint64_t)offset + 8 * W2C_EVENT_RINGS > bar_mapped) {         /* (a doorbell of 8 bytes per event ring) */
		say("apollo wifi: W2c: ERDBOFF %#x is out of range\n", offset);
		return false;
	}
	say("apollo wifi: W2c: MHI registers: channel contexts %#llx, event contexts %#llx, command context %#llx, window %#llx to %#llx, "
	    "%u event rings (doorbells at +%#x), 0 hardware event rings\n", w2c.ctrl_iova + W2C_OFF_CHAN, w2c.ctrl_iova + W2C_OFF_EVENT,
	    w2c.ctrl_iova + W2C_OFF_CMD, window_start, window_stop, W2C_EVENT_RINGS, offset);
	w2b_hold("the MHI context registers (first MMIO writes of W2c: CCABAP, ECABAP, CRCBAP, window, MHICFG)");
	mhi_write64(MHI_CCABAP_LOWER, w2c.ctrl_iova + W2C_OFF_CHAN);
	mhi_write64(MHI_ECABAP_LOWER, w2c.ctrl_iova + W2C_OFF_EVENT);
	mhi_write64(MHI_CRCBAP_LOWER, w2c.ctrl_iova + W2C_OFF_CMD);
	mhi_write64(MHI_CTRLBASE_LOWER, window_start);
	mhi_write64(MHI_DATABASE_LOWER, window_start);
	mhi_write64(MHI_CTRLLIMIT_LOWER, window_stop);
	mhi_write64(MHI_DATALIMIT_LOWER, window_stop);
	cfg = rd(bar + MHI_CFG);
	cfg &= ~(0xffu << MHICFG_NER_SHIFT | 0xffu << MHICFG_NHWER_SHIFT);
	cfg |= W2C_EVENT_RINGS << MHICFG_NER_SHIFT;
	wr(bar + MHI_CFG, cfg);
	say("apollo wifi: W2c: MHICFG now %#x\n", rd(bar + MHI_CFG));
	/* mhi_ready_state_transition(): each software event ring's write pointer to its last element, and its doorbell */
	{
		struct w2c_context *event = (struct w2c_context *)(w2c.ctrl + W2C_OFF_EVENT);

		for (unsigned ring = 0; ring < W2C_EVENT_RINGS; ring++) {
			event[ring].wp = w2c.wp[ring] = event[ring].rbase + event[ring].rlen - W2C_ELEMENT;
		}
		w2c.erdb = offset;
		w2c.chdb = rd(bar + MHI_CHDBOFF);       /* (checked where it is used: chdb_ok()) */
		for (unsigned ring = 0; ring < W2C_EVENT_RINGS; ring++) {
			mhi_write64(offset + 8 * ring, event[ring].wp);
		}
		w2c.rings = true;
	}
	/* 3. M0 */
	w2b_hold("MHI state M0 requested (MHICTRL)");
	ctrl = rd(bar + MHI_CTRL);
	ctrl &= ~(0xffu << MHICTRL_STATE_SHIFT);
	ctrl |= MHI_STATE_M0 << MHICTRL_STATE_SHIFT;
	wr(bar + MHI_CTRL, ctrl);
	say("apollo wifi: W2c: M0 requested (MHICTRL %#x)\n", ctrl);
	w2c.ee = w2c.mhi = 0x12345678;
	for (began = now_us(); now_us() - began < W2C_WAIT_US; w2c_wait()) {
		w2c_poll(began, "waiting for M0");
		mhi = w2c.mhi;
		if (mhi != ~0u && (mhi >> MHISTATUS_STATE_SHIFT & 0xff) == MHI_STATE_M0) {
			break;
		}
		if (mhi != ~0u && (mhi & MHISTATUS_SYSERR)) {
			say("apollo wifi: W2c: THE CHIP REPORTS A SYSTEM ERROR (MHISTATUS %#x, BHI error %#x, debug %#x %#x %#x)\n", mhi,
			    rd(bhi_base + BHI_ERRCODE), rd(bhi_base + BHI_ERRDBG1), rd(bhi_base + BHI_ERRDBG2), rd(bhi_base + BHI_ERRDBG3));
			return false;
		}
	}
	if (mhi == ~0u || (mhi >> MHISTATUS_STATE_SHIFT & 0xff) != MHI_STATE_M0) {
		say("apollo wifi: W2c: THE CHIP DID NOT ENTER M0 in 20 s: MHISTATUS %#x\n", mhi);
		return false;
	}
	say("apollo wifi: W2c: MHI state M0 after %llu ms\n", (now_us() - began) / 1000);
	ee = rd(bhi_base + BHI_EXECENV);
	say("apollo wifi: W2c: execution environment %#x (%s)\n", ee, execution_environment(ee));
	return true;
}

/* mhi_fw_load_bhie(): the vector table's address and size, the doorbell with a sequence id, the status polled */
static bool
w2c_bhie(void)
{
	uint64_t vector = w2c.ctrl_iova + W2C_OFF_VECTOR, began;
	uint32_t length = w2c.segments * 16, offset = rd(bar + MHI_BHIE_OFF), sequence, status = 0, db;

	if (offset == 0 || offset == ~0u || (uint64_t)offset + BHIE_RXVECSTATUS + 4 > bar_mapped) {
		say("apollo wifi: W2c: no BHIe block where MHI says (%#x)\n", offset);
		return false;
	}
	w2c.bhie = bar + offset;
	sequence = ((uint32_t)mach_absolute_time() & BHIE_SEQ_MASK) | 1;
	say("apollo wifi: W2c: BHIe at +%#x: TXVECSTATUS before %#x; vector table at %#llx, %u bytes (%u segments), sequence %u\n", offset,
	    rd(w2c.bhie + BHIE_TXVECSTATUS), vector, length, w2c.segments, sequence);
	w2b_hold("the BHIe registers and doorbell: the chip reads all of amss.bin");
	wr(w2c.bhie + BHIE_TXVECADDR_HIGH, (uint32_t)(vector >> 32));
	wr(w2c.bhie + BHIE_TXVECADDR_LOW, (uint32_t)vector);
	wr(w2c.bhie + BHIE_TXVECSIZE, length);
	db = rd(w2c.bhie + BHIE_TXVECDB);       /* mhi_write_reg_field(): read, replace the field, write */
	db = (db & ~BHIE_SEQ_MASK) | sequence;
	wr(w2c.bhie + BHIE_TXVECDB, db);
	w2c.ee = w2c.mhi = 0x12345678;
	for (began = now_us(); now_us() - began < W2C_WAIT_US; w2c_wait()) {
		w2c_poll(began, "BHIe");
		status = rd(w2c.bhie + BHIE_TXVECSTATUS);
		if (status != ~0u && (status >> 30) != 0) {
			break;
		}
	}
	if ((status >> 30) == BHIE_STATUS_ERROR || status == ~0u) {
		say("apollo wifi: W2c: THE CHIP REFUSED THE FIRMWARE: TXVECSTATUS %#x (status %u, sequence %u), BHI error %#x, debug %#x %#x %#x\n",
		    status, status >> 30, status & BHIE_SEQ_MASK, rd(bhi_base + BHI_ERRCODE), rd(bhi_base + BHI_ERRDBG1), rd(bhi_base + BHI_ERRDBG2),
		    rd(bhi_base + BHI_ERRDBG3));
		return false;
	}
	if ((status >> 30) != BHIE_STATUS_XFER_COMPL) {
		say("apollo wifi: W2c: THE CHIP DID NOT FINISH THE DOWNLOAD in 20 s: TXVECSTATUS %#x, BHI error %#x\n", status, rd(bhi_base + BHI_ERRCODE));
		return false;
	}
	say("apollo wifi: W2c: the chip took the firmware after %llu ms: TXVECSTATUS %#x (sequence %u, expected %u)\n",
	    (now_us() - began) / 1000, status, status & BHIE_SEQ_MASK, sequence);
	return true;
}

static bool
mission_mode(void)
{
	uint64_t began;
	uint32_t ee = ~0u;

	if (bhi_base == 0 || !wsmmu.on) {
		say("apollo wifi: W2c: W2b's state is not there\n");
		return false;
	}
	if (!w2c.msi_attached) {                /* (first_stage() did it already with apollo_wifi_prep) */
		w2c_msi_setup();                /* (its failure is not W2c's: the polls go on) */
	}
	if (!w2c_ready_m0()) {
		return false;
	}
	/* the chip goes on to SBL (Linux waits for the EE event; this polls BHI's execution environment) */
	for (began = now_us(); now_us() - began < W2C_WAIT_US; w2c_wait()) {
		w2c_poll(began, "waiting for SBL");
		ee = w2c.ee;
		if (ee == MHI_EE_SBL || ee == MHI_EE_AMSS || ee == MHI_EE_RDDM) {
			break;
		}
	}
	if (ee == MHI_EE_RDDM || (ee != MHI_EE_SBL && ee != MHI_EE_AMSS)) {
		say("apollo wifi: W2c: THE CHIP DID NOT REACH SBL: execution environment %#x (%s)\n", ee, execution_environment(ee));
		return false;
	}
	if (ee == MHI_EE_SBL) {
		say("apollo wifi: W2c: SBL after %llu ms\n", (now_us() - began) / 1000);
		if (!w2c_bhie()) {
			return false;
		}
	}
	/* then AMSS, with MHI in M0 */
	for (began = now_us(); now_us() - began < W2C_WAIT_US; w2c_wait()) {
		w2c_poll(began, "waiting for AMSS");
		if (w2c.ee == MHI_EE_AMSS && (w2c.mhi >> MHISTATUS_STATE_SHIFT & 0xff) == MHI_STATE_M0) {
			break;
		}
		if (w2c.ee == MHI_EE_RDDM) {
			break;
		}
	}
	w2c_events();
	if (w2c.ee != MHI_EE_AMSS || (w2c.mhi >> MHISTATUS_STATE_SHIFT & 0xff) != MHI_STATE_M0) {
		if (w2c.ee == MHI_EE_RDDM) {
			wifi_mark_dead("W2c: the chip is in RDDM, no mission mode");
			wifi_rddm_collect("W2c: the chip is in RDDM, no mission mode");
		}
		say("apollo wifi: W2c: NO MISSION MODE: execution environment %#x (%s), MHISTATUS %#x, BHI status %#x, error %#x\n", w2c.ee,
		    execution_environment(w2c.ee), w2c.mhi, rd(bhi_base + BHI_STATUS), rd(bhi_base + BHI_ERRCODE));
		return false;
	}
	mhi_wake_hold();
	say("apollo wifi: W2c DONE: the QCA6390 runs its firmware (AMSS), MHI in M0, after %llu ms\n", (now_us() - began) / 1000);
	return true;
}

/* ---- IPCR: the MHI channels 20 (to the chip) and 21 (from it), and the interface of bsd/sys/hakios.apollo_wifi.h ------------------- */

/*
 * ath11k_mhi_channels_qca6390 (drivers/net/wireless/ath/ath11k/mhi.c): "IPCR" 20, DMA_TO_DEVICE, and 21, DMA_FROM_DEVICE, each
 * num_elements = 64, event_ring = 1, doorbell MHI_DB_BRST_DISABLE, ee_mask 0x4 (AMSS), no lpm_notify, no offload; the client is
 * the QRTR MHI transport (net/qrtr/mhi.c: one QRTR packet per transfer). Neither channel has intmod (BEI clear).
 *
 * Our memory for them is one allocation in the Wi-Fi's addresses (mapped by wsmmu_map, in this order; the TRE rings are 1 KB, a
 * 4K page each keeps them aligned to their length as mhi_alloc_aligned_ring() does):
 *   +0x0000  channel 20's transfer ring (64 elements of 16 bytes)
 *   +0x1000  channel 21's transfer ring
 *   +0x2000  the bounce buffer of apollo_ipcr_send (16 KB)
 *   +0x6000  32 receive buffers of 16 KB (512 KB): channel 21's TREs, IEOT set, chain clear
 * (16 KB: a QRTR packet is up to 8192 bytes of payload and its header, more than 8 KB; Linux's buffers are MHI_MAX_MTU, 64 KB)
 */
#define IPCR_ELEMENTS           64              /* .num_elements of both channels */
#define IPCR_OFF_UL_RING        0x0000
#define IPCR_OFF_DL_RING        0x1000
#define IPCR_OFF_TX             0x2000
#define IPCR_TX_SIZE            0x4000
#define IPCR_OFF_RX             0x6000
#define IPCR_RX_SIZE            0x4000          /* (the TRE's length field is 16 bits) */
#define IPCR_RX_BUFFERS         32              /* half the ring: a TRE ring of 64 holds 63 at most */
#define IPCR_PAGES              (IPCR_OFF_RX / 4096 + IPCR_RX_BUFFERS * IPCR_RX_SIZE / 4096)
#define IPCR_CMD_TIMEOUT_MS     20000           /* ath11k_mhi_register(): mhi_ctrl->timeout_ms = 20 * 1000 */
#define IPCR_TX_TIMEOUT_MS      3000
#define IPCR_LOG_MOST           40              /* packets told in the log */

static struct {
	vm_offset_t mem;                /* never freed (the chip may use it until PERST# is down) */
	uint64_t iova;
	volatile bool up, stop, failed;
	volatile bool event_alive, deliver_alive;
	uint32_t ul_wp, dl_wp;          /* the host's write element index of each ring (mhi_ring wp) */
	volatile uint32_t ul_rp;        /* the UL element after the last one the chip has completed (its transfer events) */
	volatile bool ul_failed;        /* a send timed out: its TRE may still point at the bounce buffer, so no more sends */
	uint8_t dl_slot[IPCR_ELEMENTS]; /* the receive buffer each DL element carries */
	struct {
		uint8_t buffer;
		uint16_t length;
	} queue[IPCR_RX_BUFFERS];       /* the events waiting for the delivery thread (wifi_ev_lock) */
	unsigned queue_head, queue_count;
	uint32_t cmd_wp;                /* the command ring's write element index */
	volatile uint64_t cmd_ptr;      /* the command element the host waits for the completion of */
	volatile bool cmd_done;
	volatile uint32_t cmd_code;
	volatile uint64_t tx_ptr;       /* the UL element the sender waits for */
	volatile bool tx_done;
	volatile uint32_t tx_code;
	void (*receive)(const void *data, unsigned long length, void *context);
	void *context;
	uint32_t sent, received, dropped, told;
	uint32_t wake_refs;             /* mhi_cntrl->dev_wake */
	bool wake_set;                  /* mhi_cntrl->wake_set */
	bool wake_perm;                 /* the permanent assertion of mhi_wake_hold() is on (one of wake_refs) */
	vm_offset_t rings;              /* the two transfer rings (2 pages, one physical piece), the UNCACHED alias (ctrl_alloc) */
	uint64_t rings_iova;
} ipcr;
static uint32_t ipcr_rx_wait;           /* the event the delivery thread sleeps on (its address) */
static LCK_MTX_DECLARE(ipcr_tx_lock, &wifi_lck_grp);    /* apollo_ipcr_send: one at a time (the bounce buffer, the UL ring) */
static LCK_MTX_DECLARE(ipcr_cmd_lock, &wifi_lck_grp);   /* one command on the command ring at a time */

static void wake_toggle(void);

/* CHDBOFF in BAR0 with room for all 128 doorbells of 8 bytes (the wake doorbell 127, at +0x3f8, is the last) */
static bool
chdb_ok(void)
{
	return w2c.chdb != 0 && w2c.chdb != ~0u && (uint64_t)w2c.chdb + 8 * 128 <= bar_mapped;
}

/* mhi_write_db() of channel (or doorbell number) n: the high half first, then the low half. Not to a crashed chip; in M2 the wake first (main.c wake_toggle) */
static void
ipcr_channel_db(unsigned n, uint64_t value)
{
	if (wifi_dead || !wifi_link_alive) {
		return;
	}
	if (n != MHI_DEV_WAKE_DB && w2c_m2) {
		wake_toggle();
	}
	mhi_write64(w2c.chdb + 8 * n, value);
}

/* The crash, once: from here no register of the chip is touched except by wifi_rddm_collect() (BHI, BHIe) */
static void
wifi_mark_dead(const char *why)
{
	if (!__sync_bool_compare_and_swap(&wifi_dead, false, true)) {
		return;
	}
	say("apollo wifi: THE FIRMWARE HAS CRASHED (%s): no more register access, doorbells or sends; only the RDDM collection reads the chip\n", why);
}

/* mhi_assert_dev_wake(force) / mhi_deassert_dev_wake(): the count, and the wake doorbell (M0 is a state the doorbell is valid in). With the permanent assertion (mhi_wake_hold) only the count moves: the doorbell stays up. A crashed chip: nothing. */
void
apollo_wifi_wake(int on)
{
	bool valid = wifi_amss && chdb_ok();

	if (wifi_dead) {
		return;
	}
	lck_spin_lock(&wifi_wake_lock);
	if (on) {
		ipcr.wake_refs++;
		if (valid && !ipcr.wake_perm && !ipcr.wake_set) {       /* MHI_WAKE_DB_FORCE_SET_VALID(M0) && !wake_set */
			ipcr_channel_db(MHI_DEV_WAKE_DB, 1);
			ipcr.wake_set = true;
		}
	} else if (ipcr.wake_refs != 0 && --ipcr.wake_refs == 0 && valid && ipcr.wake_set) {  /* MHI_WAKE_DB_CLEAR_VALID(M0) */
		ipcr_channel_db(MHI_DEV_WAKE_DB, 0);
		ipcr.wake_set = false;
	}
	lck_spin_unlock(&wifi_wake_lock);
}

/*
 * The device kept in M0: mhi_assert_dev_wake(mhi_cntrl, true) (pm.c, the force case: dev_wake++, wake doorbell 127 = 1 when the PM
 * state allows and wake_set is clear), once, right after AMSS and before IPCR; it stays up (its count included) until power down. Then
 * MHISTATUS is polled for state M0, 2 s at most. wifi_amss is not set yet here, so the doorbell is rung directly (M0 is established).
 */
static void
mhi_wake_hold(void)
{
	uint64_t began = now_us();
	uint32_t mhi = ~0u;

	if (wifi_dead || !chdb_ok()) {
		return;
	}
	lck_spin_lock(&wifi_wake_lock);
	if (!ipcr.wake_perm) {
		ipcr.wake_refs++;
		ipcr.wake_perm = true;
		if (!ipcr.wake_set) {
			ipcr_channel_db(MHI_DEV_WAKE_DB, 1);
			ipcr.wake_set = true;
		}
	}
	lck_spin_unlock(&wifi_wake_lock);
	for (; now_us() - began < 2000000ull; sleep_ms(1)) {
		mhi = rd(bar + MHI_STATUS);
		if (mhi != ~0u && (mhi >> MHISTATUS_STATE_SHIFT & 0xff) == MHI_STATE_M0) {
			break;
		}
	}
	say("apollo wifi: W2c: device wake asserted for good (wake doorbell %u = 1): MHISTATUS %#x, state %u (%s) after %llu ms\n", MHI_DEV_WAKE_DB, mhi,
	    mhi >> MHISTATUS_STATE_SHIFT & 0xff, (mhi != ~0u && (mhi >> MHISTATUS_STATE_SHIFT & 0xff) == MHI_STATE_M0) ? "M0" : "NOT M0", (now_us() - began) / 1000);
}

/* power down: the permanent assertion released (mhi_deassert_dev_wake: the count down, the doorbell to 0 at the last one); a crashed chip: nothing */
static void
mhi_wake_release(void)
{
	bool ring;

	if (wifi_dead) {
		return;
	}
	lck_spin_lock(&wifi_wake_lock);
	ipcr.wake_perm = false;
	ring = false;
	if (ipcr.wake_refs != 0 && --ipcr.wake_refs == 0 && ipcr.wake_set) {
		ring = true;
		ipcr.wake_set = false;
	}
	if (ring) {
		ipcr_channel_db(MHI_DEV_WAKE_DB, 0);
	}
	lck_spin_unlock(&wifi_wake_lock);
}

static bool
mhi_wake_is_held(void)
{
	return ipcr.wake_perm || ipcr.wake_refs != 0;
}

/* mhi_pm_m1_transition()'s wake_get(force): the wake doorbell once more, the count unchanged */
static void
mhi_wake_reassert(void)
{
	lck_spin_lock(&wifi_wake_lock);
	ipcr_channel_db(MHI_DEV_WAKE_DB, 1);
	ipcr.wake_set = true;
	lck_spin_unlock(&wifi_wake_lock);
}

/* main.c mhi_toggle_dev_wake (the wake_toggle of M2, before a doorbell): wake_get(false) then wake_put(override): the wake doorbell is up afterwards, the count as before */
static void
wake_toggle(void)
{
	lck_spin_lock(&wifi_wake_lock);
	if (!ipcr.wake_set) {
		ipcr_channel_db(MHI_DEV_WAKE_DB, 1);
		ipcr.wake_set = true;
	}
	lck_spin_unlock(&wifi_wake_lock);
}

static bool
ipcr_cmd_pending(void)
{
	return ipcr.cmd_ptr != 0;
}

static bool
ipcr_up(void)
{
	return ipcr.up;
}

/* power down has begun: the IPCR threads are on their way out (wifi_rddm_collect() gives up) */
static bool
ipcr_stopping(void)
{
	return ipcr.stop;
}

/* a transfer ring element: ptr in words 0 and 1, dword[0] = length, dword[1] = type 2 (transfer) and IEOT (MHI_TRE_DATA_DWORD1(0, 1, 0, 0)) */
static void
ipcr_tre(vm_offset_t element, uint64_t buffer, uint32_t length)
{
	volatile uint32_t *tre = (volatile uint32_t *)element;

	tre[0] = (uint32_t)buffer;
	tre[1] = (uint32_t)(buffer >> 32);
	tre[2] = length & 0xffff;                       /* MHI_TRE_DATA_DWORD0 */
	tre[3] = W2C_TRE_TYPE_TRANSFER << 16 | 1u << 9; /* MHI_TRE_DATA_DWORD1: IEOT (BIT 9) */
	/* (the rings are the uncached alias: no cache maintenance; mhi_write64() orders the stores before the doorbell) */
}

/* mhi_gen_tre() for channel 21 (the ring never fills: IPCR_RX_BUFFERS < IPCR_ELEMENTS - 1) and, with ring_db, mhi_ring_chan_db() */
static void
ipcr_dl_queue(uint8_t buffer, bool ring_db)
{
	struct w2c_context *chan = (struct w2c_context *)(w2c.ctrl + W2C_OFF_CHAN);
	uint64_t ring = ipcr.rings_iova + IPCR_OFF_DL_RING;

	ipcr_tre(ipcr.rings + IPCR_OFF_DL_RING + (vm_offset_t)ipcr.dl_wp * W2C_ELEMENT, ipcr.iova + IPCR_OFF_RX + (uint64_t)buffer * IPCR_RX_SIZE, IPCR_RX_SIZE);
	ipcr.dl_slot[ipcr.dl_wp] = buffer;
	ipcr.dl_wp = (ipcr.dl_wp + 1) % IPCR_ELEMENTS;
	if (ring_db) {
		chan[IPCR_DL].wp = ring + (uint64_t)ipcr.dl_wp * W2C_ELEMENT;   /* mhi_ring_chan_db(): *ring->ctxt_wp, then the doorbell */
		ipcr_channel_db(IPCR_DL, chan[IPCR_DL].wp);
	}
}

/*
 * Called by w2c_events() with wifi_ev_lock held, on every element of an event ring. The command completion of ring 0 is recorded for
 * ipcr_command() (mhi_process_cmd_completion(): the element it points at is the command's), not consumed: false makes the caller log
 * it too. The transfer events of ring 1 once the channels run (parse_xfer_event()): true, handled.
 */
static bool
ipcr_event(unsigned ring, __unused uint64_t index, const volatile uint32_t *tre)
{
	uint32_t type = tre[3] >> 16 & 0xff, code = tre[2] >> 24, length = tre[2] & 0xffff, chan = tre[3] >> 24;
	uint64_t ptr = (uint64_t)tre[1] << 32 | tre[0];

	if (ring == 0 && type == W2C_PKT_CMD_COMPLETION && ipcr.cmd_ptr != 0 && ptr == ipcr.cmd_ptr) {
		ipcr.cmd_code = code;
		ipcr.cmd_done = true;
		thread_wakeup((event_t)&ipcr.cmd_done);
		return false;
	}
	if (ring != 1 || type != W2C_PKT_TX_EVENT || !ipcr.up) {
		return false;
	}
	if (code != W2C_EV_CC_EOT && code != W2C_EV_CC_EOB && code != W2C_EV_CC_OVERFLOW) {     /* OOB and DB_MODE: burst mode is off */
		say("apollo wifi: IPCR: channel %u transfer event code %u (ptr %#llx, length %u)\n", chan, code, ptr, length);
		return true;
	}
	if (chan == IPCR_UL) {
		uint64_t ring_base = ipcr.rings_iova + IPCR_OFF_UL_RING;

		if (ptr >= ring_base && ptr < ring_base + IPCR_ELEMENTS * W2C_ELEMENT && (ptr - ring_base) % W2C_ELEMENT == 0) {
			ipcr.ul_rp = (uint32_t)((ptr - ring_base) / W2C_ELEMENT + 1) % IPCR_ELEMENTS;  /* (the chip's rp: the ring-full check of send) */
		}
		if (ptr == ipcr.tx_ptr) {                       /* (a late event of a send that timed out is not the one waited for) */
			ipcr.tx_code = code;
			ipcr.tx_done = true;
			thread_wakeup((event_t)&ipcr.tx_done);
		}
	} else if (chan == IPCR_DL) {
		uint64_t ring_base = ipcr.rings_iova + IPCR_OFF_DL_RING;
		unsigned slot;
		uint8_t buffer;

		if (ptr < ring_base || ptr >= ring_base + IPCR_ELEMENTS * W2C_ELEMENT || (ptr - ring_base) % W2C_ELEMENT) {
			say("apollo wifi: IPCR: DL event points outside the ring (%#llx)\n", ptr);
			return true;
		}
		slot = (unsigned)((ptr - ring_base) / W2C_ELEMENT);
		buffer = ipcr.dl_slot[slot];
		if (code == W2C_EV_CC_OVERFLOW || length > IPCR_RX_SIZE || ipcr.queue_count >= IPCR_RX_BUFFERS) {
			ipcr.dropped++;                         /* a packet bigger than the buffer, as parse_xfer_event(): -EOVERFLOW; not delivered */
			say("apollo wifi: IPCR: a packet of %u bytes dropped (code %u)\n", length, code);
			ipcr_dl_queue(buffer, true);
			return true;
		}
		slot = (ipcr.queue_head + ipcr.queue_count++) % IPCR_RX_BUFFERS;
		ipcr.queue[slot].buffer = buffer;
		ipcr.queue[slot].length = (uint16_t)length;
		thread_wakeup((event_t)&ipcr_rx_wait);
	}
	return true;
}

/* Sleep on an event until the flag is set (the wakeup may come before the sleep: 1 ms steps) or ms have passed */
static bool
ipcr_wait(volatile bool *flag, uint32_t ms)
{
	for (uint64_t began = now_us(); !*flag; ) {
		if (now_us() - began > (uint64_t)ms * 1000) {
			return false;
		}
		assert_wait_timeout((event_t)flag, THREAD_UNINT, 1, NSEC_PER_MSEC);
		thread_block(THREAD_CONTINUE_NULL);
	}
	return true;
}

/* The event ring thread: both event rings, on the MSI interrupts (vectors 1 and 2 wake w2c_wait()) and every 10 ms */
static void
ipcr_event_thread(__unused void *parameter, __unused wait_result_t result)
{
	bool fault_told = false, error_told = false;

	while (!ipcr.stop) {
		w2c_events();                   /* (returns at once once the firmware has crashed) */
		/* the radio driver at work: a refused access of the chip's, or the firmware down (SYS_ERR), is told once */
		wsmmu_faults(&fault_told);
		if (!error_told && !ipcr.stop && wifi_link_alive && (wifi_dead || (rd(bar + MHI_STATUS) >> 8 & 0xff) == 0xff)) {
			error_told = true;
			wifi_mark_dead("the firmware went to SYS_ERR while the driver ran");   /* from here nothing but the RDDM collection touches the chip */
			/* (no wifi_final_dump(): its window registers are accesses to a crashed chip) */
			wifi_rddm_collect("the firmware went to SYS_ERR while the driver ran");
		}
		w2c_wait();
	}
	ipcr.event_alive = false;
	thread_terminate(current_thread());
}

/* The receive function's thread: one packet at a time, then its buffer back on the ring */
static void
ipcr_deliver_thread(__unused void *parameter, __unused wait_result_t result)
{
	while (!ipcr.stop) {
		uint8_t buffer = 0;
		unsigned length = 0;
		bool have = false;
		void (*receive)(const void *, unsigned long, void *);

		lck_mtx_lock(&wifi_ev_lock);
		if (ipcr.queue_count != 0) {
			buffer = ipcr.queue[ipcr.queue_head].buffer;
			length = ipcr.queue[ipcr.queue_head].length;
			ipcr.queue_head = (ipcr.queue_head + 1) % IPCR_RX_BUFFERS;
			ipcr.queue_count--;
			have = true;
		}
		lck_mtx_unlock(&wifi_ev_lock);
		if (!have) {
			assert_wait_timeout((event_t)&ipcr_rx_wait, THREAD_UNINT, 10, NSEC_PER_MSEC);
			thread_block(THREAD_CONTINUE_NULL);
			continue;
		}
		receive = ipcr.receive;
		if (receive != NULL) {
			vm_offset_t data = ipcr.mem + IPCR_OFF_RX + (vm_offset_t)buffer * IPCR_RX_SIZE;

			cache_flush(data, length);              /* the chip wrote it behind the cache */
			if (ipcr.told < IPCR_LOG_MOST) {
				ipcr.told++;
				say("apollo wifi: IPCR: received %u bytes (packet %u)\n", length, ipcr.received + 1);
			}
			ipcr.received++;
			receive((const void *)data, length, ipcr.context);
		} else {
			ipcr.dropped++;
		}
		lck_mtx_lock(&wifi_ev_lock);
		ipcr_dl_queue(buffer, true);
		lck_mtx_unlock(&wifi_ev_lock);
	}
	ipcr.deliver_alive = false;
	thread_terminate(current_thread());
}

/*
 * mhi_send_cmd() and mhi_update_channel_state(): the device awake (mhi_device_get_sync), the command's element on the command ring
 * (ptr 0, dword[0] 0, dword[1] = channel << 24 | command << 16: MHI_TRE_CMD_{START,STOP,RESET}_DWORD1), the ring's wp in the command
 * context and on the command ring doorbell (CRDB, mhi_ring_cmd_db()), then the CMD_COMPLETION event of ring 0 that points at the element,
 * code SUCCESS (mhi_process_cmd_completion(), checked as mhi_chan->ccs).
 */
static int
ipcr_command(unsigned chan, unsigned command, const char *what)
{
	struct w2c_context *cmd = (struct w2c_context *)(w2c.ctrl + W2C_OFF_CMD);
	uint64_t ring = w2c.ctrl_iova + W2C_OFF_CMD_RING;
	vm_offset_t element;
	volatile uint32_t *tre;
	int error = 0;

	if (wifi_dead) {
		return EIO;
	}
	lck_mtx_lock(&ipcr_cmd_lock);
	apollo_wifi_wake(1);
	element = w2c.ctrl + W2C_OFF_CMD_RING + (vm_offset_t)ipcr.cmd_wp * W2C_ELEMENT;
	tre = (volatile uint32_t *)element;
	tre[0] = tre[1] = tre[2] = 0;
	tre[3] = chan << 24 | command << 16;
	ipcr.cmd_done = false;
	ipcr.cmd_ptr = ring + (uint64_t)ipcr.cmd_wp * W2C_ELEMENT;
	ipcr.cmd_wp = (ipcr.cmd_wp + 1) % W2C_CMD_ELEMENTS;
	cmd->wp = ring + (uint64_t)ipcr.cmd_wp * W2C_ELEMENT;
	if (!wifi_dead) {
		mhi_write64(MHI_CRDB_LOWER, cmd->wp);
	}
	if (wifi_dead) {
		error = EIO;
	} else if (!ipcr_wait(&ipcr.cmd_done, IPCR_CMD_TIMEOUT_MS)) {
		say("apollo wifi: IPCR: %s of channel %u: no completion event in %u ms (command ring wp %#llx)\n", what, chan, IPCR_CMD_TIMEOUT_MS, cmd->wp);
		error = ETIMEDOUT;
	} else if (ipcr.cmd_code != W2C_EV_CC_SUCCESS) {
		say("apollo wifi: IPCR: %s of channel %u completed with code %u, not SUCCESS\n", what, chan, ipcr.cmd_code);
		error = EIO;
	} else {
		say("apollo wifi: IPCR: %s of channel %u: completion SUCCESS\n", what, chan);
	}
	ipcr.cmd_ptr = 0;
	apollo_wifi_wake(0);
	lck_mtx_unlock(&ipcr_cmd_lock);
	return error;
}

/* mhi_init_chan_ctxt(): chstate ENABLED, ring base, length in bytes, rp = wp = base (the brstmode, chtype and erindex are mhi_init_dev_ctxt's, W2c's) */
static void
ipcr_chan_context(unsigned chan, uint64_t ring)
{
	struct w2c_context *context = (struct w2c_context *)(w2c.ctrl + W2C_OFF_CHAN) + chan;

	context->word0 = (context->word0 & ~0xffu) | 1;         /* CHAN_CTX_CHSTATE: MHI_CH_STATE_ENABLED */
	context->rbase = context->rp = context->wp = ring;
	context->rlen = (uint64_t)IPCR_ELEMENTS * W2C_ELEMENT;
}

/* the IPCR channels started: memory, the two threads, the channel contexts, MHI_CMD_START_CHAN for 20 then 21 (mhi_prepare_for_transfer), the receive buffers */
static int
ipcr_start(void)
{
	vm_size_t bytes = (vm_size_t)IPCR_PAGES * 4096;
	int error;

	if (ipcr.up) {
		return 0;
	}
	if (ipcr.failed) {
		return EIO;
	}
	if (!wifi_amss || !chdb_ok()) {
		say("apollo wifi: IPCR: the chip is not in mission mode, or its channel doorbells are not known (%#x)\n", w2c.chdb);
		return ENXIO;
	}
	ipcr.failed = true;                     /* until it all works */
	if (kmem_alloc(kernel_map, &ipcr.mem, bytes, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		ipcr.mem = 0;
		say("apollo wifi: IPCR: no memory (%u KB)\n", (unsigned)(bytes / 1024));
		return ENOMEM;
	}
	cache_flush(ipcr.mem, bytes);           /* the zeroes written back before the chip writes (no dirty line later overwrites its data) */
	/* the buffers (from +IPCR_OFF_TX on; ipcr.iova is the address the first page would have: the offsets stay) and, apart, the two rings as the uncached alias */
	if (!wsmmu_map(ipcr.mem + IPCR_OFF_TX, (uint32_t)(bytes - IPCR_OFF_TX), &ipcr.iova)) {
		return ENOMEM;
	}
	ipcr.iova -= IPCR_OFF_TX;
	if (ipcr.rings == 0) {
		uint64_t physical = 0;

		ipcr.rings = ctrl_alloc(2 * 4096, &physical);
		if (ipcr.rings == 0 || !wsmmu_map_as(0, physical, 2 * 4096, WIFI_PTE, &ipcr.rings_iova)) {
			ipcr.rings = 0;
			say("apollo wifi: IPCR: no memory for the rings (8 KB in one piece)\n");
			return ENOMEM;
		}
	}
	ipcr.ul_wp = ipcr.dl_wp = ipcr.cmd_wp = ipcr.ul_rp = 0;
	ipcr.ul_failed = false;
	ipcr.queue_head = ipcr.queue_count = 0;
	ipcr.stop = false;
	ipcr.event_alive = ipcr.deliver_alive = true;
	say("apollo wifi: IPCR: %u KB at %#llx: UL ring +%#x, DL ring +%#x (%u elements each), bounce buffer +%#x (%u KB), %u receive buffers of %u KB +%#x; "
	    "channel doorbells at +%#x, command ring doorbell +%#x\n", (unsigned)(bytes / 1024), ipcr.rings_iova, IPCR_OFF_UL_RING, IPCR_OFF_DL_RING,
	    IPCR_ELEMENTS, IPCR_OFF_TX, IPCR_TX_SIZE / 1024, IPCR_RX_BUFFERS, IPCR_RX_SIZE / 1024, IPCR_OFF_RX, w2c.chdb, MHI_CRDB_LOWER);
	{
		thread_t thread;

		if (kernel_thread_start(ipcr_event_thread, NULL, &thread) != KERN_SUCCESS) {
			ipcr.event_alive = false;
			ipcr.stop = true;
			return ENOMEM;
		}
		thread_deallocate(thread);
		if (kernel_thread_start(ipcr_deliver_thread, NULL, &thread) != KERN_SUCCESS) {
			ipcr.deliver_alive = false;
			ipcr.stop = true;
			return ENOMEM;
		}
		thread_deallocate(thread);
	}
	/* mhi_prepare_channel(): mhi_init_chan_ctxt(), then the START command, UL (dir 0) first */
	ipcr_chan_context(IPCR_UL, ipcr.rings_iova + IPCR_OFF_UL_RING);
	ipcr_chan_context(IPCR_DL, ipcr.rings_iova + IPCR_OFF_DL_RING);
	w2b_hold("the first MHI command on the command ring: START_CHAN of IPCR channel 20, then 21 (the chip reads the channel contexts)");
	error = ipcr_command(IPCR_UL, W2C_CMD_START_CHAN, "START_CHAN");
	if (error == 0) {
		error = ipcr_command(IPCR_DL, W2C_CMD_START_CHAN, "START_CHAN");
	}
	if (error != 0) {
		ipcr.stop = true;
		return error;
	}
	/* the receive buffers, then channel 21's doorbell once (the autoqueue of mhi_prepare_for_transfer_autoqueue(): mhi_gen_tre() each, one mhi_ring_chan_db()) */
	lck_mtx_lock(&wifi_ev_lock);
	ipcr.up = true;                         /* (the events of channel 21 count from here) */
	for (unsigned n = 0; n < IPCR_RX_BUFFERS; n++) {
		ipcr_dl_queue((uint8_t)n, n == IPCR_RX_BUFFERS - 1);
	}
	lck_mtx_unlock(&wifi_ev_lock);
	ipcr.failed = false;
	say("apollo wifi: IPCR DONE: channels 20 and 21 run, %u receive buffers of %u bytes queued\n", IPCR_RX_BUFFERS, IPCR_RX_SIZE);
	return 0;
}

void
apollo_ipcr_set_receive(void (*receive)(const void *data, unsigned long length, void *context), void *context)
{
	ipcr.context = context;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	ipcr.receive = receive;
}

/* mhi_queue_buf() on channel 20: the bounce buffer as one TRE (IEOT), the ring's wp in the channel context and on the doorbell; then the TX event of ring 1 */
int
apollo_ipcr_send(const void *data, unsigned long length)
{
	struct w2c_context *chan = (struct w2c_context *)(w2c.ctrl + W2C_OFF_CHAN);
	uint64_t ring = ipcr.rings_iova + IPCR_OFF_UL_RING, element;
	int error = 0;

	if (wifi_dead) {
		return EIO;
	}
	if (!ipcr.up || ipcr.stop) {
		return ENOTCONN;
	}
	if (length == 0 || length > IPCR_TX_SIZE || length > 0xffff) {
		return EMSGSIZE;
	}
	lck_mtx_lock(&ipcr_tx_lock);
	/* (again under the lock: power down clears ipcr.up holding it, so no send begins on a channel being reset) */
	if (!ipcr.up || ipcr.stop || wifi_dead) {
		lck_mtx_unlock(&ipcr_tx_lock);
		return ENOTCONN;
	}
	if (ipcr.ul_failed) {
		lck_mtx_unlock(&ipcr_tx_lock);
		return EIO;
	}
	if ((ipcr.ul_wp + 1) % IPCR_ELEMENTS == ipcr.ul_rp) {   /* mhi_is_ring_full() */
		lck_mtx_unlock(&ipcr_tx_lock);
		say("apollo wifi: IPCR: the UL ring is full (wp %u, the chip's rp %u): a send of %lu bytes refused\n", ipcr.ul_wp, ipcr.ul_rp, length);
		return ENOBUFS;
	}
	memcpy((void *)(ipcr.mem + IPCR_OFF_TX), data, length);
	cache_flush(ipcr.mem + IPCR_OFF_TX, length);
	element = ring + (uint64_t)ipcr.ul_wp * W2C_ELEMENT;
	ipcr_tre(ipcr.rings + IPCR_OFF_UL_RING + (vm_offset_t)ipcr.ul_wp * W2C_ELEMENT, ipcr.iova + IPCR_OFF_TX, (uint32_t)length);
	ipcr.tx_done = false;
	ipcr.tx_ptr = element;
	ipcr.ul_wp = (ipcr.ul_wp + 1) % IPCR_ELEMENTS;
	chan[IPCR_UL].wp = ring + (uint64_t)ipcr.ul_wp * W2C_ELEMENT;   /* mhi_ring_chan_db() */
	ipcr_channel_db(IPCR_UL, chan[IPCR_UL].wp);
	if (!ipcr_wait(&ipcr.tx_done, IPCR_TX_TIMEOUT_MS)) {
		/*
		 * The chip may still read that TRE, and it points at the one bounce buffer that the next send would overwrite:
		 * the UL channel is given up (Linux's QRTR would see the same channel stuck).
		 */
		ipcr.ul_failed = true;
		say("apollo wifi: IPCR: a send of %lu bytes was not completed in %u ms: THE UL CHANNEL IS GIVEN UP, no more sends\n", length,
		    IPCR_TX_TIMEOUT_MS);
		error = ETIMEDOUT;
	} else if (ipcr.tx_code != W2C_EV_CC_EOT && ipcr.tx_code != W2C_EV_CC_EOB) {
		say("apollo wifi: IPCR: a send of %lu bytes completed with code %u\n", length, ipcr.tx_code);
		error = EIO;
	} else {
		ipcr.sent++;
	}
	ipcr.tx_ptr = 0;
	lck_mtx_unlock(&ipcr_tx_lock);
	return error;
}

unsigned
apollo_wifi_intid(unsigned vector)
{
	return W2C_MSI_INTID + vector;
}

int
apollo_wifi_msi(unsigned long long *address, unsigned *data)
{
	if (!w2c.msi_on || w2c.msi_address == 0) {
		return ENODEV;
	}
	*address = w2c.msi_address;
	*data = W2C_MSI_INTID;
	return 0;
}

/*
 * ath11k_pcic_read32/write32 (pcic.c): offsets below ATH11K_PCI_WINDOW_START are direct, above through the window register
 * (ath11k_pci_window_read32/write32, pci.c: window selected under register_window_lock, then the access). A spin lock holds both
 * steps together; not for interrupt context. The chip's wake (apollo_wifi_wake) is the caller's business, as in Linux.
 */
uint32_t
apollo_wifi_read32(uint32_t offset)
{
	uint32_t value;

	if (!wifi_amss || wifi_dead || !wifi_link_alive) {
		return ~0u;
	}
	lck_spin_lock(&wifi_chip_lock);
	value = chip_rd(offset);
	lck_spin_unlock(&wifi_chip_lock);
	return value;
}

void
apollo_wifi_write32(uint32_t offset, uint32_t value)
{
	if (!wifi_amss || wifi_dead || !wifi_link_alive) {
		return;
	}
	lck_spin_lock(&wifi_chip_lock);
	chip_wr(offset, value);
	lck_spin_unlock(&wifi_chip_lock);
}

/*
 * The buffer's pages entered in the Wi-Fi's SMMU context at the next free addresses (up to the pool of 256 MB: WIFI_L3_TABLES), as normal
 * non-cacheable memory; the caches written back and invalidated first, so that no line of the buffer is written back over what the chip
 * writes (the contents the caller has put there stay). The size is rounded up to whole 4K pages.
 */
int
apollo_wifi_dma_map(void *buffer, unsigned long size, unsigned long long *iova)
{
	uint64_t at = 0;

	if (buffer == NULL || iova == NULL || size == 0 || ((vm_offset_t)buffer & 4095) || size > WIFI_POOL_SIZE) {
		return EINVAL;
	}
	if (!wsmmu.on) {
		return ENODEV;
	}
	size = (size + 4095) & ~4095ul;
	cache_flush((vm_offset_t)buffer, size);
	if (!wsmmu_map((vm_offset_t)buffer, (uint32_t)size, &at)) {
		return ENOMEM;
	}
	*iova = at;
	return 0;
}

/* ---- the whole of step W1 to W2c, and power up/down ------------------------------------------------------- */

static volatile bool started;
static volatile bool clocked;   /* the pcie_0 power domain and clocks are on: PARF may be touched */
/*
 * apollo_wifi_power_down() ran, or W2b/W2c failed and the chip was quiesced: the state of W1 to W2c (tables, contexts) is not made for
 * a second run, so from here power up says ENOTSUP. One way: nothing clears it before the next boot.
 */
static bool wifi_downed;

/*
 * apollo_qrtr_init (hakios.apollo_qrtr.c) and apollo_lx_prepare/start (hakios.apollo_lx.c: weak there, the apollo ath11k object's when
 * it is linked in) are always there: both files are "standard" in bsd/conf/files (devices/mi-10t/xnu/mi-10t.patch).
 */

/* W1 up to the stage asked for (1 to 4: W2c is stage 4); 0 or an errno. With stage 4 and a success, the chip runs AMSS (wifi_amss). */
static int
wifi_bring_up(uint32_t stage)
{
	int error = EIO;

	pin_wlan_en = ml_io_map(TLMM_NORTH_PHYS + 0x1000UL * PIN_WLAN_EN, 0x1000);
	pins_pcie = ml_io_map(TLMM_NORTH_PHYS + 0x1000UL * PIN_PERST, 0x3000);
	gcc = ml_io_map(GCC_PHYS, GCC_SIZE);
	rsc = ml_io_map(RSC_PHYS, RSC_SIZE);
	phy = ml_io_map(PHY_PHYS, PHY_SIZE);
	parf = ml_io_map(PARF_PHYS, PARF_SIZE);
	dbi = ml_io_map(DBI_PHYS, 0x1000);
	atu = ml_io_map(ATU_PHYS, 0x1000);
	conf = ml_io_map(CONF_PHYS, 0x1000);
	if (!pin_wlan_en || !pins_pcie || !gcc || !rsc || !phy || !parf || !dbi || !atu || !conf) {
		say("apollo wifi: cannot map the registers\n");
		return ENXIO;
	}
	say("apollo wifi: W1: power the QCA6390 and bring up PCIe0 (as found: WLAN_EN %u, PERST# %u, GDSCR %#x)\n",
	    rd(pin(PIN_WLAN_EN) + 4) & 1, rd(pin(PIN_PERST) + 4) & 1, rd(gcc + PCIE0_GDSCR));
	started = true;
	/* PERST# held low and the chip off while its power comes */
	pin_setup(PIN_PERST, 0, PULL_DOWN, 2, true, false);
	pin_setup(PIN_WAKE, 0, PULL_UP, 2, false, false);
	pin_setup(PIN_WLAN_EN, 0, PULL_DOWN, 2, true, false);
	if (!power_on()) {
		goto off;
	}
	sleep_ms(100);
	pin_setup(PIN_WLAN_EN, 0, PULL_UP, 16, true, true);
	sleep_ms(60);
	say("apollo wifi: WLAN_EN is high (%u)\n", rd(pin(PIN_WLAN_EN) + 4) & 1);
	pin_setup(PIN_CLKREQ, PIN_FUNCTION_PCI_E0, PULL_UP, 2, false, false);
	if (!clocks_on()) {
		goto off;
	}
	clocked = true;
	if (!phy_on()) {
		goto off;
	}
	bcr_pulse(PCIE0_BCR);
	controller_setup();
	if (!link_on()) {
		goto off;
	}
	wifi_link_alive = true;         /* (BAR0 may be touched until apollo_wifi_quiesce()) */
	say("apollo wifi: W1 DONE: the QCA6390 is on PCIe0, link up, configuration space readable\n");
	chip_prep = stage >= 4;         /* apollo_wifi_prep: default on from W2c, off for W2b alone */
	PE_parse_boot_argn("apollo_wifi_prep", &chip_prep, sizeof(chip_prep));
	PE_parse_boot_argn("apollo_wifi_rddm", &wifi_rddm, sizeof(wifi_rddm));
	error = 0;
	if (stage >= 2 && mhi_look()) {
		say("apollo wifi: W2a DONE: the chip's MHI and BHI registers answer through BAR0\n");
		/*
		 * A failed W2b or W2c: the INTIDs given back, then the chip quiesced as power down does -- PERST# and WLAN_EN down
		 * before the SMMU context goes, so that a chip still running can not DMA through a context that is gone -- and the
		 * state marked as power down's (wifi_downed: power up says ENOTSUP, not a second pass over half-made tables).
		 */
		if (stage >= 3 && !first_stage()) {
			say("apollo wifi: W2b failed: PERST# and WLAN_EN down\n");
			w2c_msi_stop();
			apollo_wifi_quiesce();
			wifi_downed = true;
			error = EIO;
		} else if (stage >= 4) {
			if (mission_mode()) {
				wifi_amss = true;
			} else {
				say("apollo wifi: W2c failed: PERST# and WLAN_EN down\n");
				w2c_msi_stop();
				apollo_wifi_quiesce();
				wifi_downed = true;
				error = EIO;
			}
		}
	} else if (stage >= 2) {
		error = EIO;
	}
	return error;
off:
	apollo_wifi_quiesce();
	say("apollo wifi: W1 failed: PERST# and WLAN_EN are down again\n");
	return EIO;
}

/* apollo_wifi=1 to 5: the stages up to W2c as before; 5 goes on: the chip to AMSS, IPCR, the QRTR transport, then the apollo ath11k object */
static void
apollo_wifi_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t stage = 0;

	PE_parse_boot_argn("apollo_wifi", &stage, sizeof(stage));
	if (stage >= 5) {
		int error = apollo_wifi_power_up();

		say("apollo wifi: stage 5: power up (W1 to W2c, IPCR): %d\n", error);
		if (error == 0) {
			error = apollo_qrtr_init();
			say("apollo wifi: stage 5: apollo_qrtr_init: %d\n", error);
			error = apollo_lx_prepare();
			say("apollo wifi: stage 5: apollo_lx_prepare: %d\n", error);
			if (error == 0) {
				error = apollo_lx_start();
				say("apollo wifi: stage 5: apollo_lx_start: %d\n", error);
			}
		}
	} else {
		lck_mtx_lock(&wifi_lock);
		(void)wifi_bring_up(stage);
		lck_mtx_unlock(&wifi_lock);
	}
	thread_terminate(current_thread());
}

/* W1, W2a, W2b (chip preparation), W2c unless the chip already runs AMSS, then the IPCR channels. Idempotent, one caller at a time. Sleeps. */
int
apollo_wifi_power_up(void)
{
	int error = 0;

	lck_mtx_lock(&wifi_lock);
	if (!wifi_amss) {
		if (wifi_downed || started) {
			/* (started without AMSS: an earlier run stopped short (apollo_wifi < 4) or W2a failed, and left state that a second pass is not made for) */
			error = wifi_downed ? ENOTSUP : EIO;
		} else {
			error = wifi_bring_up(4);
			if (error == 0 && !wifi_amss) {
				error = EIO;
			}
		}
	}
	if (error == 0) {
		error = ipcr_start();
	}
	lck_mtx_unlock(&wifi_lock);
	return error;
}

#define IPCR_EXIT_WAIT_MS       30000           /* the threads' way out: an RDDM collection gives up at once, a receive function may take a while */

/*
 * The IPCR channels reset (mhi_unprepare_from_transfer(): RESET_CHAN, UL first), the threads stopped and waited for, then PERST# and
 * WLAN_EN down as quiesce does. ipcr.up is cleared under ipcr_tx_lock: a send under way ends first, none begins after.
 */
void
apollo_wifi_power_down(void)
{
	bool was_up;
	uint64_t began;

	lck_mtx_lock(&wifi_lock);
	lck_mtx_lock(&ipcr_tx_lock);
	was_up = ipcr.up;
	ipcr.up = false;
	lck_mtx_unlock(&ipcr_tx_lock);
	if (was_up) {
		(void)ipcr_command(IPCR_UL, W2C_CMD_RESET_CHAN, "RESET_CHAN");
		(void)ipcr_command(IPCR_DL, W2C_CMD_RESET_CHAN, "RESET_CHAN");
	}
	mhi_wake_release();                     /* the permanent wake assertion of W2c ends with the channels */
	ipcr.stop = true;
	/* until both threads say they are out (their last BAR access behind them), not a fixed time: PERST# under a thread at work hangs the bus */
	for (began = now_us(); (ipcr.event_alive || ipcr.deliver_alive) && now_us() - began < IPCR_EXIT_WAIT_MS * 1000ull; ) {
		thread_wakeup((event_t)&w2c_wake);
		thread_wakeup((event_t)&ipcr_rx_wait);
		sleep_ms(1);
	}
	if (ipcr.event_alive || ipcr.deliver_alive) {
		say("apollo wifi: power down: THE IPCR THREADS HAVE NOT ENDED in %u ms (event %d, delivery %d): the link goes anyway "
		    "(their BAR accesses check wifi_link_alive)\n", IPCR_EXIT_WAIT_MS, ipcr.event_alive, ipcr.deliver_alive);
	}
	wifi_amss = false;                      /* nothing touches the chip's registers from here */
	if (started) {
		apollo_wifi_quiesce();
		wifi_downed = true;
		say("apollo wifi: powered down: PERST# and WLAN_EN are down\n");
	}
	lck_mtx_unlock(&wifi_lock);
}

/* Any way back to the loader, or a failed step: the chip held in reset and unpowered (no lock, no sleep). */
void
apollo_wifi_quiesce(void)
{
	if (!started) {
		return;
	}
	/*
	 * The GIC's side of the chip's interrupts only: its MSI capability is not written here. That was a configuration
	 * read across a link that the line below had just taken down -- the bus hung on it, and with it the return to the
	 * loader and the log of whatever had ended the run (25 October, three runs). PERST# resets the chip's MSI anyway.
	 */
	w2c.msi_cap_skip = true;
	wifi_link_alive = false;                /* the BAR writers of the IPCR threads' paths stop here */
	__asm__ volatile ("dsb sy");
	w2c_msi_mask();                         /* before PERST# drops: no interrupt of a chip going away */
	/* (PARF only when it is clocked: a register of an unclocked block hangs the bus -- that ended a run) */
	if (clocked && parf != 0) {
		wr(parf + PARF_LTSSM, rd(parf + PARF_LTSSM) & ~(1u << 8));
	}
	pin_set(PIN_PERST, false);
	pin_set(PIN_WLAN_EN, false);
	wsmmu_off();
	started = false;
}

void
apollo_wifi_init(void)
{
	uint32_t stage = 0;
	thread_t thread;

	PE_parse_boot_argn("apollo_wifi_pause", &w2b_pause_ms, sizeof(w2b_pause_ms));
	if (!PE_parse_boot_argn("apollo_wifi", &stage, sizeof(stage)) || stage == 0) {
		return;
	}
	if (stage >= 5) {
		apollo_lx_reserve(64ul << 20);  /* the apollo build's POOL_SIZE (devices/mi-10t/tools/mk-ath11k.sh), while memory is still in large pieces */
		apollo_lx_net_early();          /* en0 before iOS's Wi-Fi side looks (wifi_station=1 with wifi_mac=) */
	}
	if (kernel_thread_start(apollo_wifi_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
