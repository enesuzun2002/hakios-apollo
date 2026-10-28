/*
 * The GPU of the Xiaomi Mi 10T (apollo), an Adreno 650 in the SM8250, woken step by step as redwood's
 * Adreno 642L is (bsd/dev/hakios.adreno.c, whose way of going about it this follows; nothing of it is
 * shared). Every address below belongs to this phone, and nothing here is touched unless boot-arg
 * apollo_gpu=N asks for it, on device=apollo only. Steps 1 to N are performed, then everything they
 * started is put back as it was found, in reverse.
 *
 *   1  reads   the GPU's clocks in the global clock controller, and the names RPMh knows its rails by
 *   2  reads   the fuse that says which of the SM8250's GPUs this is (its speed bin)
 *   3  reads   the GPU's own clock controller (0x3d90000), as the boot loader left it
 *   4  writes  the "CX" power switch on: the half of the GPU block that is not the 3D core -- the power
 *              management processor ("GMU"), the bus interface, the GPU's SMMU
 *   5  writes  the clocks of that half on (the GMU at 200 MHz), and measures with the SoC's frequency
 *              counter whether they run: the control register of the register bus's clock does not say
 *   6  reads   the first words from the GPU block itself: registers of the GMU (among them a counter
 *              that should be seen counting) and of the bus interface
 *   7  reads   the GMU's own channel to RPMh (its RSC, 0x3de0000)
 *   8  reads   the SMMU in front of the GPU: its ID registers, and which stream goes where
 *   9  reads   that SMMU's first translation context
 *  10  writes  two words into the GMU's memory and reads them back: the way its firmware goes
 *  11  asks    TrustZone questions that pass no memory: whether it takes calls from this kernel, and
 *              which firmware images it says it authenticates
 *  12  asks    TrustZone to check the signature of the GPU's "zap shader", the signed image without
 *              which the 3D core stays locked in secure mode: its header goes from the small image
 *              (fw/a650_zap.mdt, from ../fw-gpu-apollo) into memory of ours that TrustZone is pointed at
 *  13  writes  page tables for the memory the GMU's firmware asks for, and the SMMU: translation context 1
 *              with those tables, and the GMU's requests (stream 5) sent through it
 *  14  writes  the GMU's firmware (fw/a650_gmu.bin) into its own memory and into that memory
 *  15  writes  the GMU's configuration, and lets its processor out of reset: the firmware runs
 *  16  writes  the GMU's connection to RPMh; starts its message queues ("HFI")
 *  17  talks   to the GMU: the tables it needs (the 3D core's speeds and supply levels, bus votes), "start",
 *              the speed wanted, and the request to power the 3D core, whose registers are then read
 *  18  writes  the 3D core's translation context (SMMU context 0, stream 0), with its ring and its microcode
 *              (fw/a650_sqe.fw) in it
 *  19  asks    TrustZone to load the zap shader (fw/a650_zap.b02) into the memory reserved for it, and run it
 *  20  writes  the 3D core's registers as Linux's hw_init does for the a650; starts its command processor
 *              and gives it its first command
 *  21  commands the 3D core out of secure mode, and to write a word into a page of ours
 *  22  commands a picture filled with three colours, and shows it on the panel
 *
 * With apollo_gpu_use=1 (and apollo_gpu=21, steps 1 to 21: the picture of step 22 is skipped) the steps are
 * not undone: the GPU stays as step 21 leaves it, the 3D core powered and taking commands, and the service
 * ApolloGPU (iokit/Kernel/hakios.apollo_gpu.cpp) hands it to programs through the interface of Linux's msm driver,
 * which is what Mesa's freedreno speaks -- "the GPU kept on" below, a copy of hakios.adreno.c's, with
 * the a650's answers. The GPU is given back when the run ends (apollo_adreno_stop()).
 *
 * More boot-args:
 *   apollo_gpu_pause=MS  how long the newest step waits after saying what it is about to touch (default
 *                        300), for the loader to have the line before a silent reset could swallow it
 *   apollo_gpu_keep=1    leave CX and the clocks on at the end (for a look with a later run)
 *   apollo_gpu_dcvs=0    with apollo_gpu_use=1: the 3D core's speed stays at apollo_gpu_speed instead of following how
 *                        busy it is (the speed governor, "the GPU kept on" below); apollo_gpu_max=N: its top step (6)
 *   apollo_gpu_use=1     keep the GPU for the system to draw with (see above); gpu_recover=1, gpu_exact=0 and
 *                        gpu_log=1 as in redwood's (the names of the shared service's boot-args)
 *   apollo_gpu_speed=N   step 17 asks for the 3D core's Nth speed of this phone's (speed bin 0): 1 305 MHz,
 *                        2 400 MHz (the default), 3 441.6 MHz, 4 490 MHz, 5 525 MHz, 6 587 MHz
 *
 * Addresses are from the phone's own device tree (.device-local/apollo/device.dtb: qcom,kgsl-3d0@3d00000,
 * qcom,gmu@3d6a000, qcom,gpucc@3d90000, gdsc@3d9106c, kgsl-smmu@3da0000, the qfprom cell gpu_speed_bin),
 * register layouts from Xiaomi's apollo kernel (MiCode/Xiaomi_Kernel_OpenSource apollo-q-oss:
 * drivers/clk/qcom/gcc-kona.c, gpucc-kona.c, debugcc-kona.c) and Linux (gpucc-sm8250.c, gdsc.c,
 * clk-rcg2.c, drivers/gpu/drm/msm/adreno/a6xx_gmu.c with registers/adreno/a6xx{,_gmu}.xml, cmd-db.c).
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.apollo_input.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_offset_t ml_io_map_wcomb(vm_offset_t phys_addr, vm_size_t size);
extern uint64_t kvtophys(vm_offset_t va);       /* pmap_paddr_t */

static uint32_t last_step, pause_ms = 300, keep, use, speed_index = 2;
static uint32_t step;                   /* the one being performed */
static uint32_t speed_bin;
static bool quiet;                      /* accesses that have been seen to work, repeated: say nothing */

/* into the loader's log as well (as the other apollo drivers): a run that ends in a reset leaves no other */
extern void loader_log_putc(char c);

static int say(const char *format, ...) __printflike(1, 2);

static int
say(const char *format, ...)
{
	char line[400];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	return printf("%s", line);
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

/* ---- register access ------------------------------------------------------------------------------- */

/* one plain load or store of a general register (hakios.adreno.c: what a hypervisor's imitation can carry out) */
static uint32_t __attribute__((noinline))
rd(vm_offset_t address)
{
	uint32_t value;

	__asm__ volatile ("ldr %w0, [%1]" : "=r" (value) : "r" (address) : "memory");
	return value;
}

static uint32_t __attribute__((noinline))
rd8(vm_offset_t address)
{
	uint32_t value;

	__asm__ volatile ("ldrb %w0, [%1]" : "=r" (value) : "r" (address) : "memory");
	return value;
}

static void __attribute__((noinline))
wr(vm_offset_t address, uint32_t value)
{
	__asm__ volatile ("str %w0, [%1]" : : "r" (value), "r" (address) : "memory");
}

/* A stretch of registers; mapped when a step first needs it. */
struct block {
	const char *name;
	uint32_t phys, size;
	vm_offset_t base;
};

static void
need(struct block *b)
{
	if (b->base == 0) {
		b->base = ml_io_map(b->phys, b->size);
	}
}

/* time for the loader to have what has just been said, where it is not yet known to be harmless */
static void
hold(void)
{
	if (step == last_step && pause_ms) {
		sleep_ms(pause_ms);
	}
}

static void
announce(const char *what)
{
	say("apollo gpu: step %u: %s\n", step, what);
	hold();
}

static uint32_t
get(struct block *b, const char *name, uint32_t offset)
{
	uint32_t value;

	need(b);
	if (quiet) {
		return rd(b->base + offset);
	}
	value = rd(b->base + offset);
	say("apollo gpu: %u: %s %s (%#x) = %#x\n", step, b->name, name, b->phys + offset, value);
	return value;
}

static void
put(struct block *b, const char *name, uint32_t offset, uint32_t value)
{
	need(b);
	if (!quiet) {
		say("apollo gpu: %u: about to write %s %s at %#x: %#x\n", step, b->name, name, b->phys + offset, value);
	}
	wr(b->base + offset, value);
}

/* Look at a register until the masked bits have the value, for so many milliseconds. */
static bool
wait_for(struct block *b, uint32_t offset, uint32_t mask, uint32_t value, uint32_t ms)
{
	for (uint32_t i = 0; i < ms * 10; i++) {
		if ((rd(b->base + offset) & mask) == value) {
			return true;
		}
		sleep_us(100);
	}
	return (rd(b->base + offset) & mask) == value;
}

/*
 * What the steps change, to be put back in reverse when they are over: a register and the value it had.
 */
struct undo {
	struct block *b;
	const char *name;
	uint32_t offset, value;
};
static struct undo undo[32];
static unsigned undo_count;

/* write a register, noting what it held first (once: the first value is the one to go back to) */
static void
change(struct block *b, const char *name, uint32_t offset, uint32_t value)
{
	uint32_t was = get(b, name, offset);
	unsigned i;

	if (was == value) {
		return;
	}
	for (i = 0; i < undo_count && !(undo[i].b == b && undo[i].offset == offset); i++) {
	}
	if (i == undo_count && undo_count < sizeof(undo) / sizeof(undo[0])) {
		undo[undo_count++] = (struct undo){ b, name, offset, was };
	}
	put(b, name, offset, value);
}

/* ---- step 1: the clocks from outside, and RPMh's names ------------------------------------------------ */

/*
 * dt: qcom,gcc@100000 (gcc-kona.c). The GPU's clocks there, by the names the GPU's nodes give them:
 * gcc_gpu_memnoc_gfx and gcc_gpu_snoc_dvm_gfx (its bus connections), gcc_ddrss_gpu_axi (memory),
 * gcc_gpu_cfg_ahb (the registers of its clock controller), and the two taps of GPLL0 its clock
 * controller takes as inputs. The touch screen, the audio and the PCIe drivers use this controller too.
 */
static struct block gcc = { "gcc", 0x00100000, 0x80000 };
#define GCC_PLL_VOTE            0x52000         /* bit 15: gcc_gpu_gpll0_clk_src (600 MHz), bit 16: ..._div_clk_src (300 MHz) */
#define GCC_PLL_VOTE_GPU        (1u << 15)
#define GCC_PLL_VOTE_GPU_DIV    (1u << 16)
#define GCC_GPU_CFG_AHB_CBCR    0x71004
#define GCC_GPU_MEMNOC_GFX_CBCR 0x7100c
#define GCC_GPU_SNOC_DVM_GFX_CBCR 0x71018
#define GCC_DDRSS_GPU_AXI_CBCR  0x71154
#define CBCR_ENABLE             (1u << 0)       /* a clock's control register: on */
#define CBCR_OFF                (1u << 31)      /* ... and whether it is in fact off */

/* dt: reserved-memory cmd_db@80860000: the boot loader's list of what RPMh can switch (cmd-db.c) */
#define CMD_DB_PHYS             0x80860000UL
#define CMD_DB_SIZE             0x20000UL

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

/* where RPMh has a resource (0: nowhere) and, for a rail, the levels it knows (the GMU is told positions in it) */
static uint32_t
rpmh_resource(const char *name, uint16_t *levels, unsigned *level_count)
{
	const struct cmd_db_header *db = cmd_db;
	char id[8] = { 0 };

	if (level_count != NULL) {
		*level_count = 0;
	}
	if (db == NULL) {
		db = cmd_db = (const struct cmd_db_header *)ml_io_map_wcomb(CMD_DB_PHYS, CMD_DB_SIZE);  /* memory, not registers */
	}
	if (db->magic[0] != 0xdb || db->magic[1] != 0x30 || db->magic[2] != 0x03 || db->magic[3] != 0x0c) {
		return 0;
	}
	strlcpy(id, name, sizeof(id));
	for (unsigned s = 0; s < 8; s++) {
		const struct cmd_db_slave *slave = &db->slave[s];
		const struct cmd_db_entry *entry = (const struct cmd_db_entry *)(db->data + slave->header_offset);

		if (slave->id == 0 || slave->count > 512) {
			continue;
		}
		for (unsigned i = 0; i < slave->count; i++) {
			const uint8_t *data = db->data + slave->data_offset + entry[i].offset;
			unsigned count = entry[i].length / 2u;

			if (memcmp(entry[i].id, id, sizeof(id)) != 0) {
				continue;
			}
			if ((unsigned)slave->data_offset + entry[i].offset + entry[i].length > CMD_DB_SIZE - 0x1000) {
				count = 0;
			}
			for (unsigned l = 0; levels != NULL && l < count && l < 16; l++) {
				levels[l] = (uint16_t)(data[2 * l] | data[2 * l + 1] << 8);
				if (l > 0 && levels[l] == 0) {
					break;
				}
				*level_count = l + 1;
			}
			return entry[i].address;
		}
	}
	return 0;
}

static void
show_rpmh_resource(const char *name)
{
	uint16_t level[16];
	unsigned count = 0;
	uint32_t address = rpmh_resource(name, level, &count);
	char levels[160];
	int at = 0;

	if (address == 0) {
		say("apollo gpu: %u: RPMh has no \"%s\"\n", step, name);
		return;
	}
	levels[0] = 0;
	for (unsigned l = 0; l < count && at < (int)sizeof(levels) - 8; l++) {
		at += snprintf(levels + at, sizeof(levels) - (size_t)at, " %u", level[l]);
	}
	say("apollo gpu: %u: RPMh has \"%s\" at %#x; levels:%s\n", step, name, address, levels);
}

static bool
step_outside(void)
{
	announce("read the GPU's clocks in the global clock controller");
	get(&gcc, "pll votes", GCC_PLL_VOTE);
	get(&gcc, "gpu_cfg_ahb", GCC_GPU_CFG_AHB_CBCR);
	get(&gcc, "gpu_memnoc_gfx", GCC_GPU_MEMNOC_GFX_CBCR);
	get(&gcc, "gpu_snoc_dvm_gfx", GCC_GPU_SNOC_DVM_GFX_CBCR);
	get(&gcc, "ddrss_gpu_axi", GCC_DDRSS_GPU_AXI_CBCR);

	announce("look up the GPU's rails in the boot loader's list of RPMh resources (memory at 0x80860000)");
	show_rpmh_resource("gfx.lvl");          /* the 3D core's own rail (dt: VDD_GFX_LEVEL, pm8150a s1) */
	show_rpmh_resource("cx.lvl");           /* the CX half's, shared with most of the SoC */
	show_rpmh_resource("mx.lvl");           /* memories */
	return true;
}

/* ---- step 2: the fuse ---------------------------------------------------------------------------------- */

/*
 * dt: the qfprom cell gpu_speed_bin at 0x419b, bits 5 to 7 (Linux's sm8250.dtsi has the same cell:
 * qfprom@784000 + 0x19b). Which bin it is chooses the GPU's speeds (dt: qcom,gpu-pwrlevel-bins):
 * 0 up to 587 MHz, 1, 2 and 4 up to 670 MHz, 3 up to 525 MHz.
 */
static struct block qfprom = { "qfprom", 0x00780000, 0x7000 };
#define QFPROM_GPU_SPEED_BIN    0x419b

static bool
step_fuses(void)
{
	uint32_t raw;

	announce("read the GPU's speed bin fuse (0x78419b)");
	need(&qfprom);
	raw = rd8(qfprom.base + QFPROM_GPU_SPEED_BIN);
	speed_bin = raw >> 5 & 7;
	say("apollo gpu: %u: qfprom %#x = %#x: speed bin %u, an Adreno 650 good for %s MHz\n", step,
	    qfprom.phys + QFPROM_GPU_SPEED_BIN, raw, speed_bin, speed_bin == 0 ? "587" : speed_bin == 3 ? "525" :
	    speed_bin == 1 || speed_bin == 2 || speed_bin == 4 ? "670" : "(a bin the device tree does not know)");
	return true;
}

/* ---- step 3: the GPU's clock controller ----------------------------------------------------------------- */

/*
 * dt: qcom,gpucc@3d90000, 0x9000 long; offsets: gpucc-kona.c, Linux's gpucc-sm8250.c and the GDSCs of
 * the device tree. Its registers answer only while gcc_gpu_cfg_ahb runs. Unlike the 642L's, the SM8250's
 * has no "hub" and no "cb" clock.
 */
static struct block gpucc = { "gpucc", 0x03d90000, 0x9000 };
#define GPUCC_PLL0_MODE         0x0000          /* the 3D core's PLL: bit 31 locked, bit 0 output on */
#define GPUCC_PLL1_MODE         0x0100          /* a 500 MHz PLL, an alternative source for the GMU's clock */
#define GPUCC_CXO_AON_CBCR      0x1004
#define GPUCC_GX_SW_RESET       0x1008          /* dt: gpu_gx_gdsc sw-reset */
#define GPUCC_GX_GDSCR          0x100c          /* dt: gpu_gx_gdsc, the 3D core's power switch: the GMU's to operate */
#define GPUCC_GX_GMU_CBCR       0x1064
#define GPUCC_CX_GDSCR          0x106c          /* dt: gpu_cx_gdsc */
#define GPUCC_AHB_CBCR          0x1078          /* gpu_cc_ahb_clk: the register bus inside the GPU block */
#define GPUCC_CX_SNOC_DVM_CBCR  0x108c          /* gpu_cc_cx_snoc_dvm_clk */
#define GPUCC_CX_GMU_CBCR       0x1098          /* gpu_cc_cx_gmu_clk: the GMU */
#define GPUCC_CX_GMU_WAKE_SLEEP (0xfu << 8 | 0xfu << 4)  /* gpucc-kona.c's probe: "recommended WAKEUP/SLEEP settings" */
#define GPUCC_CXO_CBCR          0x109c          /* gpu_cc_cxo_clk: the crystal, for the GMU's timers */
#define GPUCC_GMU_RCGR          0x1120          /* gpu_cc_gmu_clk_src: +0 command, +4 source and divider */
#define GPUCC_GX_DOMAIN_MISC    0x1508          /* dt: gpu_gx_gdsc domain-addr (Linux: its clamp_io_ctrl) */
#define GPUCC_CX_HW_CTRL        0x1540          /* dt: gpu_cx_gdsc hw-ctrl-addr: bit 31 is where CX is seen on */
#define GPUCC_SMMU_VOTE_CBCR    0x5000          /* gpu_cc_hlos1_vote_gpu_smmu_clk: our vote for the SMMU's clock */

#define GDSC_SW_COLLAPSE        (1u << 0)       /* a power switch's control register (gdsc.c): off, by software */
#define GDSC_HW_CONTROL         (1u << 1)
#define GDSC_POWER_ON           (1u << 31)

static bool
gpucc_answers(void)
{
	need(&gcc);
	need(&gpucc);
	if (rd(gcc.base + GCC_GPU_CFG_AHB_CBCR) & CBCR_OFF) {
		say("apollo gpu: %u: the GPU clock controller's registers have no clock (gcc gpu_cfg_ahb %#x)\n", step,
		    rd(gcc.base + GCC_GPU_CFG_AHB_CBCR));
		return false;
	}
	return true;
}

static bool
step_gpucc(void)
{
	uint32_t ahb;

	announce("make sure the clock of the GPU clock controller's registers runs (gcc_gpu_cfg_ahb)");
	ahb = get(&gcc, "gpu_cfg_ahb", GCC_GPU_CFG_AHB_CBCR);
	if (!(ahb & CBCR_ENABLE)) {
		change(&gcc, "gpu_cfg_ahb", GCC_GPU_CFG_AHB_CBCR, ahb | CBCR_ENABLE);
	}
	if (!wait_for(&gcc, GCC_GPU_CFG_AHB_CBCR, CBCR_OFF, 0, 10)) {
		say("apollo gpu: %u: it stays off (%#x): the GPU's clock controller would not answer\n", step,
		    rd(gcc.base + GCC_GPU_CFG_AHB_CBCR));
		return false;
	}

	announce("read the GPU's clock controller (0x3d90000) as it is: the first access to the GPU's part of the SoC");
	get(&gpucc, "cx_gdscr", GPUCC_CX_GDSCR);
	get(&gpucc, "cx_hw_ctrl", GPUCC_CX_HW_CTRL);
	get(&gpucc, "gx_gdscr", GPUCC_GX_GDSCR);
	get(&gpucc, "gx_sw_reset", GPUCC_GX_SW_RESET);
	get(&gpucc, "gx_domain_misc", GPUCC_GX_DOMAIN_MISC);
	get(&gpucc, "pll0 mode", GPUCC_PLL0_MODE);
	get(&gpucc, "pll0 multiplier", GPUCC_PLL0_MODE + 4);
	get(&gpucc, "pll1 mode", GPUCC_PLL1_MODE);
	get(&gpucc, "pll1 multiplier", GPUCC_PLL1_MODE + 4);
	get(&gpucc, "gmu_clk_src command", GPUCC_GMU_RCGR);
	get(&gpucc, "gmu_clk_src source", GPUCC_GMU_RCGR + 4);
	get(&gpucc, "cxo_aon", GPUCC_CXO_AON_CBCR);
	get(&gpucc, "gx_gmu", GPUCC_GX_GMU_CBCR);
	get(&gpucc, "ahb", GPUCC_AHB_CBCR);
	get(&gpucc, "cx_snoc_dvm", GPUCC_CX_SNOC_DVM_CBCR);
	get(&gpucc, "cx_gmu", GPUCC_CX_GMU_CBCR);
	get(&gpucc, "cxo", GPUCC_CXO_CBCR);
	get(&gpucc, "smmu vote", GPUCC_SMMU_VOTE_CBCR);
	return true;
}

/* ---- step 4: CX power --------------------------------------------------------------------------------- */

static bool
cx_is_on(void)
{
	return (rd(gpucc.base + GPUCC_CX_GDSCR) | rd(gpucc.base + GPUCC_CX_HW_CTRL)) & GDSC_POWER_ON;
}

/*
 * The switch hangs on the rail cx.lvl, up for as long as the SoC runs at all; the vendor's driver
 * (gdsc-regulator.c) clears the collapse bit and waits for bit 31 of hw-ctrl-addr, Linux for bit 31 of
 * the control register itself.
 */
static bool
step_cx_power(void)
{
	uint32_t gdscr;

	if (!gpucc_answers()) {
		return false;
	}
	announce("switch the CX half of the GPU block on (gpu_cx_gdsc): clear the collapse bit of its power switch");
	gdscr = get(&gpucc, "cx_gdscr", GPUCC_CX_GDSCR);
	get(&gpucc, "cx_hw_ctrl", GPUCC_CX_HW_CTRL);
	if (cx_is_on()) {
		say("apollo gpu: %u: CX is on already (an earlier kernel, or the boot loader, left it so)\n", step);
	} else {
		if (gdscr & GDSC_HW_CONTROL) {
			say("apollo gpu: %u: the switch is under hardware control: not ours to operate\n", step);
			return false;
		}
		change(&gpucc, "cx_gdscr", GPUCC_CX_GDSCR, gdscr & ~GDSC_SW_COLLAPSE);
		for (int i = 0; i < 500 && !cx_is_on(); i++) {
			sleep_us(100);
		}
		get(&gpucc, "cx_gdscr", GPUCC_CX_GDSCR);
		get(&gpucc, "cx_hw_ctrl", GPUCC_CX_HW_CTRL);
		if (!cx_is_on()) {
			say("apollo gpu: %u: CX does not come on\n", step);
			return false;
		}
		say("apollo gpu: %u: CX IS ON\n", step);
	}
	if (rd(gpucc.base + GPUCC_GX_GDSCR) & GDSC_POWER_ON) {
		say("apollo gpu: %u: the 3D core (GX) has power already: left on by an earlier kernel\n", step);
	}
	return true;
}

/* ---- step 5: clocks ------------------------------------------------------------------------------------ */

#define RCG_UPDATE              (1u << 0)       /* command register (clk-rcg2.c): take the new source and divider */
#define RCG_SOURCE_MASK         0x371fu         /* [4:0] 2*divider-1, [10:8] source, [13:12] mode */
#define RCG_GPLL0_DIV(twice)    (6u << 8 | ((twice) - 1))       /* source 6: GPLL0's 300 MHz tap (gpucc-kona.c parent map) */

/* Turn a clock on and see it run. Not every clock's status can be believed (halt_check in Linux): `must` says which. */
static bool
clock_on(struct block *b, const char *name, uint32_t cbcr, bool must)
{
	uint32_t was = get(b, name, cbcr);

	if (!(was & CBCR_ENABLE)) {
		change(b, name, cbcr, was | CBCR_ENABLE);
	}
	if (wait_for(b, cbcr, CBCR_OFF, 0, 10)) {
		return true;
	}
	say("apollo gpu: %u: %s %s stays off (%#x)%s\n", step, b->name, name, rd(b->base + cbcr),
	    must ? "" : "; its status bit is not one to rely on, going on");
	return !must;
}

static bool
clock_source(const char *name, uint32_t rcgr, uint32_t source)
{
	uint32_t was = get(&gpucc, name, rcgr + 4);

	if ((was & RCG_SOURCE_MASK) == source) {
		return true;
	}
	change(&gpucc, name, rcgr + 4, (was & ~RCG_SOURCE_MASK) | source);
	put(&gpucc, name, rcgr, rd(gpucc.base + rcgr) | RCG_UPDATE);
	if (!wait_for(&gpucc, rcgr, RCG_UPDATE, 0, 10)) {
		say("apollo gpu: %u: %s does not take its new source (command %#x)\n", step, name, rd(gpucc.base + rcgr));
		return false;
	}
	get(&gpucc, name, rcgr + 4);
	return true;
}

/*
 * The SoC's frequency counter (debugcc-kona.c): a clock controller puts one of its clocks on its debug
 * output, the global controller counts what it is given there against the crystal / 4. gpu_cc_ahb has
 * "no bit to check" (BRANCH_HALT_DELAY): whether it runs is measured.
 */
#define GCC_DEBUG_MUX           0x62000         /* [9:0] which clock goes to the counter */
#define GCC_DEBUG_MUX_GPUCC     0x163           /* ... the GPU clock controller's debug output */
#define GCC_DEBUG_DIV           0x62004         /* [3:0] divider - 1 */
#define GCC_DEBUG_CBCR          0x62008
#define GCC_XO_DIV4_CBCR        0x4300c         /* the counter's time base: the crystal / 4, 4.8 MHz */
#define GCC_MEASURE_CTL         0x62038         /* [19:0] how many ticks of the time base to count for */
#define MEASURE_RUN             (1u << 20)
#define MEASURE_CLEAR           (1u << 21)
#define GCC_MEASURE_STATUS      0x6203c         /* [24:0] the count */
#define MEASURE_DONE            (1u << 25)
#define MEASURE_BRIEF           0x1000          /* ticks: a millisecond */
#define MEASURE_FULL            0x20000         /* 27 ms */
#define MEASURE_FAILED          0xffffffffu
#define GPUCC_DEBUG_DIV         0x10fc          /* [1:0] divider - 1 */
#define GPUCC_DEBUG_CBCR        0x1100
#define GPUCC_DEBUG_MUX         0x1568          /* [7:0] which of the GPU's clocks */
#define GPUCC_DEBUG_CXO_AON     0x0a
#define GPUCC_DEBUG_GX_GFX3D    0x0b            /* the 3D core's own clock */
#define GPUCC_DEBUG_GX_GMU      0x0f
#define GPUCC_DEBUG_AHB         0x10
#define GPUCC_DEBUG_CX_SNOC_DVM 0x15
#define GPUCC_DEBUG_CX_GMU      0x18
#define GPUCC_DEBUG_CXO         0x19

static uint32_t
count_clock(uint32_t ticks)
{
	uint32_t control = get(&gcc, "measure control", GCC_MEASURE_CTL) & ~(MEASURE_RUN | MEASURE_CLEAR | 0xfffffu);
	uint32_t status = 0;

	put(&gcc, "measure control", GCC_MEASURE_CTL, control | MEASURE_CLEAR);
	sleep_us(10);
	put(&gcc, "measure control", GCC_MEASURE_CTL, control | ticks);
	put(&gcc, "measure control", GCC_MEASURE_CTL, control | ticks | MEASURE_RUN);
	for (int i = 0; i < 2000 && !(status & MEASURE_DONE); i++) {
		sleep_us(100);
		status = rd(gcc.base + GCC_MEASURE_STATUS);
	}
	put(&gcc, "measure control", GCC_MEASURE_CTL, control | ticks);
	return status & MEASURE_DONE ? get(&gcc, "measure status", GCC_MEASURE_STATUS) & 0x1ffffff : MEASURE_FAILED;
}

/* the rate in Hz of one of the GPU clock controller's clocks; 0 if it stands still. Everything changed is put back. */
static uint32_t
measure(uint32_t select)
{
	uint32_t gpu_mux = get(&gpucc, "debug mux", GPUCC_DEBUG_MUX), gpu_div = get(&gpucc, "debug divider", GPUCC_DEBUG_DIV);
	uint32_t gpu_cbcr = get(&gpucc, "debug output", GPUCC_DEBUG_CBCR), mux = get(&gcc, "debug mux", GCC_DEBUG_MUX);
	uint32_t div = get(&gcc, "debug divider", GCC_DEBUG_DIV), cbcr = get(&gcc, "debug input", GCC_DEBUG_CBCR);
	uint32_t base = get(&gcc, "crystal/4", GCC_XO_DIV4_CBCR), control = get(&gcc, "measure control", GCC_MEASURE_CTL);
	uint32_t brief, full;

	put(&gpucc, "debug mux", GPUCC_DEBUG_MUX, (gpu_mux & ~0xffu) | select);
	put(&gpucc, "debug divider", GPUCC_DEBUG_DIV, (gpu_div & ~0x3u) | 1);
	put(&gpucc, "debug output", GPUCC_DEBUG_CBCR, gpu_cbcr | CBCR_ENABLE);
	put(&gcc, "debug mux", GCC_DEBUG_MUX, (mux & ~0x3ffu) | GCC_DEBUG_MUX_GPUCC);
	put(&gcc, "debug divider", GCC_DEBUG_DIV, (div & ~0xfu) | 1);
	put(&gcc, "debug input", GCC_DEBUG_CBCR, cbcr | CBCR_ENABLE);
	put(&gcc, "crystal/4", GCC_XO_DIV4_CBCR, base | CBCR_ENABLE);
	/* a counter whose input stands still keeps its last count: the two counts differ only if the clock runs */
	brief = count_clock(MEASURE_BRIEF);
	full = brief == MEASURE_FAILED ? MEASURE_FAILED : count_clock(MEASURE_FULL);
	put(&gcc, "measure control", GCC_MEASURE_CTL, control);
	put(&gcc, "crystal/4", GCC_XO_DIV4_CBCR, base);
	put(&gcc, "debug input", GCC_DEBUG_CBCR, cbcr);
	put(&gcc, "debug divider", GCC_DEBUG_DIV, div);
	put(&gcc, "debug mux", GCC_DEBUG_MUX, mux);
	put(&gpucc, "debug output", GPUCC_DEBUG_CBCR, gpu_cbcr);
	put(&gpucc, "debug divider", GPUCC_DEBUG_DIV, gpu_div);
	put(&gpucc, "debug mux", GPUCC_DEBUG_MUX, gpu_mux);

	if (full == MEASURE_FAILED) {
		return MEASURE_FAILED;
	}
	if (full == brief) {
		return 0;
	}
	/* the vendor's formula; times 4 for the two dividers on the way to the counter */
	return (uint32_t)(((uint64_t)full * 10 + 15) * 4800000 / ((uint64_t)MEASURE_FULL * 10 + 35) * 4);
}

static uint32_t
measure_quietly(uint32_t select)
{
	uint32_t hz;

	quiet = true;
	hz = measure(select);
	quiet = false;
	return hz;
}

/* The crystal's first and aloud: its rate is known, so it shows whether the counter can be believed. */
static bool
measure_clocks(void)
{
	static const struct {
		const char *name;
		uint32_t select;
	} clocks[] = {
		{ "cx_gmu (200 MHz was asked for)", GPUCC_DEBUG_CX_GMU }, { "cxo_aon", GPUCC_DEBUG_CXO_AON },
		{ "cx_snoc_dvm", GPUCC_DEBUG_CX_SNOC_DVM }, { "gx_gmu (should be off)", GPUCC_DEBUG_GX_GMU },
		{ "gx_gfx3d (should be off)", GPUCC_DEBUG_GX_GFX3D }, { "ahb, THE ONE IN QUESTION,", GPUCC_DEBUG_AHB },
	};
	uint32_t hz, ahb_hz = MEASURE_FAILED;

	announce("measure the GPU block's clocks with the SoC's frequency counter (gcc 0x162000.., gpucc 0x3d910fc, "
	    "0x3d91100, 0x3d91568)");
	hz = measure(GPUCC_DEBUG_CXO);
	if (hz == MEASURE_FAILED) {
		say("apollo gpu: %u: the counter did not finish: it cannot be used as it is set up here\n", step);
		return false;
	}
	say("apollo gpu: %u: measured: cxo runs at %u Hz (the crystal: 19200000)\n", step, hz);
	if (hz < 18600000 || hz > 19800000) {
		say("apollo gpu: %u: the counter does not show the crystal's rate: nothing it says can be used\n", step);
		return false;
	}
	for (unsigned i = 0; i < sizeof(clocks) / sizeof(clocks[0]); i++) {
		hz = measure_quietly(clocks[i].select);
		if (hz == MEASURE_FAILED) {
			say("apollo gpu: %u: measured: %s: the counter did not finish\n", step, clocks[i].name);
		} else {
			say("apollo gpu: %u: measured: %s runs at %u Hz\n", step, clocks[i].name, hz);
		}
		if (clocks[i].select == GPUCC_DEBUG_AHB) {
			ahb_hz = hz;
		}
	}
	if (ahb_hz != MEASURE_FAILED && ahb_hz >= 1000000) {
		say("apollo gpu: %u: THE REGISTER BUS OF THE GPU BLOCK HAS ITS CLOCK\n", step);
		return true;
	}
	say("apollo gpu: %u: the register bus of the GPU block has no clock: its registers are not to be touched\n", step);
	return false;
}

/*
 * The clocks the device tree gives the GMU and the SMMU (gmu_clk, cxo_clk, gpu_cc_ahb, axi_clk,
 * memnoc_clk; the SMMU's gcc_gpu_snoc_dvm_gfx), the GMU at the 200 MHz both the vendor's driver and
 * Linux start it with: GPLL0's 300 MHz tap / 1.5, a PLL that runs anyway.
 */
static bool
step_clocks(void)
{
	uint32_t gmu_clock;

	if (!gpucc_answers()) {
		return false;
	}
	if (!cx_is_on()) {
		say("apollo gpu: %u: CX is off: step 4\n", step);
		return false;
	}
	announce("in the global clock controller: GPLL0's taps for the GPU; the GPU's bus and memory clocks on");
	change(&gcc, "pll votes", GCC_PLL_VOTE, rd(gcc.base + GCC_PLL_VOTE) | GCC_PLL_VOTE_GPU | GCC_PLL_VOTE_GPU_DIV);
	if (!clock_on(&gcc, "gpu_memnoc_gfx", GCC_GPU_MEMNOC_GFX_CBCR, false) ||
	    !clock_on(&gcc, "gpu_snoc_dvm_gfx", GCC_GPU_SNOC_DVM_GFX_CBCR, true) ||
	    !clock_on(&gcc, "ddrss_gpu_axi", GCC_DDRSS_GPU_AXI_CBCR, false)) {
		return false;
	}

	announce("in the GPU's clock controller: the GMU's clock to 200 MHz");
	if (!clock_source("gmu_clk_src", GPUCC_GMU_RCGR, RCG_GPLL0_DIV(3))) {       /* 300 MHz / 1.5 */
		return false;
	}

	announce("in the GPU's clock controller: the clocks of the GMU and of the SMMU on");
	gmu_clock = get(&gpucc, "cx_gmu", GPUCC_CX_GMU_CBCR);
	if ((gmu_clock & GPUCC_CX_GMU_WAKE_SLEEP) != GPUCC_CX_GMU_WAKE_SLEEP) {
		change(&gpucc, "cx_gmu", GPUCC_CX_GMU_CBCR, gmu_clock | GPUCC_CX_GMU_WAKE_SLEEP);
	}
	if (!clock_on(&gpucc, "cxo", GPUCC_CXO_CBCR, true) ||
	    !clock_on(&gpucc, "ahb", GPUCC_AHB_CBCR, false) ||
	    !clock_on(&gpucc, "cx_snoc_dvm", GPUCC_CX_SNOC_DVM_CBCR, false) ||
	    !clock_on(&gpucc, "cx_gmu", GPUCC_CX_GMU_CBCR, true) ||
	    !clock_on(&gpucc, "smmu vote", GPUCC_SMMU_VOTE_CBCR, false)) {
		return false;
	}
	return measure_clocks();
}

/* What every later step stands on: CX powered, the GMU's clock running, the register bus's clock measured running. */
static bool
cx_ready(void)
{
	uint32_t gmu_clock, ahb_hz;

	if (!gpucc_answers()) {
		return false;
	}
	gmu_clock = rd(gpucc.base + GPUCC_CX_GMU_CBCR);
	if (!cx_is_on() || (gmu_clock & (CBCR_ENABLE | CBCR_OFF)) != CBCR_ENABLE) {
		say("apollo gpu: %u: needs CX on (it is %s) and the GMU's clock running (cx_gmu %#x): steps 4 and 5\n", step,
		    cx_is_on() ? "on" : "off", gmu_clock);
		return false;
	}
	ahb_hz = measure_quietly(GPUCC_DEBUG_AHB);
	if (ahb_hz == MEASURE_FAILED || ahb_hz < 1000000) {
		say("apollo gpu: %u: the GPU block's register bus clock does not run (%u Hz): not touching it\n", step, ahb_hz);
		return false;
	}
	return true;
}

/* ---- step 6: the first words from the GPU block ------------------------------------------------------- */

/* dt: qcom,gmu@3d6a000 "kgsl_gmu_reg", 0x30000 long. Offsets in a6xx_gmu.xml are in words, from here. */
static struct block gmu = { "gmu", 0x03d6a000, 0x30000 };
#define GMU_WORD(n)             (4u * (n))
#define GMU_DTCM_VERSIONS       GMU_WORD(0x1c00 + 0xff8) /* five words where loaded firmware carries its versions */
#define GMU_CM3_SYSRESET        GMU_WORD(0x5000)        /* 1: the processor is held in reset */
#define GMU_CM3_BOOT_CONFIG     GMU_WORD(0x5001)
#define GMU_CM3_FW_INIT_RESULT  GMU_WORD(0x501c)
#define GMU_CM3_CFG             GMU_WORD(0x502d)
#define GMU_ALWAYS_ON_COUNTER_L GMU_WORD(0x5088)        /* counts the 19.2 MHz crystal */
#define GMU_ALWAYS_ON_COUNTER_H GMU_WORD(0x5089)
#define GMU_RPMH_POWER_STATE    GMU_WORD(0x50ec)
#define GMU_HFI_CTRL_STATUS     GMU_WORD(0x5180)
#define GMU_HFI_VERSION_INFO    GMU_WORD(0x5181)
#define GMU_GENERAL_7           GMU_WORD(0x51cc)
#define GMU_AO_HOST_INTERRUPT_STATUS GMU_WORD(0x9305)
#define GMU_AO_GPU_CX_BUSY_STATUS GMU_WORD(0x930c)
#define GMU_AHB_FENCE_STATUS    GMU_WORD(0x9313)

/*
 * The 3D core (dt: kgsl_3d0_reg_memory at 0x3d00000) is not to be touched without power, which the GMU
 * gives it -- except its bus interface ("GBIF", words 0x3c00 on in a6xx.xml), which is in the CX half.
 */
static struct block gbif = { "gbif", 0x03d00000 + 4 * 0x3c00, 0x1000 };
#define GBIF_SCACHE_CNTL0       (4 * 0x01)
#define GBIF_SCACHE_CNTL1       (4 * 0x02)
#define GBIF_HALT               (4 * 0x45)
#define GBIF_HALT_ACK           (4 * 0x46)

static bool
step_first_words(void)
{
	uint64_t start, nanos;
	uint32_t before, hz;

	if (!cx_ready()) {
		return false;
	}
	announce("read registers of the GMU (0x3d6a000): the first words from the GPU block itself");
	get(&gmu, "cm3_sysreset", GMU_CM3_SYSRESET);
	get(&gmu, "cm3_boot_config", GMU_CM3_BOOT_CONFIG);
	get(&gmu, "cm3_fw_init_result", GMU_CM3_FW_INIT_RESULT);
	get(&gmu, "cm3_cfg", GMU_CM3_CFG);
	get(&gmu, "hfi_ctrl_status", GMU_HFI_CTRL_STATUS);
	get(&gmu, "hfi_version_info", GMU_HFI_VERSION_INFO);
	get(&gmu, "general_7", GMU_GENERAL_7);
	get(&gmu, "rpmh_power_state", GMU_RPMH_POWER_STATE);
	get(&gmu, "ao_host_interrupt_status", GMU_AO_HOST_INTERRUPT_STATUS);
	get(&gmu, "ao_gpu_cx_busy_status", GMU_AO_GPU_CX_BUSY_STATUS);
	get(&gmu, "ahb_fence_status", GMU_AHB_FENCE_STATUS);
	get(&gmu, "always_on_counter_h", GMU_ALWAYS_ON_COUNTER_H);
	get(&gmu, "always_on_counter_l", GMU_ALWAYS_ON_COUNTER_L);
	start = mach_absolute_time();
	before = rd(gmu.base + GMU_ALWAYS_ON_COUNTER_L);
	sleep_ms(10);
	hz = rd(gmu.base + GMU_ALWAYS_ON_COUNTER_L) - before;
	absolutetime_to_nanoseconds(mach_absolute_time() - start, &nanos);
	hz = nanos ? (uint32_t)((uint64_t)hz * NSEC_PER_SEC / nanos) : 0;
	say("apollo gpu: %u: the GMU's counter counts at %u Hz: %s\n", step, hz, hz > 18000000 && hz < 20500000 ?
	    "the 19.2 MHz crystal: THE GPU BLOCK IS ALIVE" : "not the crystal's 19.2 MHz (firmware may start it)");
	for (uint32_t i = 0; i < 5; i++) {
		get(&gmu, "dtcm, where firmware has its versions", GMU_DTCM_VERSIONS + 4 * i);
	}

	announce("read the GPU's bus interface (0x3d0f000)");
	get(&gbif, "scache_cntl0", GBIF_SCACHE_CNTL0);
	get(&gbif, "scache_cntl1", GBIF_SCACHE_CNTL1);
	get(&gbif, "halt", GBIF_HALT);
	get(&gbif, "halt_ack", GBIF_HALT_ACK);
	return true;
}

/* ---- step 7: the GMU's channel to RPMh ----------------------------------------------------------------- */

/*
 * dt: qcom,kgsl-3d0 "rscc" at 0x3de0000 (the a650 family has it apart from the GMU's registers;
 * Linux: a6xx_gmu.c, "rscc"): command slots through which the GMU's firmware raises the 3D core's
 * rail. Offsets in words: a6xx_gmu.xml, domain A6XX_RSCC.
 */
static struct block rscc = { "rscc", 0x03de0000, 0x2000 };
#define RSCC_RSC_STATUS0_DRV0   (4 * 0x004)
#define RSCC_PDC_SLAVE_ID_DRV0  (4 * 0x00b)
#define RSCC_SEQ_BUSY_DRV0      (4 * 0x101)
#define RSCC_SEQ_MEM_0_DRV0     (4 * 0x180)
#define RSCC_TCS0_DRV0_STATUS   (4 * 0x346)
#define GMU_RSCC_CONTROL_ACK    GMU_WORD(0x9308)

static bool
step_rscc(void)
{
	if (!cx_ready()) {
		return false;
	}
	announce("read the GMU's RPMh channel (0x3de0000)");
	get(&rscc, "rsc_status0_drv0", RSCC_RSC_STATUS0_DRV0);
	get(&rscc, "pdc_slave_id_drv0", RSCC_PDC_SLAVE_ID_DRV0);
	get(&rscc, "seq_busy_drv0", RSCC_SEQ_BUSY_DRV0);
	get(&rscc, "seq_mem_0_drv0", RSCC_SEQ_MEM_0_DRV0);
	get(&rscc, "tcs0_drv0_status", RSCC_TCS0_DRV0_STATUS);
	get(&gmu, "rscc_control_ack", GMU_RSCC_CONTROL_ACK);
	return true;
}

/* ---- steps 8 and 9: the SMMU --------------------------------------------------------------------------- */

/*
 * dt: kgsl-smmu@3da0000 (qcom,qsmmu-v500: an Arm MMU-500), 0x10000 long, "qcom,skip-init": the
 * vendor's driver leaves its stream table as it finds it. Its streams (dt: qcom,kgsl-iommu and
 * qcom,gmu "iommus"): 0 the GPU (mask 0x401), 2 secure content, 4 and 5 the GMU (mask 0x400).
 * Offsets: arm-smmu.h.
 */
static struct block smmu = { "smmu", 0x03da0000, 0x10000 };
#define SMMU_SCR0               0x000
#define SMMU_IDR0               0x020           /* [7:0] stream match registers */
#define SMMU_IDR1               0x024           /* bit 31: 64K pages, [30:28] log2(pages)-1, [7:0] translation contexts */
#define SMMU_IDR2               0x028
#define SMMU_IDR7               0x03c
#define SMMU_SGFSR              0x048
#define SMMU_SMR(n)             (0x800 + 4 * (n))       /* bit 31 valid, [30:16] mask, [14:0] stream */
#define SMMU_S2CR(n)            (0xc00 + 4 * (n))       /* [17:16] 0 translate 1 bypass 2 fault, [7:0] context */
#define SMMU_CBAR(n)            (4 * (n))               /* in the second page; [17:16] kind of context */
#define SMMU_CB_SCTLR           0x00            /* a context's own page: bit 0 = translation on */
#define SMMU_CB_TTBR0           0x20
#define SMMU_CB_TCR             0x30
#define SMMU_CB_FSR             0x58

static uint32_t smmu_page, smmu_pages, smmu_contexts, smmu_matches;

/* The layout of the SMMU's register space from its ID registers; false if it is not what the device tree says. */
static bool
smmu_layout(void)
{
	uint32_t idr0 = get(&smmu, "idr0", SMMU_IDR0), idr1 = get(&smmu, "idr1", SMMU_IDR1);

	smmu_page = idr1 >> 31 ? 0x10000 : 0x1000;
	smmu_pages = 2u << (idr1 >> 28 & 7);
	smmu_contexts = idr1 & 0xff;
	smmu_matches = idr0 & 0xff;
	say("apollo gpu: %u: SMMU: %u stream match registers, %u contexts, %u pages of %#x before the contexts' own\n", step,
	    smmu_matches, smmu_contexts, smmu_pages, smmu_page);
	if (2 * smmu_pages * smmu_page != smmu.size) {
		say("apollo gpu: %u: that makes %#x of registers, the device tree says %#x: not reading further\n", step,
		    2 * smmu_pages * smmu_page, smmu.size);
		return false;
	}
	return true;
}

static bool
step_smmu(void)
{
	if (!cx_ready()) {
		return false;
	}
	announce("read the ID registers of the GPU's SMMU (0x3da0000): global registers, which the hypervisor answers for");
	if (!smmu_layout()) {
		return false;
	}
	get(&smmu, "idr2", SMMU_IDR2);
	get(&smmu, "idr7", SMMU_IDR7);
	get(&smmu, "scr0", SMMU_SCR0);
	get(&smmu, "sgfsr", SMMU_SGFSR);

	announce("read the SMMU's stream table and the kinds of its contexts, as the boot loader and hypervisor left them");
	for (uint32_t i = 0; i < smmu_matches && i < 16; i++) {
		get(&smmu, "smr", SMMU_SMR(i));
		get(&smmu, "s2cr", SMMU_S2CR(i));
	}
	for (uint32_t i = 0; i < smmu_contexts && i < 16; i++) {
		get(&smmu, "cbar", smmu_page + SMMU_CBAR(i));
	}
	return true;
}

/* only context 0, the one both the vendor's driver and Linux give the GPU itself (others may be TrustZone's) */
static bool
step_smmu_context(void)
{
	uint32_t context;

	if (!cx_ready()) {
		return false;
	}
	announce("read translation context 0 of the SMMU: registers of the hardware itself, not the hypervisor's");
	if (!smmu_layout()) {
		return false;
	}
	context = smmu_pages * smmu_page;
	get(&smmu, "cb0 sctlr", context + SMMU_CB_SCTLR);
	get(&smmu, "cb0 ttbr0, low", context + SMMU_CB_TTBR0);
	get(&smmu, "cb0 ttbr0, high", context + SMMU_CB_TTBR0 + 4);
	get(&smmu, "cb0 tcr", context + SMMU_CB_TCR);
	get(&smmu, "cb0 fsr", context + SMMU_CB_FSR);
	return true;
}

/* ---- step 10: the GMU's memory -------------------------------------------------------------------------- */

/*
 * Firmware is loaded by writing it through these registers into the processor's memory while it is held
 * in reset (a6xx_gmu_fw_load). Two words of data memory (0x3d71000, a6xx_gmu.xml GMU_CM3_DTCM_START) go
 * there and back, and what was there is put back.
 */
#define GMU_DTCM                GMU_WORD(0x1c00)

static bool
step_gmu_memory(void)
{
	static const uint32_t pattern[2] = { 0x600d0001, 0x0a110002 };
	uint32_t was[2], back[2];

	if (!cx_ready()) {
		return false;
	}
	announce("hold the GMU's processor in reset, write two words into its data memory (0x3d71000), read them back");
	if (get(&gmu, "cm3_fw_init_result", GMU_CM3_FW_INIT_RESULT) == 0xbabeface) {
		say("apollo gpu: %u: firmware is running in the GMU: leaving its memory alone\n", step);
		return false;
	}
	put(&gmu, "cm3_sysreset", GMU_CM3_SYSRESET, 1);
	get(&gmu, "cm3_sysreset", GMU_CM3_SYSRESET);
	for (int i = 0; i < 2; i++) {
		was[i] = get(&gmu, "dtcm", GMU_DTCM + 4 * i);
	}
	for (int i = 0; i < 2; i++) {
		put(&gmu, "dtcm", GMU_DTCM + 4 * i, pattern[i]);
	}
	for (int i = 0; i < 2; i++) {
		back[i] = get(&gmu, "dtcm", GMU_DTCM + 4 * i);
	}
	for (int i = 0; i < 2; i++) {
		put(&gmu, "dtcm", GMU_DTCM + 4 * i, was[i]);
	}
	say("apollo gpu: %u: the GMU's memory %s\n", step, back[0] == pattern[0] && back[1] == pattern[1] ?
	    "TAKES WHAT IS WRITTEN: its firmware can be loaded" : "does not hold what was written");
	return back[0] == pattern[0] && back[1] == pattern[1];
}

/* ---- step 11: TrustZone -------------------------------------------------------------------------------- */

/*
 * The 3D core comes out of reset in "secure" mode, and only TrustZone lets it out: it does when handed the
 * vendor's signed zap shader (a6xx_gpu.c: a6xx_hw_init; qcom_scm.c: qcom_scm_pas_*; dt: qcom,kgsl-hyp,
 * qcom,pas-id 13). Questions that pass no memory first: whether it listens, whether the calls that
 * authenticate an image are there, and which images it says it "supports" -- the GPU's and, for
 * comparison, the audio DSP's (1, which hakios.apollo_adsp.c has it start), the modem's (4) and video's (9).
 * A call is an SMC, which the hypervisor passes on; convention: qcom_scm-smc.c.
 */
#define SCM_FAST                (1ull << 31)
#define SCM_64                  (1ull << 30)
#define SCM_SIP                 (2ull << 24)
#define SCM_CALL(service, command) ((uint64_t)(service) << 8 | (command))
#define SCM_IS_CALL_AVAILABLE   SCM_CALL(6, 1)
#define SCM_PAS_INIT_IMAGE      SCM_CALL(2, 1)
#define SCM_PAS_AUTH_AND_RESET  SCM_CALL(2, 5)
#define SCM_PAS_IS_SUPPORTED    SCM_CALL(2, 7)
#define SCM_ONE_VALUE           1
#define SCM_VALUE_AND_BUFFER    (2 | 2 << 6)    /* a number, and the address of memory it may read and write */
#define SCM_INTERRUPTED         1
#define PAS_GPU                 13

static int32_t
scm(const char *what, uint64_t function, uint64_t arguments, uint64_t first, uint64_t second, uint64_t third,
    uint64_t *answer)
{
	uint64_t session = 0;

	say("apollo gpu: %u: about to ask TrustZone %s: smc %#llx, arguments %#llx, %#llx, %#llx\n", step, what, function,
	    first, second, third);
	hold();
	for (int i = 0; i < 100; i++) {
		register uint64_t x0 __asm__("x0") = i == 0 ? function : SCM_INTERRUPTED;
		register uint64_t x1 __asm__("x1") = arguments;
		register uint64_t x2 __asm__("x2") = first;
		register uint64_t x3 __asm__("x3") = second;
		register uint64_t x4 __asm__("x4") = third;
		register uint64_t x5 __asm__("x5") = 0;
		register uint64_t x6 __asm__("x6") = session;
		register uint64_t x7 __asm__("x7") = 0;

		__asm__ volatile ("smc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3), "+r" (x4), "+r" (x5), "+r" (x6),
		    "+r" (x7) : : "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
		session = x6;
		if (x0 != SCM_INTERRUPTED) {
			say("apollo gpu: %u: TrustZone returns %d, answer %#llx\n", step, (int32_t)x0, x1);
			*answer = x1;
			return (int32_t)x0;
		}
	}
	say("apollo gpu: %u: TrustZone keeps saying the call was interrupted\n", step);
	return -1;
}

static bool
step_trustzone(void)
{
	static const struct {
		const char *name;
		uint32_t id;
	} images[] = { { "audio", 1 }, { "modem", 4 }, { "video", 9 }, { "GPU zap shader", PAS_GPU } };
	uint64_t answer = 0, how = SCM_FAST | SCM_64 | SCM_SIP;
	uint32_t fast[4] = { 0 }, slow[4] = { 0 };

	announce("ask TrustZone what Linux asks it first: is the call \"is this call available\" available");
	if (scm("whether it can be asked", how | SCM_IS_CALL_AVAILABLE, SCM_ONE_VALUE, SCM_SIP | SCM_IS_CALL_AVAILABLE, 0,
	    0, &answer) != 0 || answer != 1) {
		how = SCM_FAST | SCM_SIP;
		if (scm("the same, the 32-bit way", how | SCM_IS_CALL_AVAILABLE, SCM_ONE_VALUE, SCM_SIP | SCM_IS_CALL_AVAILABLE,
		    0, 0, &answer) != 0 || answer != 1) {
			say("apollo gpu: %u: TrustZone does not answer the way it answers Linux\n", step);
			return false;
		}
	}

	announce("ask TrustZone whether it has the calls that authenticate a firmware image");
	scm("for the call that takes an image's signature", how | SCM_IS_CALL_AVAILABLE, SCM_ONE_VALUE,
	    SCM_SIP | SCM_PAS_INIT_IMAGE, 0, 0, &answer);
	scm("for the call that authenticates and starts an image", how | SCM_IS_CALL_AVAILABLE, SCM_ONE_VALUE,
	    SCM_SIP | SCM_PAS_AUTH_AND_RESET, 0, 0, &answer);

	announce("ask TrustZone which images it \"supports\", each both ways: at once, and as a call that may be preempted");
	for (unsigned i = 0; i < 4; i++) {
		answer = 0;
		fast[i] = scm(images[i].name, how | SCM_PAS_IS_SUPPORTED, SCM_ONE_VALUE, images[i].id, 0, 0, &answer) == 0 && answer;
		answer = 0;
		slow[i] = scm(images[i].name, (how & ~SCM_FAST) | SCM_PAS_IS_SUPPORTED, SCM_ONE_VALUE, images[i].id, 0,
		    0, &answer) == 0 && answer;
	}
	say("apollo gpu: %u: \"supported\", asked at once / as Linux asks: audio %u/%u, modem %u/%u, video %u/%u, GPU %u/%u\n",
	    step, fast[0], slow[0], fast[1], slow[1], fast[2], slow[2], fast[3], slow[3]);
	say("apollo gpu: %u: %s\n", step, fast[3] || slow[3] ? "TrustZone lists the GPU's zap shader" :
	    "TrustZone does not list the GPU's: step 12 shows whether that matters");
	return true;
}

/* ---- step 12: the zap shader's signature ----------------------------------------------------------------- */

static void
to_memory(vm_offset_t start, vm_size_t size)
{
	for (vm_offset_t line = start & ~(vm_offset_t)63; line < start + size; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__asm__ volatile ("dsb sy");
}

/* a file of the small image that travels with the kernel (userland/build.sh puts ../fw-gpu-apollo into its fw/) */
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);

static const uint8_t *
image_file(const char *name, uint64_t *size)
{
	int disk = -1;
	const uint8_t *data = hakios_image_file("fw", name, size, &disk);      /* (hakios.image.c, shared) */

	if (data != NULL) {
		say("apollo gpu: %u: fw/%s: %llu bytes in the image of /dev/md%d\n", step, name, *size, disk);
	} else {
		say("apollo gpu: %u: no fw/%s in the small image: put the phone's /vendor/firmware/%s into fw-gpu-apollo/ and "
		    "rebuild the images\n", step, name, name);
	}
	return data;
}

/*
 * The first of the three calls with which Linux has a zap shader loaded (mdt_loader.c: qcom_mdt_pas_init;
 * qcom_scm.c: qcom_scm_pas_init_image): TrustZone is given the image's header -- the ELF header and the
 * block of hashes and certificates, all of a650_zap.mdt -- and checks the signature. Nothing of the GPU is
 * involved. The header goes in a page of ours that is not touched again, and never freed: TrustZone
 * protects it while it works.
 */
static bool
zap_header(uint64_t *phys)
{
	const uint8_t *header;
	vm_offset_t page = 0;
	uint64_t size = 0;
	uint32_t segments, hashes;

	header = image_file("a650_zap.mdt", &size);
	if (header == NULL) {
		return false;
	}
	/* a 32-bit ELF: the count of segments at 44, the table of them at 52, 32 bytes each, the file size of one at +16 */
	segments = size >= 52 + 2 * 32 ? header[44] | header[45] << 8 : 0;
	if (memcmp(header, "\177ELF\1", 5) != 0 || segments < 2 || size > PAGE_SIZE) {
		say("apollo gpu: %u: that is not such a header (%02x %02x %02x %02x, %u segments)\n", step, header[0], header[1],
		    header[2], header[3], segments);
		return false;
	}
	memcpy(&hashes, header + 52 + 32 + 16, sizeof(hashes));
	say("apollo gpu: %u: an ELF header of %u segments, %u bytes with its table, then %llu bytes; by the table, %u of hashes\n",
	    step, segments, 52 + 32 * segments, size - (52 + 32 * segments), hashes);
	if (52 + 32 * segments + (uint64_t)hashes != size) {
		say("apollo gpu: %u: the file is not the header followed by the hashes alone: not what TrustZone takes\n", step);
		return false;
	}
	if (kmem_alloc(kernel_map, &page, PAGE_SIZE, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		say("apollo gpu: %u: no memory\n", step);
		return false;
	}
	memcpy((void *)page, header, size);
	to_memory(page, PAGE_SIZE);
	*phys = kvtophys(page);
	return true;
}

static bool
step_zap_signature(void)
{
	uint64_t header = 0, answer = ~0ull;
	int32_t result;

	announce("find the zap shader's header, fw/a650_zap.mdt, in the small image that came with the kernel");
	if (!zap_header(&header)) {
		return false;
	}
	announce("hand the header to TrustZone to check its signature (image 13); a call that may be preempted, as Linux's is");
	result = scm("to take the zap shader's signature", SCM_64 | SCM_SIP | SCM_PAS_INIT_IMAGE, SCM_VALUE_AND_BUFFER, PAS_GPU,
	    header, 0, &answer);
	if (result == 0 && answer == 0) {
		say("apollo gpu: %u: TRUSTZONE ACCEPTS THE ZAP SHADER'S SIGNATURE FROM THIS KERNEL\n", step);
		return true;
	}
	say("apollo gpu: %u: TrustZone does not take it: the call returns %d, its answer is %#llx\n", step, result, answer);
	return false;
}

/* ---- memory the GPU block reaches ------------------------------------------------------------------- */

/*
 * Whatever in the GPU block reads or writes memory -- the GMU's processor, later the 3D core -- does it
 * through the SMMU, walking page tables of ours: three levels of 4K pages, 39 bits of address (dt:
 * qcom,use-3-lvl-tables). Formats and the order of programming: Linux io-pgtable-arm.c and arm-smmu.c
 * (as hakios.adreno.c has them).
 */
#define PTE_TABLE               3ull
#define PTE_PAGE                3ull
#define PTE_SHARED              (2ull << 8)
#define PTE_ACCESSED            (1ull << 10)
#define PTE_NOT_GLOBAL          (1ull << 11)
#define PTE_ADDRESS             0x0000fffffffff000ull
#define PTE_GMU                 (PTE_SHARED | PTE_ACCESSED | PTE_NOT_GLOBAL)

#define SMMU_S2CR_FAULT         (2u << 16)
#define SMMU_SMR_VALID          (1u << 31)
#define SMMU_CBAR_STAGE_1       (1u << 16 | 3u << 8 | 0xfu << 12)       /* our tables alone decide; weakest defaults */
#define SMMU_CBA2R(n)           (0x800 + 4 * (n))                       /* in the second page, like CBAR */
#define SMMU_CBA2R_64_BIT       1
#define SMMU_CB_TCR2            0x10
#define SMMU_CB_TTBR1           0x28
#define SMMU_CB_MAIR0           0x38
#define SMMU_CB_MAIR1           0x3c
#define SMMU_CB_FAR             0x60
#define SMMU_CB_FSYNR0          0x68
#define SMMU_CB_TLBIASID        0x610
#define SMMU_CB_TLBSYNC         0x7f0
#define SMMU_CB_TLBSTATUS       0x7f4
#define SMMU_SCTLR_ON           (1u << 0 | 1u << 1 | 1u << 2 | 1u << 5 | 1u << 12) /* on; faults recorded, no interrupt */
#define SMMU_TCR_39_BITS        (25u | 1u << 8 | 1u << 10 | 3u << 12 | 1u << 23)   /* one table of 4K pages, cached walks */
#define SMMU_TCR2               (1u | 1u << 4 | 7u << 15)       /* 36 bits of physical address, 16 of context tag */
#define SMMU_MAIR0              0xf404ff44      /* Linux's: 0 uncached, 1 cached, 2 device */
#define SMMU_FSR_ALL            0xc00001fe

/*
 * Every table of a space, of any level: enough for the whole of the first 4 GB, which is where the 3D core's
 * addresses are (the steps' below GPU_ADDRESSES_START, the clients' from there up to 4 GB: checked below,
 * where those are): 2048 tables of the last level (2 MB each), 4 above them and the root.
 */
#define SPACE_TABLES            2056

struct gpu_space {
	const char *name;
	uint32_t context;               /* which of the SMMU's */
	uint32_t mask;                  /* dt: the second cell of its stream's "iommus" */
	uint32_t control;               /* bits of its control register besides the usual */
	uint32_t table_count;
	uint64_t *root;
	struct {
		uint64_t phys;
		uint64_t *table;
		bool changed;           /* since it was last written out of this processor's cache (space_commit()) */
	} tables[SPACE_TABLES];
	uint64_t *last_entry, *last_table;      /* the level 2 entry looked up last, and its table */
	uint32_t last_changed;                  /* the table an entry was written in last */
};

static uint64_t *
space_new_table(struct gpu_space *space)
{
	static vm_offset_t spare;
	static unsigned spare_left;
	uint64_t *table;

	if (space->table_count == SPACE_TABLES) {
		return NULL;
	}
	if (spare_left == 0) {
		if (kmem_alloc(kernel_map, &spare, PAGE_SIZE, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) !=
		    KERN_SUCCESS) {
			return NULL;
		}
		spare_left = PAGE_SIZE / 4096;
	}
	table = (uint64_t *)(spare + PAGE_SIZE - 4096 * spare_left--);
	space->tables[space->table_count].phys = kvtophys((vm_offset_t)table);
	space->tables[space->table_count++].table = table;
	return table;
}

/* An entry of one of the tables has been written: that table is to go out to memory. */
static void
space_changed(struct gpu_space *space, const uint64_t *entry)
{
	if ((uintptr_t)entry - (uintptr_t)space->tables[space->last_changed].table < 4096) {
		space->tables[space->last_changed].changed = true;      /* page after page of one buffer */
		return;
	}
	for (uint32_t i = space->table_count; i-- > 0;) {
		if ((uintptr_t)entry - (uintptr_t)space->tables[i].table < 4096) {
			space->tables[i].changed = true;
			space->last_changed = i;
			return;
		}
	}
}

/* the SMMU reads the tables from memory, not from this processor's cache */
static void
space_commit(struct gpu_space *space)
{
	for (uint32_t i = 0; i < space->table_count; i++) {
		if (space->tables[i].changed) {
			to_memory((vm_offset_t)space->tables[i].table, 4096);
			space->tables[i].changed = false;
		}
	}
}

/* the table an entry of a higher table leads to; made if the entry is empty */
static uint64_t *
space_next(struct gpu_space *space, uint64_t *entry)
{
	uint64_t *table;

	if (*entry & 1) {
		for (uint32_t i = 0; i < space->table_count; i++) {
			if (space->tables[i].phys == (*entry & PTE_ADDRESS)) {
				return space->tables[i].table;
			}
		}
		return NULL;
	}
	table = space_new_table(space);
	if (table != NULL) {
		*entry = kvtophys((vm_offset_t)table) | PTE_TABLE;
		space_changed(space, entry);
	}
	return table;
}

/* one 4K page (the tables go out to memory when the context is given them: space_on(), later space_commit());
 * no physical address and no attributes: the page is taken out again */
static bool
space_map(struct gpu_space *space, uint64_t address, uint64_t phys, uint64_t attributes)
{
	uint64_t *level2, *level3;

	if (address >> 39 || (space->root == NULL && (space->root = space_new_table(space)) == NULL)) {
		return false;
	}
	level2 = space_next(space, &space->root[address >> 30 & 0x1ff]);
	if (level2 == NULL) {
		return false;
	}
	if (space->last_entry == &level2[address >> 21 & 0x1ff]) {
		level3 = space->last_table;             /* page after page of one buffer: the same table */
	} else {
		level3 = space_next(space, &level2[address >> 21 & 0x1ff]);
		space->last_entry = &level2[address >> 21 & 0x1ff];
		space->last_table = level3;
	}
	if (level3 == NULL) {
		space->last_entry = NULL;
		return false;
	}
	level3[address >> 12 & 0x1ff] = phys == 0 && attributes == 0 ? 0 : phys | attributes | PTE_PAGE;
	space_changed(space, level3);
	return true;
}

/* Memory of ours that the GPU block sees at an address of its own. */
struct gpu_block {
	uint64_t address;               /* the GPU's */
	uint32_t size;
	vm_offset_t memory;             /* ours */
};

static bool
block_new(struct gpu_space *space, struct gpu_block *block, uint64_t address, uint32_t size, uint64_t attributes)
{
	block->address = address;
	block->size = (size + 4095) & ~4095u;
	if (kmem_alloc(kernel_map, &block->memory, round_page(block->size), KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		return false;
	}
	for (uint32_t offset = 0; offset < block->size; offset += 4096) {
		if (!space_map(space, address + offset, kvtophys(block->memory + offset), attributes)) {
			return false;
		}
	}
	to_memory(block->memory, block->size);
	say("apollo gpu: %u: %s sees %#llx..%#llx in memory of ours (%#llx)\n", step, space->name, address,
	    address + block->size - 1, kvtophys(block->memory));
	return true;
}

/* a second view of a block without this processor's cache in between (blocks in one piece only) */
static vm_offset_t
block_uncached(const struct gpu_block *block)
{
	uint64_t phys = kvtophys(block->memory);

	for (uint32_t offset = 0; offset < block->size; offset += 4096) {
		if (kvtophys(block->memory + offset) != phys + offset) {
			return 0;
		}
	}
	return ml_io_map_wcomb(phys, block->size);
}

static uint32_t
smmu_context(const struct gpu_space *space)
{
	return (smmu_pages + space->context) * smmu_page;
}

/* what a context has recorded of a request it could not serve */
static bool
space_faulted(const struct gpu_space *space)
{
	uint32_t context = smmu_context(space), status = rd(smmu.base + context + SMMU_CB_FSR);

	if ((status & SMMU_FSR_ALL) == 0) {
		return false;
	}
	say("apollo gpu: %u: THE SMMU REFUSED %s: status %#x, address %#llx, syndrome %#x\n", step, space->name, status,
	    (uint64_t)rd(smmu.base + context + SMMU_CB_FAR + 4) << 32 | rd(smmu.base + context + SMMU_CB_FAR),
	    rd(smmu.base + context + SMMU_CB_FSYNR0));
	return true;
}

/* give a context our tables and send a stream through it; `match` is the entry of the stream table to use */
static bool
space_on(struct gpu_space *space, uint32_t match, uint32_t stream)
{
	uint32_t context = smmu_context(space), smr = SMMU_SMR_VALID | space->mask << 16 | stream;
	uint64_t root = space->tables[0].phys | (uint64_t)space->context << 48;        /* tagged with the context's number */

	for (uint32_t i = 0; i < space->table_count; i++) {
		to_memory((vm_offset_t)space->tables[i].table, 4096);
	}
	put(&smmu, "cb sctlr", context + SMMU_CB_SCTLR, 0);
	put(&smmu, "cba2r", smmu_page + SMMU_CBA2R(space->context), SMMU_CBA2R_64_BIT);
	put(&smmu, "cbar", smmu_page + SMMU_CBAR(space->context), SMMU_CBAR_STAGE_1);
	put(&smmu, "cb tcr2", context + SMMU_CB_TCR2, SMMU_TCR2);
	put(&smmu, "cb tcr", context + SMMU_CB_TCR, SMMU_TCR_39_BITS);
	put(&smmu, "cb ttbr0, low", context + SMMU_CB_TTBR0, (uint32_t)root);
	put(&smmu, "cb ttbr0, high", context + SMMU_CB_TTBR0 + 4, (uint32_t)(root >> 32));
	put(&smmu, "cb ttbr1, low", context + SMMU_CB_TTBR1, 0);
	put(&smmu, "cb ttbr1, high", context + SMMU_CB_TTBR1 + 4, 0);
	put(&smmu, "cb mair0", context + SMMU_CB_MAIR0, SMMU_MAIR0);
	put(&smmu, "cb mair1", context + SMMU_CB_MAIR1, 0);
	put(&smmu, "cb fsr", context + SMMU_CB_FSR, SMMU_FSR_ALL);
	put(&smmu, "cb tlbiasid", context + SMMU_CB_TLBIASID, space->context);
	put(&smmu, "cb tlbsync", context + SMMU_CB_TLBSYNC, 1);
	if (!wait_for(&smmu, context + SMMU_CB_TLBSTATUS, 1, 0, 100)) {
		say("apollo gpu: %u: the SMMU does not finish forgetting old translations (%#x)\n", step,
		    rd(smmu.base + context + SMMU_CB_TLBSTATUS));
		return false;
	}
	put(&smmu, "cb sctlr", context + SMMU_CB_SCTLR, SMMU_SCTLR_ON | space->control);
	put(&smmu, "s2cr", SMMU_S2CR(match), space->context);
	put(&smmu, "smr", SMMU_SMR(match), smr);

	/* the global ones are the hypervisor's to grant: see what it made of them */
	if (get(&smmu, "smr", SMMU_SMR(match)) != smr ||
	    (get(&smmu, "s2cr", SMMU_S2CR(match)) & 0x300ff) != space->context ||
	    (get(&smmu, "cbar", smmu_page + SMMU_CBAR(space->context)) & 0x30000) != (SMMU_CBAR_STAGE_1 & 0x30000) ||
	    !(get(&smmu, "cb sctlr", context + SMMU_CB_SCTLR) & 1) ||
	    get(&smmu, "cb ttbr0, low", context + SMMU_CB_TTBR0) != (uint32_t)root ||
	    get(&smmu, "cb ttbr0, high", context + SMMU_CB_TTBR0 + 4) != (uint32_t)(root >> 32)) {
		say("apollo gpu: %u: the SMMU did not take what was written: stream %u is not going through context %u\n", step,
		    stream, space->context);
		return false;
	}
	get(&smmu, "cb tcr", context + SMMU_CB_TCR);
	get(&smmu, "cba2r", smmu_page + SMMU_CBA2R(space->context));
	get(&smmu, "cb fsr", context + SMMU_CB_FSR);
	get(&smmu, "sgfsr", SMMU_SGFSR);
	return true;
}

/* a stream goes nowhere again, and its context translates nothing */
static void
space_off(struct gpu_space *space, uint32_t match)
{
	put(&smmu, "smr", SMMU_SMR(match), 0);
	put(&smmu, "s2cr", SMMU_S2CR(match), SMMU_S2CR_FAULT);
	put(&smmu, "cb sctlr", smmu_context(space) + SMMU_CB_SCTLR, 0);
}

/* ---- step 13: memory for the GMU ------------------------------------------------------------------------ */

/*
 * The GMU is a Cortex-M3 with 16K of code memory and 16K of data memory of its own (step 10); the rest
 * of its firmware and everything it shares with us it reaches through the SMMU, at addresses the
 * firmware file names: a file of blocks, each "this goes at address A" or, with no data, "I need N
 * bytes at address A" (a6xx_gmu.c: a6xx_gmu_fw_load). a650_gmu.bin asks for 0x8000 at 0x4000, 0x13000
 * at 0x144000, 0x9000 at 0x157000 and a page at 0x60000000. Besides those: 16K for the firmware's log,
 * and a page for the table and queues of the message protocol ("HFI").
 */
#define GMU_ITCM_ADDRESS        0x00000000u
#define GMU_DTCM_ADDRESS        0x10004000u     /* a6xx_gmu_fw_load: the 650 family's */
#define GMU_TCM_SIZE            0x4000u
#define GMU_LOG_ADDRESS         0x60002000u
#define GMU_LOG_SIZE            0x4000u
#define GMU_HFI_ADDRESS         0x60006000u
#define GMU_STREAM              5               /* dt: qcom,gmu gmu_kernel, iommus <5 0x400> */
#define GMU_MATCH               1               /* entry of the stream table; 0 is kept for the 3D core */

struct gmu_firmware_block {
	uint32_t address, size, type, value;
};
#define GMU_FIRMWARE_NEEDS      1               /* types of blocks without data: memory wanted, value = bytes */
#define GMU_FIRMWARE_NEEDS_KEPT 7

static struct gpu_space gmu_space = { .name = "the GMU", .context = 1, .mask = 0x400 };
static struct gpu_block gmu_blocks[12], gmu_log, gmu_hfi;
static unsigned gmu_block_count;
static const uint8_t *gmu_firmware;
static uint64_t gmu_firmware_size;
static bool gmu_space_is_on, gmu_runs, gmu_talks, rsc_awake;

static bool
in_tcm(uint32_t address)
{
	return address - GMU_ITCM_ADDRESS < GMU_TCM_SIZE || address - GMU_DTCM_ADDRESS < GMU_TCM_SIZE;
}

static bool
step_gmu_space(void)
{
	if (!cx_ready()) {
		return false;
	}
	announce("find the GMU's firmware, fw/a650_gmu.bin, in the small image, and make the memory it asks for");
	gmu_firmware = image_file("a650_gmu.bin", &gmu_firmware_size);
	if (gmu_firmware == NULL) {
		return false;
	}
	for (uint64_t at = 0; at + sizeof(struct gmu_firmware_block) <= gmu_firmware_size;) {
		struct gmu_firmware_block block;

		memcpy(&block, gmu_firmware + at, sizeof(block));
		at += sizeof(block) + block.size;
		if (at > gmu_firmware_size) {
			say("apollo gpu: %u: the file ends inside a block\n", step);
			return false;
		}
		if (block.size != 0 || (block.type != GMU_FIRMWARE_NEEDS && block.type != GMU_FIRMWARE_NEEDS_KEPT) ||
		    in_tcm(block.address)) {
			continue;
		}
		if (gmu_block_count == sizeof(gmu_blocks) / sizeof(gmu_blocks[0]) || (block.address & 4095) ||
		    block.value > 0x100000 || !block_new(&gmu_space, &gmu_blocks[gmu_block_count++], block.address, block.value,
		    PTE_GMU)) {
			say("apollo gpu: %u: cannot make %#x bytes at %#x for the firmware\n", step, block.value, block.address);
			return false;
		}
	}
	if (!block_new(&gmu_space, &gmu_log, GMU_LOG_ADDRESS, GMU_LOG_SIZE, PTE_GMU) ||
	    !block_new(&gmu_space, &gmu_hfi, GMU_HFI_ADDRESS, PAGE_SIZE, PTE_GMU)) {
		return false;
	}

	announce("give the SMMU's context 1 these tables and send the GMU's requests (stream 5) through it: the first "
	    "writes to the SMMU, the global ones (cbar, cba2r, s2cr, smr) through the hypervisor");
	if (!smmu_layout()) {
		return false;
	}
	gmu_space_is_on = true;         /* whatever of it has been written by the time something fails is taken back */
	return space_on(&gmu_space, GMU_MATCH, GMU_STREAM);
}

/* ---- step 14: the GMU's firmware ----------------------------------------------------------------------- */

#define GMU_ITCM                GMU_WORD(0x0c00)

static bool
step_gmu_firmware(void)
{
	uint32_t wrong = 0;

	if (!cx_ready() || gmu_firmware == NULL) {
		return false;
	}
	announce("load the GMU's firmware: its code and data memory through the GMU's registers (0x3d6d000, 0x3d71000), "
	    "the rest into the memory just made");
	put(&gmu, "cm3_sysreset", GMU_CM3_SYSRESET, 1);
	for (int pass = 0; pass < 2; pass++) {
		for (uint64_t at = 0; at + sizeof(struct gmu_firmware_block) <= gmu_firmware_size;) {
			struct gmu_firmware_block block;
			const uint8_t *data = gmu_firmware + at + sizeof(block);

			memcpy(&block, gmu_firmware + at, sizeof(block));
			at += sizeof(block) + block.size;
			if (block.size == 0) {
				continue;
			}
			/* (ends worked out in 64 bits: a block's address and size are the file's to say, and must not wrap) */
			if (in_tcm(block.address) && (uint64_t)block.address + block.size - 1 <= 0xffffffffull &&
			    in_tcm(block.address + block.size - 1) &&
			    (block.address < GMU_DTCM_ADDRESS) == (block.address + block.size - 1 < GMU_DTCM_ADDRESS)) {
				uint32_t registers = block.address < GMU_DTCM_ADDRESS ? GMU_ITCM + block.address - GMU_ITCM_ADDRESS :
				    GMU_DTCM + block.address - GMU_DTCM_ADDRESS;

				for (uint32_t offset = 0; offset + 4 <= block.size; offset += 4) {
					uint32_t word;

					memcpy(&word, data + offset, sizeof(word));
					if (pass == 0) {
						wr(gmu.base + registers + offset, word);
					} else if (rd(gmu.base + registers + offset) != word) {
						wrong++;
					}
				}
				if (pass == 1) {
					say("apollo gpu: %u: %#x bytes at the GMU's %#x, through its registers\n", step, block.size,
					    block.address);
				}
				continue;
			}
			for (unsigned i = 0; pass == 0 && i <= gmu_block_count; i++) {
				if (i == gmu_block_count) {
					say("apollo gpu: %u: the firmware has %#x bytes for %#x, where it asked for no memory\n", step,
					    block.size, block.address);
					return false;
				}
				if (block.address >= gmu_blocks[i].address &&
				    (uint64_t)block.address + block.size <= gmu_blocks[i].address + gmu_blocks[i].size) {
					memcpy((void *)(gmu_blocks[i].memory + (block.address - gmu_blocks[i].address)), data, block.size);
					to_memory(gmu_blocks[i].memory, gmu_blocks[i].size);
					say("apollo gpu: %u: %#x bytes at the GMU's %#x, in memory\n", step, block.size, block.address);
					break;
				}
			}
		}
	}
	if (wrong) {
		say("apollo gpu: %u: %u words of the GMU's own memory do not read back as written\n", step, wrong);
		return false;
	}
	say("apollo gpu: %u: THE GMU'S FIRMWARE IS LOADED; its versions, read back from the GMU: core %#x, power %#x, HFI %#x\n",
	    step, rd(gmu.base + GMU_DTCM_VERSIONS), rd(gmu.base + GMU_DTCM_VERSIONS + 8), rd(gmu.base + GMU_DTCM_VERSIONS + 16));
	return true;
}

/* ---- step 15: start the GMU ---------------------------------------------------------------------------- */

#define GMU_ICACHE_CONFIG       GMU_WORD(0x4c00)
#define GMU_DCACHE_CONFIG       GMU_WORD(0x4c01)
#define GMU_SYS_BUS_CONFIG      GMU_WORD(0x4c0f)
#define GMU_PWR_COL_INTER_FRAME_CTRL GMU_WORD(0x50c0)
#define GMU_RPMH_CTRL           GMU_WORD(0x50e8)
#define GMU_RPMH_CTRL_ALL       0xf11           /* the RPMh interface, and every vote the firmware may cast */
#define GMU_CX_FAL_INTF         GMU_WORD(0x50f0)
#define GMU_CX_FALNEXT_INTF     GMU_WORD(0x50f1)
#define GMU_PWR_COL_CP_MSG      GMU_WORD(0x5100)        /* at start: where the log is, and its pages - 1 */
#define GMU_PWR_COL_CP_RESP     GMU_WORD(0x5101)        /* ... and how far it was written before */
#define GMU_HFI_SFR_ADDR        GMU_WORD(0x5182)        /* at start: which GPU this is */
#define GMU_HFI_QTBL_INFO       GMU_WORD(0x5184)
#define GMU_HFI_QTBL_ADDR       GMU_WORD(0x5185)
#define GMU_HFI_CTRL_INIT       GMU_WORD(0x5186)
#define GMU_GMU2HOST_INTR_CLR   GMU_WORD(0x5191)
#define GMU_GMU2HOST_INTR_INFO  GMU_WORD(0x5192)
#define GMU_RSCC_CONTROL_REQ    GMU_WORD(0x9307)
#define GMU_AHB_FENCE_RANGE_0   GMU_WORD(0x9311)
#define GMU_AHB_FENCE           (1u << 31 | 0xau << 18 | 0xa0)  /* a6xx_gmu_fw_start, for the a6xx parts */
#define GMU_CHIP                0x06050200      /* dt qcom,chipid 0x06050002 the way a6xx_gmu_fw_start encodes it */
#define GMU_OLD_FIRMWARE        0x20010004      /* a core version up to this says 0xbabeface when it has started */
#define GMU_STARTED_MASK        0x1ff           /* ... a newer one says 0x100 in these bits */
#define GMU_STARTED             0x100
#define GMU_FAILED              0xe00
#define GBIF_QSB_SIDE0          (4 * 0x03)      /* a6xx_catalog.c a640_gbif, the a650's: written before the GMU starts */
#define RSCC_ASLEEP             (1u << 16)      /* of rsc_status0: handed over to the always-on sequencer */

/* The table at the head of the HFI page and the three queues after it, as the vendor's driver lays them out. */
struct hfi_queue {
	uint32_t status, start, type, words, message_words, unused[5], read, write;
};
struct hfi_table {
	uint32_t version, words, first_queue, queue_words, queues, active_queues;
	struct hfi_queue queue[17];
};
#define HFI_QUEUES              3               /* 0: ours to the GMU; 1: its answers; 2: its debugging messages */
#define HFI_QUEUE_WORDS         1024
#define HFI_QUEUE_AT(n)         (((uint32_t)sizeof(struct hfi_table) + 15) / 16 * 16 + 4096 * (n))

static volatile struct hfi_table *hfi;  /* the uncached view */

static void
rscc_show(void)
{
	get(&rscc, "rsc_status0_drv0", RSCC_RSC_STATUS0_DRV0);
	get(&rscc, "seq_busy_drv0", RSCC_SEQ_BUSY_DRV0);
	get(&rscc, "seq_mem_0_drv0", RSCC_SEQ_MEM_0_DRV0);
	get(&gmu, "rscc_control_ack", GMU_RSCC_CONTROL_ACK);
}

/* what can be seen of a GMU that does not come up: the tail of its log (16 bytes an event), its RSC, its state */
static void
gmu_post_mortem(void)
{
	uint32_t next = rd(gmu.base + GMU_PWR_COL_CP_RESP) & 0xffff, entries = GMU_LOG_SIZE / 16;
	const volatile uint32_t *log = (const volatile uint32_t *)gmu_log.memory;

	to_memory(gmu_log.memory, GMU_LOG_SIZE);
	say("apollo gpu: %u: the GMU's log (event, time, two values), %u events in, the last 20:\n", step, next);
	for (uint32_t i = next > 20 ? next - 20 : 0; i < next && i < entries; i++) {
		say("apollo gpu: %u:   %x %x %x %x\n", step, log[4 * i], log[4 * i + 1], log[4 * i + 2], log[4 * i + 3]);
	}
	rscc_show();
	get(&gpucc, "gx_gdscr", GPUCC_GX_GDSCR);
	get(&gmu, "rpmh_power_state", GMU_RPMH_POWER_STATE);
	get(&gmu, "ao_host_interrupt_status", GMU_AO_HOST_INTERRUPT_STATUS);
	get(&gmu, "ahb_fence_status", GMU_AHB_FENCE_STATUS);
	space_faulted(&gmu_space);
}

/*
 * What Linux writes before it lets the processor go (a6xx_gmu_fw_start, a6xx_gmu_power_config), in its
 * order, with the a650's values; then out of reset (a6xx_gmu_start).
 */
static bool
step_gmu_start(void)
{
	static const uint8_t priority[HFI_QUEUES] = { 10, 10, 40 };
	uint32_t result = 0, version, mask, started;

	if (!cx_ready() || !gmu_space_is_on) {
		return false;
	}
	hfi = (volatile struct hfi_table *)block_uncached(&gmu_hfi);
	if (hfi == NULL) {
		say("apollo gpu: %u: the page for the messages is not in one piece\n", step);
		return false;
	}
	hfi->words = sizeof(struct hfi_table) / 4;
	hfi->first_queue = 6;
	hfi->queue_words = sizeof(struct hfi_queue) / 4;
	hfi->queues = hfi->active_queues = HFI_QUEUES;
	for (unsigned i = 0; i < HFI_QUEUES; i++) {
		hfi->queue[i].start = GMU_HFI_ADDRESS + HFI_QUEUE_AT(i);
		hfi->queue[i].type = i | (uint32_t)priority[i] << 8;
		hfi->queue[i].words = HFI_QUEUE_WORDS;
		hfi->queue[i].status = 1;
	}

	announce("tell the GMU where things are, by its registers: the message table, the log, which GPU this is; its "
	    "caches and its use of RPMh; then let its processor out of reset");
	if (get(&gmu, "cm3_sysreset", GMU_CM3_SYSRESET) != 1) {
		say("apollo gpu: %u: the GMU's processor is not held in reset: not configuring it while it runs\n", step);
		return false;
	}
	/* a6xx_rsc_wakeup: the GMU's RPMh channel taken back from the always-on sequencer, should it have it */
	rscc_show();
	if (rd(rscc.base + RSCC_RSC_STATUS0_DRV0) & RSCC_ASLEEP) {     /* (as a run before this one leaves it: gmu_stop()) */
		put(&gmu, "rscc_control_req", GMU_RSCC_CONTROL_REQ, 2);
		if (!wait_for(&gmu, GMU_RSCC_CONTROL_ACK, 2, 2, 100) || !wait_for(&rscc, RSCC_SEQ_BUSY_DRV0, ~0u, 0, 100)) {
			say("apollo gpu: %u: the GMU's RPMh channel does not wake\n", step);
		}
		get(&gmu, "rscc_control_ack", GMU_RSCC_CONTROL_ACK);
		put(&gmu, "rscc_control_req", GMU_RSCC_CONTROL_REQ, 0);
	}
	rsc_awake = true;
	put(&gmu, "cx_fal_intf", GMU_CX_FAL_INTF, 1);
	put(&gmu, "cx_falnext_intf", GMU_CX_FALNEXT_INTF, 1);
	put(&gmu, "general_7", GMU_GENERAL_7, 1);
	put(&gmu, "pwr_col_cp_resp", GMU_PWR_COL_CP_RESP, 0);
	put(&gmu, "cm3_fw_init_result", GMU_CM3_FW_INIT_RESULT, 0);
	put(&gmu, "cm3_boot_config", GMU_CM3_BOOT_CONFIG, 2);
	put(&gmu, "hfi_qtbl_addr", GMU_HFI_QTBL_ADDR, GMU_HFI_ADDRESS);
	put(&gmu, "hfi_qtbl_info", GMU_HFI_QTBL_INFO, 1);
	put(&gmu, "ahb_fence_range_0", GMU_AHB_FENCE_RANGE_0, GMU_AHB_FENCE);
	put(&gmu, "cm3_cfg", GMU_CM3_CFG, 0x4052);
	put(&gmu, "hfi_sfr_addr", GMU_HFI_SFR_ADDR, GMU_CHIP);
	put(&gmu, "pwr_col_cp_msg", GMU_PWR_COL_CP_MSG, GMU_LOG_ADDRESS | (GMU_LOG_SIZE / 4096 - 1));
	for (uint32_t i = 0; i < 4; i++) {
		put(&gbif, "qsb_side", GBIF_QSB_SIDE0 + 4 * i, 0x00071620);
	}
	put(&gmu, "sys_bus_config", GMU_SYS_BUS_CONFIG, 1);
	put(&gmu, "icache_config", GMU_ICACHE_CONFIG, 1);
	put(&gmu, "dcache_config", GMU_DCACHE_CONFIG, 1);
	put(&gmu, "pwr_col_inter_frame_ctrl", GMU_PWR_COL_INTER_FRAME_CTRL, 0x9c40400);
	put(&gmu, "rpmh_ctrl", GMU_RPMH_CTRL, rd(gmu.base + GMU_RPMH_CTRL) | GMU_RPMH_CTRL_ALL);
	put(&gmu, "gmu2host_intr_clr", GMU_GMU2HOST_INTR_CLR, 0xffffffff);

	version = rd(gmu.base + GMU_DTCM_VERSIONS);
	mask = version <= GMU_OLD_FIRMWARE ? 0xffffffffu : GMU_STARTED_MASK;
	started = version <= GMU_OLD_FIRMWARE ? 0xbabeface : GMU_STARTED;
	gmu_runs = true;
	put(&gmu, "cm3_sysreset", GMU_CM3_SYSRESET, 0);
	for (int i = 0; i < 5000 && (result & mask) != started && !(mask != 0xffffffffu && (result & GMU_FAILED)); i++) {
		sleep_us(100);
		result = rd(gmu.base + GMU_CM3_FW_INIT_RESULT);
	}
	get(&gmu, "cm3_fw_init_result", GMU_CM3_FW_INIT_RESULT);
	get(&gmu, "gmu2host_intr_info", GMU_GMU2HOST_INTR_INFO);
	get(&gmu, "ao_host_interrupt_status", GMU_AO_HOST_INTERRUPT_STATUS);
	get(&gmu, "hfi_ctrl_status", GMU_HFI_CTRL_STATUS);
	if (space_faulted(&gmu_space) || (result & mask) != started) {
		say("apollo gpu: %u: the GMU's firmware does not come up (core version %#x: waited for %#x in %#x)\n", step,
		    version, started, mask);
		gmu_post_mortem();
		return false;
	}
	say("apollo gpu: %u: THE GMU'S FIRMWARE RUNS\n", step);
	return true;
}

/* ---- step 16: the GMU's way to RPMh, and its message queues ------------------------------------------------ */

/*
 * The GMU raises and drops the 3D core's supply through command slots of its own (step 7) and a
 * sequencer in the always-on part of the SoC, "PDC", which on the a650 family the always-on processor
 * programs (a6xx_gmu_rpmh_init: pdc_in_aop). Left to us are the PDC's two enabling registers and the
 * GMU side's little program, after the firmware has started and before the message queues are.
 */
static struct block pdc = { "pdc", 0x0b290000, 0x10000 };       /* dt: qcom,gmu "kgsl_gmu_pdc_cfg" */
#define PDC_GPU_ENABLE          (4 * 0x1140)
#define PDC_GPU_SEQ_START_ADDR  (4 * 0x1148)
#define RSCC_PDC_SEQ_START_ADDR (4 * 0x008)
#define RSCC_PDC_MATCH_VALUE_LO (4 * 0x009)
#define RSCC_PDC_MATCH_VALUE_HI (4 * 0x00a)
#define RSCC_HIDDEN_TCS_CMD0_ADDR (4 * 0x00d)
#define RSCC_HIDDEN_TCS_CMD0_DATA (4 * 0x00e)
#define RSCC_OVERRIDE_START_ADDR (4 * 0x100)

static bool
step_gmu_queues(void)
{
	static const uint32_t program[5] = { 0xeaaae5a0, 0xe1a1ebab, 0xa2e0a581, 0xecac82e2, 0x0020edad };

	if (!cx_ready() || !gmu_runs) {
		return false;
	}
	announce("give the GMU's RPMh channel its sleep and wake program (rscc, 0x3de0000)");
	put(&rscc, "rsc_status0_drv0", RSCC_RSC_STATUS0_DRV0, 1u << 24);
	put(&rscc, "pdc_slave_id_drv0", RSCC_PDC_SLAVE_ID_DRV0, 1);
	put(&rscc, "hidden_tcs_cmd0_data", RSCC_HIDDEN_TCS_CMD0_DATA, 0);
	put(&rscc, "hidden_tcs_cmd0_addr", RSCC_HIDDEN_TCS_CMD0_ADDR, 0);
	put(&rscc, "hidden_tcs_cmd1_data", RSCC_HIDDEN_TCS_CMD0_DATA + 4 * 2, 0);
	put(&rscc, "hidden_tcs_cmd1_addr", RSCC_HIDDEN_TCS_CMD0_ADDR + 4 * 2, 0);
	put(&rscc, "hidden_tcs_cmd2_data", RSCC_HIDDEN_TCS_CMD0_DATA + 4 * 4, 0x80000000);    /* the a650's (Linux) */
	put(&rscc, "hidden_tcs_cmd2_addr", RSCC_HIDDEN_TCS_CMD0_ADDR + 4 * 4, 0);
	put(&rscc, "override_start_addr", RSCC_OVERRIDE_START_ADDR, 0);
	put(&rscc, "pdc_seq_start_addr", RSCC_PDC_SEQ_START_ADDR, 0x4520);
	put(&rscc, "pdc_match_value_lo", RSCC_PDC_MATCH_VALUE_LO, 0x4510);
	put(&rscc, "pdc_match_value_hi", RSCC_PDC_MATCH_VALUE_HI, 0x4514);
	for (unsigned i = 0; i < 5; i++) {
		put(&rscc, "seq_mem_drv0", RSCC_SEQ_MEM_0_DRV0 + 4 * i, program[i]);
	}
	get(&rscc, "seq_mem_0_drv0", RSCC_SEQ_MEM_0_DRV0);

	announce("enable the GPU's part of the always-on sequencer (pdc, 0xb294500 and 0xb294520: not touched before)");
	get(&pdc, "gpu_enable", PDC_GPU_ENABLE);
	get(&pdc, "gpu_seq_start_addr", PDC_GPU_SEQ_START_ADDR);
	put(&pdc, "gpu_seq_start_addr", PDC_GPU_SEQ_START_ADDR, 0);
	put(&pdc, "gpu_enable", PDC_GPU_ENABLE, 0x80000001);
	get(&pdc, "gpu_enable", PDC_GPU_ENABLE);

	announce("ask the GMU to start its message queues");
	put(&gmu, "hfi_ctrl_init", GMU_HFI_CTRL_INIT, 1);
	if (!wait_for(&gmu, GMU_HFI_CTRL_STATUS, 1, 1, 500)) {
		get(&gmu, "hfi_ctrl_status", GMU_HFI_CTRL_STATUS);
		get(&gmu, "cm3_fw_init_result", GMU_CM3_FW_INIT_RESULT);
		say("apollo gpu: %u: the GMU does not start its message queues\n", step);
		gmu_post_mortem();
		return false;
	}
	get(&gmu, "hfi_ctrl_status", GMU_HFI_CTRL_STATUS);
	get(&gmu, "hfi_version_info", GMU_HFI_VERSION_INFO);
	gmu_talks = true;
	say("apollo gpu: %u: THE GMU'S MESSAGE QUEUES ARE UP\n", step);
	return !space_faulted(&gmu_space);
}

/* ---- step 17: talk to the GMU; the 3D core powered ---------------------------------------------------------- */

/*
 * A message is words: the first says which (bits 7:0), how many words (15:8), a request or an answer
 * (19:16), and a number (31:20) that the answer repeats in its second word; the third word of an answer
 * is the GMU's verdict, 0 for done. Linux a6xx_hfi.c, a6xx_hfi.h.
 */
#define HFI_BW_TABLE            3
#define HFI_PERF_TABLE          4
#define HFI_START               10
#define HFI_CORE_FW_START       14
#define HFI_GX_BW_PERF_VOTE     30
#define HFI_PREPARE_SLUMBER     33
#define HFI_ERROR               100
#define HFI_ANSWER              1               /* in bits 19:16 */
#define HFI_NO_INDEX            0xff
#define GMU_HOST2GMU_INTR_SET   GMU_WORD(0x5194)
#define GMU_SPTPRAC_PWR_CLK_STATUS GMU_WORD(0x50d0)     /* bit 6: the 3D core has no power, bit 7: no clock */
#define GMU_CORE_REQUEST        (1u << 30)      /* of host2gmu_intr_set: OOB "GPU_SET", keep the 3D core up for us */
#define GMU_CORE_RELEASE        (1u << 31)      /* ... no more */
#define GMU_CORE_GRANTED        (1u << 31)      /* of gmu2host_intr_info */

struct hfi_perf_table {
	uint32_t header, gpu_levels, gmu_levels;
	struct {
		uint32_t vote, acd, khz;
	} gpu[16];
	struct {
		uint32_t vote, khz;
	} gmu[4];
};
struct hfi_bw_table {
	uint32_t header, levels, cnoc_commands, ddr_commands, cnoc_wait, ddr_wait;
	uint32_t cnoc_address[6], cnoc_data[2][6], ddr_address[8], ddr_data[16][8];
};

static uint32_t hfi_number;
static bool core_is_on;

/*
 * everything waiting in a queue of the GMU's; true if the answer to the message numbered `number` was among it.
 * How far the GMU has written is the GMU's word in shared memory: taken once, and only if it is a place in the
 * queue where a message can begin (messages are whole multiples of four words); and no more messages than fit.
 */
static bool
hfi_receive(unsigned which, uint32_t number, uint32_t *verdict)
{
	volatile struct hfi_queue *queue = &hfi->queue[which];
	volatile uint32_t *data = (volatile uint32_t *)((vm_offset_t)hfi + HFI_QUEUE_AT(which));
	uint32_t write = queue->write;
	bool answered = false;

	if (write >= HFI_QUEUE_WORDS || (write & 3)) {
		say("apollo gpu: %u: the GMU's queue %u says it is written up to %#x: not a place in it\n", step, which, write);
		return false;
	}
	for (uint32_t messages = 0; queue->read % HFI_QUEUE_WORDS != write && messages < HFI_QUEUE_WORDS / 4; messages++) {
		uint32_t message[8] = { 0 }, at = queue->read % HFI_QUEUE_WORDS, words = data[at] >> 8 & 0xff;

		for (uint32_t i = 0; i < words; i++) {
			if (i < 8) {
				message[i] = data[at];
			}
			at = (at + 1) % HFI_QUEUE_WORDS;
		}
		queue->read = (at + 3) / 4 * 4 % HFI_QUEUE_WORDS;
		if ((message[0] >> 16 & 0xf) == HFI_ANSWER && message[1] >> 20 == number) {
			*verdict = message[2];
			answered = true;
		} else {
			message[7] = 0;
			if ((message[0] & 0xff) == HFI_ERROR) {
				say("apollo gpu: %u: THE GMU REPORTS AN ERROR: %u, %u, \"%s\"\n", step, message[1] >> 16,
				    message[1] & 0xffff, (const char *)&message[2]);
			} else {
				say("apollo gpu: %u: the GMU says (queue %u, %u words): %#x %#x %#x %#x %#x %#x\n", step, which, words,
				    message[0], message[1], message[2], message[3], message[4], message[5]);
			}
		}
		if (words == 0) {
			break;
		}
	}
	return answered;
}

/* send a message whose first word is yet to be made and wait for the GMU's verdict on it */
static bool
hfi_send(const char *what, uint32_t id, uint32_t *message, uint32_t words)
{
	volatile struct hfi_queue *queue = &hfi->queue[0];
	volatile uint32_t *data = (volatile uint32_t *)((vm_offset_t)hfi + HFI_QUEUE_AT(0));
	uint32_t at = queue->write % HFI_QUEUE_WORDS, number = ++hfi_number & 0xfff, verdict = 0, waiting = 0;
	uint32_t room = (queue->read % HFI_QUEUE_WORDS + HFI_QUEUE_WORDS - at - 1) % HFI_QUEUE_WORDS;
	bool answered = false;

	if (room < (words + 3) / 4 * 4) {
		/* (the GMU has not taken what it was sent before: writing on would overwrite it) */
		say("apollo gpu: %u: no room in the GMU's queue for %s: %u words free of %u, read up to %u, written up to %u\n", step,
		    what, room, HFI_QUEUE_WORDS, queue->read, queue->write);
		return false;
	}
	message[0] = id | words << 8 | number << 20;
	if (!quiet) {
		say("apollo gpu: %u: about to send the GMU %s: message %u, %u words, number %u\n", step, what, id, words, number);
	}
	hold();
	for (uint32_t i = 0; i < (words + 3) / 4 * 4; i++) {
		data[at] = i < words ? message[i] : 0xfafafafa;
		at = (at + 1) % HFI_QUEUE_WORDS;
	}
	queue->write = at;
	__asm__ volatile ("dsb sy");
	wr(gmu.base + GMU_HOST2GMU_INTR_SET, 1);
	for (int i = 0; i < 10000 && !answered; i++) {
		sleep_us(100);
		waiting = rd(gmu.base + GMU_GMU2HOST_INTR_INFO);
		if (waiting & 1) {
			wr(gmu.base + GMU_GMU2HOST_INTR_CLR, 1);
			answered = hfi_receive(1, number, &verdict);
		}
		if (rd(gmu.base + GMU_CM3_FW_INIT_RESULT) & GMU_FAILED) {
			break;
		}
	}
	hfi_receive(2, ~0u, &waiting);
	if (!answered || verdict != 0) {
		say("apollo gpu: %u: the GMU %s (%#x) to %s; fw_init_result %#x, gmu2host_intr_info %#x\n", step,
		    answered ? "says no" : "does not answer", verdict, what, rd(gmu.base + GMU_CM3_FW_INIT_RESULT),
		    rd(gmu.base + GMU_GMU2HOST_INTR_INFO));
		gmu_post_mortem();
		return false;
	}
	if (!quiet) {
		say("apollo gpu: %u: the GMU accepts %s\n", step, what);
	}
	return true;
}

/*
 * A vote for a level of a supply: the level, its place in RPMh's list for the rail, and the place of
 * the level the memories' rail must have with it (a6xx_gmu.c: a6xx_gmu_rpmh_arc_votes_init).
 */
static bool
rail_vote(uint32_t level, const uint16_t *rail, unsigned rail_levels, const uint16_t *mx, unsigned mx_levels, uint32_t *vote)
{
	unsigned at, with = 0;

	for (at = 0; at < rail_levels && rail[at] < level; at++) {
	}
	if (at == rail_levels || mx_levels == 0) {
		return false;
	}
	while (with + 1 < mx_levels && mx[with] < rail[at]) {
		with++;
	}
	*vote = (uint32_t)rail[at] << 16 | with << 8 | at;
	return true;
}

/* dt: kgsl_3d0_reg_memory: the 3D core's registers; a6xx.xml counts in words */
static struct block gpu = { "gpu", 0x03d00000, 0x40000 };
#define GPU_WORD(n)             (4u * (n))
#define RBBM_INT_0_STATUS       GPU_WORD(0x0201)
#define RBBM_STATUS             GPU_WORD(0x0210)
#define RBBM_CLOCK_CNTL         GPU_WORD(0x00ae)
#define CP_RB_BASE              GPU_WORD(0x0800)

static uint32_t core_khz;

/*
 * Linux's order (a6xx_hfi_start, then a6xx_gmu_set_initial_freq, then hw_init's a6xx_gmu_set_oob): the
 * table of the 3D core's speeds with the supply level each needs -- this phone's (dt: qcom,gpu-pwrlevels
 * of speed bin 0, levels from gpu-opp-table_v2) -- and the GMU's own; the a650's table of bus votes
 * (a6xx_hfi.c a650_build_bw_table: one level, "off"); "start the core firmware", "start"; the speed
 * wanted; then the request to keep the 3D core powered, after which its registers can be read.
 */
static bool
step_core_power(void)
{
	static const struct {
		uint32_t khz, level;
	} speeds[] = { { 0, 0 }, { 305000, 64 }, { 400000, 128 }, { 441600, 144 }, { 490000, 192 }, { 525000, 224 },
		       { 587000, 256 } };
	static struct hfi_perf_table perf;
	static struct hfi_bw_table bw;
	uint32_t core_start[2] = { 0, 0 }, start[1] = { 0 }, vote[4] = { 0, 1, 2, HFI_NO_INDEX }, granted = 0;
	uint16_t gfx[16], cx[16], mx[16];
	unsigned gfx_levels, cx_levels, mx_levels;

	if (!cx_ready() || !gmu_talks) {
		return false;
	}
	if (!rpmh_resource("gfx.lvl", gfx, &gfx_levels) || !rpmh_resource("cx.lvl", cx, &cx_levels) ||
	    !rpmh_resource("mx.lvl", mx, &mx_levels)) {
		say("apollo gpu: %u: RPMh's list lacks one of the rails\n", step);
		return false;
	}
	perf.gpu_levels = sizeof(speeds) / sizeof(speeds[0]);
	for (unsigned i = 0; i < perf.gpu_levels; i++) {
		perf.gpu[i].khz = speeds[i].khz;
		perf.gpu[i].acd = 0xffffffff;           /* no adaptive clocking */
		if (!rail_vote(speeds[i].level, gfx, gfx_levels, mx, mx_levels, &perf.gpu[i].vote)) {
			say("apollo gpu: %u: RPMh has no level %u for the 3D core's rail\n", step, speeds[i].level);
			return false;
		}
	}
	perf.gmu_levels = 2;
	perf.gmu[1].khz = 200000;
	if (!rail_vote(0, cx, cx_levels, mx, mx_levels, &perf.gmu[0].vote) ||
	    !rail_vote(48, cx, cx_levels, mx, mx_levels, &perf.gmu[1].vote)) {  /* sm8250.dtsi: the GMU's 200 MHz at MIN_SVS */
		return false;
	}
	say("apollo gpu: %u: votes: 3D core %#x %#x %#x %#x %#x %#x %#x, GMU %#x %#x\n", step, perf.gpu[0].vote,
	    perf.gpu[1].vote, perf.gpu[2].vote, perf.gpu[3].vote, perf.gpu[4].vote, perf.gpu[5].vote, perf.gpu[6].vote,
	    perf.gmu[0].vote, perf.gmu[1].vote);

	bw.levels = 1;
	bw.ddr_commands = 3;
	bw.ddr_wait = 1;
	bw.ddr_address[0] = rpmh_resource("MC0", NULL, NULL);
	bw.ddr_address[1] = rpmh_resource("SH0", NULL, NULL);
	bw.ddr_address[2] = rpmh_resource("ACV", NULL, NULL);
	bw.ddr_data[0][0] = bw.ddr_data[0][1] = bw.ddr_data[0][2] = 0x40000000;
	bw.cnoc_commands = 1;
	bw.cnoc_wait = 1;
	bw.cnoc_address[0] = 0x500a4;           /* Linux's, fixed for the a650; cmd-db has no name for it here */
	bw.cnoc_data[0][0] = 0x40000000;
	bw.cnoc_data[1][0] = 0x60000001;
	say("apollo gpu: %u: RPMh's bus resources: MC0 %#x, SH0 %#x, ACV %#x (Linux has 0x50000, 0x50004, 0x5007c)\n", step,
	    bw.ddr_address[0], bw.ddr_address[1], bw.ddr_address[2]);
	if (bw.ddr_address[0] == 0 || bw.ddr_address[1] == 0 || bw.ddr_address[2] == 0) {
		return false;
	}

	if (speed_index >= 1 && speed_index < perf.gpu_levels) {
		vote[2] = speed_index;
	}
	core_khz = speeds[vote[2]].khz;
	say("apollo gpu: %u: the speed wanted: %u.%u MHz\n", step, core_khz / 1000, core_khz % 1000 / 100);
	announce("send the GMU its tables (speeds and supply levels, bus votes), \"start\", and the speed wanted; "
	    "from here on it casts votes in RPMh by itself");
	if (!hfi_send("the table of speeds", HFI_PERF_TABLE, &perf.header, sizeof(perf) / 4) ||
	    !hfi_send("the table of bus votes", HFI_BW_TABLE, &bw.header, sizeof(bw) / 4) ||
	    !hfi_send("\"start the core firmware\"", HFI_CORE_FW_START, core_start, 2) ||
	    !hfi_send("\"start\"", HFI_START, start, 1) ||
	    !hfi_send("the speed wanted", HFI_GX_BW_PERF_VOTE, vote, 4)) {
		return false;
	}

	announce("ask the GMU to power the 3D core and keep it powered");
	get(&gmu, "sptprac_pwr_clk_status", GMU_SPTPRAC_PWR_CLK_STATUS);
	get(&gpucc, "gx_gdscr", GPUCC_GX_GDSCR);
	put(&gmu, "host2gmu_intr_set", GMU_HOST2GMU_INTR_SET, GMU_CORE_REQUEST);
	for (int i = 0; i < 5000 && !(granted & GMU_CORE_GRANTED); i++) {
		sleep_us(100);
		granted = rd(gmu.base + GMU_GMU2HOST_INTR_INFO);
	}
	core_is_on = true;                      /* asked for, at least: to be given back */
	put(&gmu, "gmu2host_intr_clr", GMU_GMU2HOST_INTR_CLR, GMU_CORE_GRANTED);
	get(&gmu, "sptprac_pwr_clk_status", GMU_SPTPRAC_PWR_CLK_STATUS);
	get(&gpucc, "gx_gdscr", GPUCC_GX_GDSCR);
	get(&gmu, "rpmh_power_state", GMU_RPMH_POWER_STATE);
	if (!(granted & GMU_CORE_GRANTED) || (rd(gmu.base + GMU_SPTPRAC_PWR_CLK_STATUS) & 0xc0) != 0) {
		say("apollo gpu: %u: the GMU does not power the 3D core (gmu2host_intr_info %#x, fw_init_result %#x)\n", step,
		    granted, rd(gmu.base + GMU_CM3_FW_INIT_RESULT));
		gmu_post_mortem();
		return false;
	}
	say("apollo gpu: %u: measured: the 3D core's clock runs at %u Hz (%u MHz was asked for), the GMU's in it at %u Hz\n", step,
	    measure_quietly(GPUCC_DEBUG_GX_GFX3D), core_khz / 1000, measure_quietly(GPUCC_DEBUG_GX_GMU));

	announce("read registers of the 3D core itself (0x3d00000): the first time it has power");
	get(&gpu, "rbbm_status", RBBM_STATUS);
	get(&gpu, "rbbm_int_0_status", RBBM_INT_0_STATUS);
	get(&gpu, "rbbm_clock_cntl", RBBM_CLOCK_CNTL);
	get(&gpu, "cp_rb_base", CP_RB_BASE);
	say("apollo gpu: %u: THE 3D CORE HAS POWER AND ANSWERS\n", step);
	return !space_faulted(&gmu_space);
}

/* ---- step 18: the 3D core's memory and its microcode -------------------------------------------------------- */

/*
 * The 3D core takes its orders as words in a ring of memory, which a small processor inside it ("SQE",
 * with microcode of its own: a650_sqe.fw) reads up to where a register says we have written. Like the GMU
 * it reaches memory through the SMMU: stream 0 (dt: gfx3d_user, iommus <0 0x401>), to which this gives
 * context 0. The a650 has no second command processor (the 642L's "LPAC", hakios.adreno.c), so nothing
 * else of the core needs a stream. a650_sqe.fw is version 0x093: Linux wants 0x095 for "other security
 * fixes" (a6xx_ucode_check_version); the phone's own Android runs this one.
 */
#define CORE_STREAM             0
#define CORE_MATCH              0
#define PTE_UNPRIVILEGED        (1ull << 6)     /* the 3D core's accesses are; the GMU's are not */
#define PTE_READ_ONLY           (1ull << 7)
#define PTE_CORE                (PTE_GMU | PTE_UNPRIVILEGED)
/*
 * What is the kernel's alone in the 3D core's space -- the ring and the scratch page with the fences and the
 * times -- is mapped for privileged accesses only, so that a program's command buffers cannot read or write
 * it (Linux: MSM_BO_MAP_PRIV, IOMMU_PRIV, for the ring and its memptrs when the GPU has hw_apriv, which the
 * a650 has). That works because CP_APRIV_CNTL (step 20) makes the command processor's own fetches of the
 * ring and its writes from the ring privileged, as Linux's a6xx hw_init does. The microcode stays as Linux
 * maps it (adreno_fw_create_bo: read-only, not privileged): it is the public a650_sqe.fw. Boot-arg
 * gpu_protect=0: none of this (and no CP_PROTECT either, step 20) -- as before, for a phone where it does
 * not work; it is not yet tried on the hardware.
 */
static uint32_t gpu_protect = 1;
#define PTE_CORE_OWN            (gpu_protect ? PTE_GMU : PTE_CORE)
#define CORE_RING_ADDRESS       0x01000000u
#define CORE_RING_SIZE          0x8000u         /* Linux's; 8192 words */
#define CORE_SQE_ADDRESS        0x01100000u
#define CORE_SCRATCH_ADDRESS    0x01200000u
#define SMMU_SCTLR_SERVE_AFTER_FAULT (1u << 8)  /* Linux (qcom_adreno_smmu_write_sctlr): or the core hangs on a fault */

#define RBBM_GBIF_CLIENT_QOS_CNTL GPU_WORD(0x0011)
#define RBBM_GBIF_HALT          GPU_WORD(0x0016)
#define RBBM_GBIF_HALT_ACK      GPU_WORD(0x0017)
#define RBBM_INTERFACE_HANG_INT_CNTL GPU_WORD(0x001f)
#define RBBM_PERFCTR_CNTL       GPU_WORD(0x0500)
#define RBBM_PERFCTR_GPU_BUSY_MASKED GPU_WORD(0x050b)
#define CP_RB_CNTL              GPU_WORD(0x0802)
#define CP_RB_RPTR              GPU_WORD(0x0806)
#define CP_RB_WPTR              GPU_WORD(0x0807)
#define CP_SQE_CNTL             GPU_WORD(0x0808)
#define CP_HW_FAULT             GPU_WORD(0x0821)
#define CP_INTERRUPT_STATUS     GPU_WORD(0x0823)
#define CP_PROTECT_STATUS       GPU_WORD(0x0824)
#define CP_SQE_INSTR_BASE       GPU_WORD(0x0830)
#define CP_APRIV_CNTL           GPU_WORD(0x0844)
#define CP_PROTECT_CNTL         GPU_WORD(0x084f)
#define CP_PROTECT(n)           GPU_WORD(0x0850 + (n))
#define CP_PROTECTS             48              /* the a650's count_max (DECLARE_ADRENO_PROTECT(a650_protect, 48)) */
#define CP_PROTECT_CNTL_ON      (1u << 0 | 1u << 1 | 1u << 3)   /* access protection, fault on violation, last span to the end */
#define PROTECT_RDONLY(reg, len) (((len) & 0x3fffu) << 18 | ((reg) & 0x3ffffu))
#define PROTECT_NORDWR(reg, len) (1u << 31 | PROTECT_RDONLY(reg, len))
#define CP_ROQ_THRESHOLDS_1     GPU_WORD(0x08c1)
#define CP_ROQ_THRESHOLDS_2     GPU_WORD(0x08c2)
#define CP_MEM_POOL_SIZE        GPU_WORD(0x08c3)
#define CP_AHB_CNTL             GPU_WORD(0x098d)
#define UCHE_WRITE_RANGE_MAX    GPU_WORD(0x0e05)
#define UCHE_WRITE_THRU_BASE    GPU_WORD(0x0e07)
#define UCHE_TRAP_BASE          GPU_WORD(0x0e09)
#define UCHE_CACHE_WAYS         GPU_WORD(0x0e17)
#define UCHE_FILTER_CNTL        GPU_WORD(0x0e18)
#define UCHE_CLIENT_PF          GPU_WORD(0x0e19)
#define PC_DBG_ECO_CNTL         GPU_WORD(0x9e00)
#define RBBM_SECVID_TSB_TRUSTED_BASE GPU_WORD(0xf800)
#define RBBM_SECVID_TSB_TRUSTED_SIZE GPU_WORD(0xf802)
#define RBBM_SECVID_TSB_CNTL    GPU_WORD(0xf803)
#define CP_RB_CNTL_VALUE        (12u | 2u << 8 | 1u << 27)      /* 2^12 pairs of words; no read position copied to memory */

/* commands (Mesa's adreno_pm4.xml); a packet is a header word and so many more */
#define CP_MEM_WRITE            0x3d
#define CP_ME_INIT              0x48
#define CP_SET_SECURE_MODE      0x66

static struct gpu_space core_space = {
	.name = "the 3D core", .context = 0, .mask = 0x401, .control = SMMU_SCTLR_SERVE_AFTER_FAULT,
};
static struct gpu_block core_ring, core_sqe, core_scratch;
static bool core_space_is_on, core_runs, zap_is_loaded;
static bool core_stuck;                 /* the 3D core has not finished what it was given, and is not waited for */
#define RING_WORDS              (CORE_RING_SIZE / 4)
static uint32_t ring_at, ring_sent;

static uint32_t
odd_parity(uint32_t v)
{
	return 0x9669 >> (0xf & (v ^ v >> 4 ^ v >> 8 ^ v >> 12 ^ v >> 16 ^ v >> 20 ^ v >> 24 ^ v >> 28)) & 1;
}

static void
ring(uint32_t word)
{
	((uint32_t *)core_ring.memory)[ring_at] = word;
	ring_at = (ring_at + 1) % RING_WORDS;
}

/* the header of a command with so many words after it */
static void
ring_command(uint32_t command, uint32_t words)
{
	ring(7u << 28 | words | odd_parity(words) << 15 | command << 16 | odd_parity(command) << 23);
}

static void
ring_send(void)
{
	if (ring_at >= ring_sent) {
		to_memory(core_ring.memory + 4 * ring_sent, 4 * (ring_at - ring_sent));
	} else {
		to_memory(core_ring.memory + 4 * ring_sent, 4 * (RING_WORDS - ring_sent));
		to_memory(core_ring.memory, 4 * ring_at);
	}
	ring_sent = ring_at;
	wr(gpu.base + CP_RB_WPTR, ring_at);
}

/* let the 3D core have what is in the ring, and wait until it has taken all of it and rests */
static bool
ring_run(const char *what)
{
	uint32_t taken = ~0u, status = ~0u;

	say("apollo gpu: %u: about to have the 3D core %s: the ring up to word %u\n", step, what, ring_at);
	hold();
	ring_send();
	for (int i = 0; i < 20000 && (taken != ring_at || (status & ~1u)); i++) {
		sleep_us(100);
		taken = rd(gpu.base + CP_RB_RPTR);
		status = rd(gpu.base + RBBM_STATUS);
	}
	if (taken == ring_at && !(status & ~1u) && rd(gpu.base + CP_HW_FAULT) == 0 && !space_faulted(&core_space)) {
		say("apollo gpu: %u: the 3D core has done it (ring read up to %u, status %#x)\n", step, taken, status);
		return true;
	}
	say("apollo gpu: %u: the 3D core does not get through: ring read up to %u of %u, status %#x, interrupts %#x, "
	    "command processor: fault %#x, interrupts %#x, protection %#x\n", step, taken, ring_at, status,
	    rd(gpu.base + RBBM_INT_0_STATUS), rd(gpu.base + CP_HW_FAULT), rd(gpu.base + CP_INTERRUPT_STATUS),
	    rd(gpu.base + CP_PROTECT_STATUS));
	space_faulted(&core_space);
	return false;
}

static void
put64(struct block *b, const char *name, uint32_t offset, uint64_t value)
{
	put(b, name, offset, (uint32_t)value);
	put(b, name, offset + 4, (uint32_t)(value >> 32));
}

static bool
step_core_space(void)
{
	const uint8_t *microcode;
	uint64_t size = 0;
	uint32_t version;

	if (!cx_ready() || !core_is_on) {
		return false;
	}
	announce("make the 3D core's memory: its ring, its microcode (fw/a650_sqe.fw), a page to try writing to");
	microcode = image_file("a650_sqe.fw", &size);
	if (microcode == NULL || size < 8 || size > 0x40000 ||
	    !block_new(&core_space, &core_ring, CORE_RING_ADDRESS, CORE_RING_SIZE, PTE_CORE_OWN) ||
	    !block_new(&core_space, &core_sqe, CORE_SQE_ADDRESS, (uint32_t)size - 4, PTE_CORE | PTE_READ_ONLY) ||
	    !block_new(&core_space, &core_scratch, CORE_SCRATCH_ADDRESS, 4096, PTE_CORE_OWN)) {
		return false;
	}
	memcpy((void *)core_sqe.memory, microcode + 4, size - 4);       /* the file's first word is not microcode */
	to_memory(core_sqe.memory, core_sqe.size);
	memcpy(&version, microcode + 4, sizeof(version));
	say("apollo gpu: %u: the microcode's version: %#x (Linux wants at least 0x095)\n", step, version & 0xfff);

	announce("give the SMMU's context 0 these tables and send the 3D core's requests (stream 0) through it");
	core_space_is_on = true;
	if (!space_on(&core_space, CORE_MATCH, CORE_STREAM)) {
		/* the 642L's SMMU takes no 0x401 (hakios.adreno.c: it reads back 0): then as the GMU's, 0x400 */
		say("apollo gpu: %u: trying the stream with the mask 0x400 instead\n", step);
		core_space.mask = 0x400;
		if (!space_on(&core_space, CORE_MATCH, CORE_STREAM)) {
			return false;
		}
	}
	get(&gpu, "rbbm_status", RBBM_STATUS);
	return !space_faulted(&core_space);
}

/* ---- step 19: the zap shader, loaded -------------------------------------------------------------------- */

/*
 * The rest of what Linux does with the zap shader (mdt_loader.c: qcom_mdt_pas_init, qcom_mdt_load;
 * adreno_gpu.c: zap_shader_load_mdt): after the header (step 12), TrustZone is told where the image will
 * be -- dt: reserved-memory pil_gpu_region@8681a000, 0x2000 (Linux's sm8250.dtsi gpu_mem) -- the image's
 * one loadable segment (a650_zap.b02, 0x68c bytes at 0x5000 of the image, at the start of that memory:
 * the span of its segments rounded to 4K) is put there, and TrustZone is asked to authenticate it and
 * run it. Our kernels come and go without the phone restarting: a kernel gives the zap shader back when
 * it gives the GPU back (gmu_stop()), and asks for that first here too, for one that did not end in order.
 */
#define ZAP_MEMORY              0x8681a000ull
#define ZAP_MEMORY_SIZE         0x2000u
#define SCM_PAS_MEM_SETUP       SCM_CALL(2, 2)
#define SCM_PAS_SHUTDOWN        SCM_CALL(2, 6)
#define SCM_THREE_VALUES        3

static bool
zap_unload(void)
{
	uint64_t answer = ~0ull;

	return scm("to give up the zap shader", SCM_64 | SCM_SIP | SCM_PAS_SHUTDOWN, SCM_ONE_VALUE, PAS_GPU, 0, 0, &answer) == 0 &&
	       answer == 0;
}

static bool
step_zap_load(void)
{
	const uint8_t *segment;
	volatile uint8_t *memory;
	uint64_t size = 0, header = 0, answer = ~0ull;

	if (!cx_ready() || !core_is_on) {
		return false;
	}
	announce("ask TrustZone to give up a zap shader that a kernel before this one may have left");
	say("apollo gpu: %u: %s\n", step, zap_unload() ? "TrustZone has given up what it had (it says so with none, too)" :
	    "TRUSTZONE DOES NOT GIVE IT UP: the next answers tell whether it keeps one");
	segment = image_file("a650_zap.b02", &size);
	if (segment == NULL || size == 0 || size > 0x1000) {
		return false;
	}
	announce("hand the zap shader's header to TrustZone, tell it where the image goes, put the image there, and ask "
	    "TrustZone to authenticate and run it");
	if (!zap_header(&header)) {
		return false;
	}
	if (scm("to take the zap shader's signature", SCM_64 | SCM_SIP | SCM_PAS_INIT_IMAGE, SCM_VALUE_AND_BUFFER, PAS_GPU,
	    header, 0, &answer) != 0 || answer != 0) {
		say("apollo gpu: %u: TrustZone does not take the signature (%#llx): taken to mean that it keeps a zap shader "
		    "from before, which is used as it is\n", step, answer);
		zap_is_loaded = true;
		return true;
	}
	if (scm("where the image will be", SCM_64 | SCM_SIP | SCM_PAS_MEM_SETUP, SCM_THREE_VALUES, PAS_GPU, ZAP_MEMORY, 0x1000,
	    &answer) != 0 || answer != 0) {
		say("apollo gpu: %u: TrustZone does not take the place (%#llx): taken to mean that it keeps a zap shader from "
		    "before, which is used as it is\n", step, answer);
		zap_is_loaded = true;
		return true;
	}
	/* (the memory is looked at only now: TrustZone has just said that it is not its own) */
	memory = (volatile uint8_t *)ml_io_map_wcomb(ZAP_MEMORY, ZAP_MEMORY_SIZE);
	say("apollo gpu: %u: the reserved memory begins %02x %02x %02x %02x, the zap shader %02x %02x %02x %02x\n", step,
	    memory[0], memory[1], memory[2], memory[3], segment[0], segment[1], segment[2], segment[3]);
	say("apollo gpu: %u: about to write %llu bytes at %#llx\n", step, size, ZAP_MEMORY);
	hold();
	for (uint64_t i = 0; i < 0x1000; i++) {
		memory[i] = i < size ? segment[i] : 0;
	}
	__asm__ volatile ("dsb sy");
	if (scm("to authenticate the zap shader and run it", SCM_64 | SCM_SIP | SCM_PAS_AUTH_AND_RESET, SCM_ONE_VALUE, PAS_GPU, 0,
	    0, &answer) != 0 || answer != 0) {
		say("apollo gpu: %u: TrustZone does not authenticate the zap shader (%#llx)\n", step, answer);
		return false;
	}
	say("apollo gpu: %u: TRUSTZONE HAS AUTHENTICATED AND STARTED THE ZAP SHADER\n", step);
	zap_is_loaded = true;
	return true;
}

/* ---- step 20: the 3D core's registers, its command processor started, its first command ------------------- */

/*
 * Linux's a6xx_gpu.c hw_init, the a650's branches, less what only matters once it draws (clock gating,
 * counters' selection, interrupts): the values are checked against it one by one. The protection of
 * registers against user programs' commands (a6xx_set_cp_protect) is in, unless gpu_protect=0: programs'
 * command buffers reach the core, and without it they may write the registers that say where the ring,
 * the microcode and the SMMU's tables are. UBWC (a6xx_set_ubwc_config with SM8250's ubwc_4_0_hbb16:
 * UBWC 4.0, highest bank bit 16): the same values Mesa will be told.
 */
static bool
step_core_start(void)
{
	static const uint32_t address_modes[] = { 0x0842, 0x0c01, 0x8601, 0x8e05, 0x9e01, 0xbe05, 0xa601, 0x9601, 0x0e00, 0xae01,
		                                  0xb601, 0xf810 };

	if (!cx_ready() || !core_is_on || !core_space_is_on) {
		return false;
	}
	announce("write the 3D core's registers as Linux does for the a650 before it starts the command processor");
	put(&gbif, "halt", GBIF_HALT, 0);
	put(&gpu, "rbbm_gbif_halt", RBBM_GBIF_HALT, 0);
	put(&gpu, "rbbm_secvid_tsb_cntl", RBBM_SECVID_TSB_CNTL, 0);
	put64(&gpu, "rbbm_secvid_tsb_trusted_base", RBBM_SECVID_TSB_TRUSTED_BASE, 0);
	put(&gpu, "rbbm_secvid_tsb_trusted_size", RBBM_SECVID_TSB_TRUSTED_SIZE, 0);
	for (unsigned i = 0; i < sizeof(address_modes) / sizeof(address_modes[0]); i++) {
		put(&gpu, "address mode: 64 bits", GPU_WORD(address_modes[i]), 1);
	}
	put(&gpu, "rbbm_gbif_client_qos_cntl", RBBM_GBIF_CLIENT_QOS_CNTL, 3);
	put(&gpu, "rbbm_perfctr_gpu_busy_masked", RBBM_PERFCTR_GPU_BUSY_MASKED, 0xffffffff);
	put64(&gpu, "uche_write_range_max", UCHE_WRITE_RANGE_MAX, 0x0001fffffffff000ull + 0xfc0);
	put64(&gpu, "uche_trap_base", UCHE_TRAP_BASE, 0x0001fffffffff000ull);
	put64(&gpu, "uche_write_thru_base", UCHE_WRITE_THRU_BASE, 0x0001fffffffff000ull);
	put(&gpu, "uche_filter_cntl", UCHE_FILTER_CNTL, 0x804);
	put(&gpu, "uche_cache_ways", UCHE_CACHE_WAYS, 0x4);
	put(&gpu, "cp_roq_thresholds_2", CP_ROQ_THRESHOLDS_2, 0x02000140);
	put(&gpu, "cp_roq_thresholds_1", CP_ROQ_THRESHOLDS_1, 0x8040362c);
	put(&gpu, "cp_mem_pool_size", CP_MEM_POOL_SIZE, 128);
	put(&gpu, "pc_dbg_eco_cntl", PC_DBG_ECO_CNTL, 0x00300200);     /* a6xx_catalog.c: the a650's prim_fifo_threshold */
	put(&gpu, "cp_ahb_cntl", CP_AHB_CNTL, 1);
	put(&gpu, "rbbm_perfctr_cntl", RBBM_PERFCTR_CNTL, 1);
	put(&gpu, "rb_nc_mode_cntl", GPU_WORD(0x8e08), 1u << 11 | 1u << 4 | 3u << 1);     /* rgb565 predicator, amsbc, hbb 3 */
	put(&gpu, "tpl1_nc_mode_cntl", GPU_WORD(0xb604), 3u << 1);
	put(&gpu, "sp_nc_mode_cntl", GPU_WORD(0xae02), 2u << 4 | 3u << 1);              /* uavflagprd_inv 2: the a650 family */
	put(&gpu, "uche_mode_cntl", GPU_WORD(0x0e01), 3u << 21);
	put(&gpu, "rbbm_nc_mode_cntl", GPU_WORD(0x0534), 1);                             /* macrotile mode */
	put(&gpu, "rbbm_interface_hang_int_cntl", RBBM_INTERFACE_HANG_INT_CNTL, 1u << 30 | 0x1fffff);
	put(&gpu, "uche_client_pf", UCHE_CLIENT_PF, 1u << 7 | 1);
	put(&gpu, "cp_apriv_cntl", CP_APRIV_CNTL, 1u << 6 | 1u << 5 | 1u << 3 | 1u << 2 | 1u << 1);
	if (gpu_protect) {
		/*
		 * The ranges of registers a command buffer that is not the ring's may not write (and some not
		 * read): Linux's a650_protect_regs (drivers/gpu/drm/msm/adreno/a6xx_catalog.c, before that
		 * a6xx_gpu.c; "these are for a620 and a650"), written as a6xx_set_cp_protect writes them: the
		 * last entry into the last register, its span to the end of the registers.
		 */
		static const uint32_t a650_protect[] = {
			PROTECT_RDONLY(0x00000, 0x04ff), PROTECT_RDONLY(0x00501, 0x0005), PROTECT_RDONLY(0x0050b, 0x02f4),
			PROTECT_NORDWR(0x0050e, 0x0000), PROTECT_NORDWR(0x00510, 0x0000), PROTECT_NORDWR(0x00534, 0x0000),
			PROTECT_NORDWR(0x00800, 0x0082), PROTECT_NORDWR(0x008a0, 0x0008), PROTECT_NORDWR(0x008ab, 0x0024),
			PROTECT_RDONLY(0x008de, 0x00ae), PROTECT_NORDWR(0x00900, 0x004d), PROTECT_NORDWR(0x0098d, 0x0272),
			PROTECT_NORDWR(0x00e00, 0x0001), PROTECT_NORDWR(0x00e03, 0x000c), PROTECT_NORDWR(0x03c00, 0x00c3),
			PROTECT_RDONLY(0x03cc4, 0x1fff), PROTECT_NORDWR(0x08630, 0x01cf), PROTECT_NORDWR(0x08e00, 0x0000),
			PROTECT_NORDWR(0x08e08, 0x0000), PROTECT_NORDWR(0x08e50, 0x001f), PROTECT_NORDWR(0x08e80, 0x027f),
			PROTECT_NORDWR(0x09624, 0x01db), PROTECT_NORDWR(0x09e60, 0x0011), PROTECT_NORDWR(0x09e78, 0x0187),
			PROTECT_NORDWR(0x0a630, 0x01cf), PROTECT_NORDWR(0x0ae02, 0x0000), PROTECT_NORDWR(0x0ae50, 0x032f),
			PROTECT_NORDWR(0x0b604, 0x0000), PROTECT_NORDWR(0x0b608, 0x0007), PROTECT_NORDWR(0x0be02, 0x0001),
			PROTECT_NORDWR(0x0be20, 0x17df), PROTECT_NORDWR(0x0f000, 0x0bff), PROTECT_RDONLY(0x0fc00, 0x1fff),
			PROTECT_NORDWR(0x18400, 0x1fff), PROTECT_NORDWR(0x1a800, 0x1fff), PROTECT_NORDWR(0x1f400, 0x0443),
			PROTECT_RDONLY(0x1f844, 0x007b), PROTECT_NORDWR(0x1f887, 0x001b),
			PROTECT_NORDWR(0x1f8c0, 0x0000),        /* the last: to the end of the registers */
		};
		const unsigned last = sizeof(a650_protect) / sizeof(a650_protect[0]) - 1;

		put(&gpu, "cp_protect_cntl", CP_PROTECT_CNTL, CP_PROTECT_CNTL_ON);
		for (unsigned i = 0; i < last; i++) {
			wr(gpu.base + CP_PROTECT(i), a650_protect[i]);
		}
		wr(gpu.base + CP_PROTECT(CP_PROTECTS - 1), a650_protect[last]);
		if (!quiet) {
			say("apollo gpu: %u: %u ranges of the 3D core's registers protected from command buffers (cp_protect)\n", step,
			    last + 1);
		}
	}
	put64(&gpu, "cp_sqe_instr_base", CP_SQE_INSTR_BASE, CORE_SQE_ADDRESS);
	put64(&gpu, "cp_rb_base", CP_RB_BASE, CORE_RING_ADDRESS);
	put(&gpu, "cp_rb_cntl", CP_RB_CNTL, CP_RB_CNTL_VALUE);
	get(&gpu, "cp_sqe_instr_base", CP_SQE_INSTR_BASE);
	get(&gpu, "cp_rb_base", CP_RB_BASE);
	get(&gpu, "cp_rb_cntl", CP_RB_CNTL);
	get(&gpu, "rbbm_status", RBBM_STATUS);

	announce("start the 3D core's command processor");
	core_runs = true;
	put(&gpu, "cp_sqe_cntl", CP_SQE_CNTL, 1);
	sleep_ms(20);
	say("apollo gpu: %u: alive after it; status %#x, interrupts %#x, command processor: fault %#x, interrupts %#x, "
	    "protection %#x, ring read up to %u\n", step, rd(gpu.base + RBBM_STATUS), rd(gpu.base + RBBM_INT_0_STATUS),
	    rd(gpu.base + CP_HW_FAULT), rd(gpu.base + CP_INTERRUPT_STATUS), rd(gpu.base + CP_PROTECT_STATUS),
	    rd(gpu.base + CP_RB_RPTR));
	get(&smmu, "sgfsr", SMMU_SGFSR);
	if (space_faulted(&core_space)) {
		return false;
	}

	announce("give the 3D core its first command, \"initialize\"");
	ring_command(CP_ME_INIT, 8);            /* a6xx_cp_init */
	ring(0x0000002f);
	ring(0x00000003);                       /* several hardware contexts */
	ring(0x20000000);                       /* error detection */
	ring(0);
	ring(0);
	ring(0);
	ring(0);
	ring(0);
	if (!ring_run("initialize its command processor")) {
		return false;
	}
	say("apollo gpu: %u: THE 3D CORE TAKES COMMANDS\n", step);
	return true;
}

/* ---- step 21: out of secure mode; a word written on command ------------------------------------------------ */

static bool
step_core_commands(void)
{
	volatile uint32_t *scratch = (volatile uint32_t *)core_scratch.memory;

	if (!cx_ready() || !core_runs) {
		return false;
	}
	announce("command the 3D core out of secure mode (it asks the zap shader), then to write a word into a page of ours");
	ring_command(CP_SET_SECURE_MODE, 1);
	ring(0);
	if (!ring_run("leave secure mode")) {
		return false;
	}
	ring_command(CP_MEM_WRITE, 3);
	ring(CORE_SCRATCH_ADDRESS);
	ring(0);
	ring(0x600dc0de);
	if (!ring_run("write a word to memory")) {
		return false;
	}
	to_memory(core_scratch.memory, 64);
	say("apollo gpu: %u: the page reads %#x: %s\n", step, scratch[0], scratch[0] == 0x600dc0de ?
	    "THE 3D CORE WRITES OUR MEMORY ON COMMAND" : "not what it was told to write");
	return scratch[0] == 0x600dc0de;
}

/* ---- step 22: a frame filled by the GPU, on the panel ------------------------------------------------------ */

/*
 * The simplest thing the 3D core draws: its 2D engine filling a rectangle with one colour (Mesa's
 * fd6_blitter.cc: fd6_clear_surface; registers as Linux's a6xx.xml names them), three times for three
 * bands. The picture is 32-bit pixels in rows of a multiple of 64 bytes, in memory of ours; the
 * framebuffer driver copies it to the panel (MiniIOMobileFramebufferShow, shared). RB_CCU_CNTL: where the
 * colour cache sits when drawing straight into memory, Mesa's ccu_offset_bypass for the a650 -- three
 * CCUs of 64K (freedreno_devices.py num_ccu 3) -- and no concurrent resolve (that is the a660 family's).
 */
#define CP_NOP                  0x10
#define CP_WAIT_FOR_IDLE        0x26
#define CP_BLIT                 0x2c
#define CP_EVENT_WRITE          0x46
#define CP_SET_MARKER           0x65
#define RB_CCU_CNTL             0x8e07          /* register numbers: these are written by command */
#define RB_CCU_CNTL_BYPASS      ((3u * 0x10000 >> 12) << 23)
#define GRAS_2D_BLIT_CNTL       0x8400
#define GRAS_2D_DST_TL          0x8405
#define GRAS_2D_DST_BR          0x8406
#define RB_2D_BLIT_CNTL         0x8c00
#define RB_2D_PIXEL_CNTL        0x8c01
#define RB_2D_DST_INFO          0x8c17
#define RB_2D_DST               0x8c18
#define RB_2D_DST_PITCH         0x8c1a
#define RB_2D_SRC_SOLID_C0      0x8c2c
#define SP_2D_DST_FORMAT        0xacc0
#define FORMAT_8888             0x30
#define BLIT_SOLID_8888         (1u << 7 | FORMAT_8888 << 8 | 0xfu << 20 | 0x10u << 24) /* one colour; all channels; 8-bit */
#define DST_BGRA_LINEAR         (FORMAT_8888 | 1u << 10)        /* rows of pixels, blue first in memory */
#define EVENT_FLUSH_COLOR       0x1d
#define EVENT_CACHE_FLUSH       0x04
#define CORE_TARGET_ADDRESS     0x10000000u
#define TARGET_WIDTH            1080            /* the Mi 10T's panel */
#define TARGET_HEIGHT           2400
#define TARGET_ROW              4352            /* 1088 pixels: the next multiple of 64 bytes */

extern void MiniIOMobileFramebufferShow(const uint8_t *pixels, uint32_t width, uint32_t height, uint32_t row_bytes);

static struct gpu_block core_target;

/* a register of the 3D core (by its word number) written by command, in the order of the other commands */
static void
ring_register(uint32_t number, uint32_t value)
{
	ring(4u << 28 | 1 | odd_parity(1) << 7 | number << 8 | odd_parity(number) << 27);
	ring(value);
}

static void
ring_fill(uint32_t top, uint32_t bottom, uint32_t red, uint32_t green, uint32_t blue)
{
	ring_register(GRAS_2D_DST_TL, top << 16);
	ring_register(GRAS_2D_DST_BR, (TARGET_WIDTH - 1) | (bottom - 1) << 16);
	ring_register(RB_2D_SRC_SOLID_C0, red);         /* a register for each part of the colour, 0 to 255 */
	ring_register(RB_2D_SRC_SOLID_C0 + 1, green);
	ring_register(RB_2D_SRC_SOLID_C0 + 2, blue);
	ring_register(RB_2D_SRC_SOLID_C0 + 3, 255);
	ring_register(RB_2D_BLIT_CNTL, BLIT_SOLID_8888);
	ring_register(GRAS_2D_BLIT_CNTL, BLIT_SOLID_8888);
	ring_register(SP_2D_DST_FORMAT, 1 | FORMAT_8888 << 3 | 0xfu << 12);
	ring_register(RB_2D_PIXEL_CNTL, 0);
	ring_register(RB_2D_DST_INFO, DST_BGRA_LINEAR);
	ring_register(RB_2D_DST, CORE_TARGET_ADDRESS);
	ring_register(RB_2D_DST + 1, 0);
	ring_register(RB_2D_DST_PITCH, TARGET_ROW >> 6);
	ring_command(CP_WAIT_FOR_IDLE, 0);
	ring_command(CP_BLIT, 1);
	ring(3);                                /* "scale": the 2D engine's one operation */
}

static bool
step_frame(void)
{
	const volatile uint8_t *pixel;

	if (!cx_ready() || !core_runs) {
		return false;
	}
	announce("make a picture's worth of memory for the 3D core (10 MB) and have it fill three bands of colour");
	/* (PTE_CORE, not PTE_CORE_OWN: it is the 2D engine that writes it, and its writes are not privileged) */
	if (!block_new(&core_space, &core_target, CORE_TARGET_ADDRESS, TARGET_ROW * TARGET_HEIGHT, PTE_CORE)) {
		return false;
	}
	/* tables that the SMMU has in use have grown: it has not seen these addresses, so nothing stale to forget */
	for (uint32_t i = 0; i < core_space.table_count; i++) {
		to_memory((vm_offset_t)core_space.tables[i].table, 4096);
	}
	ring_command(CP_SET_MARKER, 1);
	ring(12);                               /* "2D blits follow" */
	ring_register(RB_CCU_CNTL, RB_CCU_CNTL_BYPASS);
	ring_fill(0, 800, 255, 128, 0);         /* orange */
	ring_fill(800, 1600, 255, 255, 255);
	ring_fill(1600, TARGET_HEIGHT, 0, 128, 255);
	/* what it has drawn, out of its caches into memory; each event writes a word when done (Linux's a6xx_submit) */
	ring_command(CP_EVENT_WRITE, 4);
	ring(EVENT_FLUSH_COLOR);
	ring(CORE_SCRATCH_ADDRESS + 32);
	ring(0);
	ring(0x0c0c0c0c);
	ring_command(CP_EVENT_WRITE, 4);
	ring(EVENT_CACHE_FLUSH);
	ring(CORE_SCRATCH_ADDRESS + 16);
	ring(0);
	ring(0x0d0e0d0e);
	ring_command(CP_WAIT_FOR_IDLE, 0);
	if (!ring_run("fill a picture")) {
		return false;
	}
	to_memory(core_scratch.memory, 64);
	say("apollo gpu: %u: the words it writes when its caches are in memory: %#x, %#x (told: 0xc0c0c0c, 0xd0e0d0e)\n", step,
	    ((volatile uint32_t *)core_scratch.memory)[8], ((volatile uint32_t *)core_scratch.memory)[4]);
	to_memory(core_target.memory, core_target.size);
	pixel = (const volatile uint8_t *)core_target.memory;
	say("apollo gpu: %u: the picture's first pixel is %02x %02x %02x %02x (blue green red alpha; told: 00 80 ff ff), one in "
	    "the middle %02x %02x %02x %02x, one in the last row %02x %02x %02x %02x\n", step, pixel[0], pixel[1], pixel[2],
	    pixel[3], pixel[1200 * TARGET_ROW + 2160], pixel[1200 * TARGET_ROW + 2161], pixel[1200 * TARGET_ROW + 2162],
	    pixel[1200 * TARGET_ROW + 2163], pixel[2399 * TARGET_ROW + 4316], pixel[2399 * TARGET_ROW + 4317],
	    pixel[2399 * TARGET_ROW + 4318], pixel[2399 * TARGET_ROW + 4319]);
	if ((pixel[0] | pixel[1] | pixel[2]) == 0 || pixel[1200 * TARGET_ROW + 2160] == 0 || (pixel[2399 * TARGET_ROW + 4316] |
	    pixel[2399 * TARGET_ROW + 4317] | pixel[2399 * TARGET_ROW + 4318]) == 0) {
		say("apollo gpu: %u: the picture is not what the 3D core was told to draw\n", step);
		return false;
	}
	MiniIOMobileFramebufferShow((const uint8_t *)core_target.memory, TARGET_WIDTH, TARGET_HEIGHT, TARGET_ROW);
	say("apollo gpu: %u: A PICTURE DRAWN BY THE GPU IS ON THE PANEL: orange, white, blue from the top\n", step);
	sleep_ms(3000);                         /* to be seen before the system draws over it */
	return true;
}

/*
 * The GPU block's bus interface told to let what is under way finish and start nothing more (Linux's
 * a6xx_bus_clear_pending_transactions), before the core's context is taken away: `for_good` halts the
 * core's own side too and leaves it so (a panic, a core that does not rest).
 */
static void
core_bus_halt(bool for_good)
{
	if (for_good) {
		wr(gpu.base + RBBM_GBIF_HALT, 1);
		for (int i = 0; i < 100000 && !(rd(gpu.base + RBBM_GBIF_HALT_ACK) & 1); i++) {
		}
	}
	wr(gbif.base + GBIF_HALT, 1);
	for (int i = 0; i < 100000 && !(rd(gbif.base + GBIF_HALT_ACK) & 1); i++) {
	}
	wr(gbif.base + GBIF_HALT, 2);
	for (int i = 0; i < 100000 && !(rd(gbif.base + GBIF_HALT_ACK) & 2); i++) {
	}
	if (!for_good) {
		wr(gbif.base + GBIF_HALT, 0);
	}
}

/*
 * The GMU stopped again, before the registers the steps changed are put back (the clocks, CX): in the
 * order of Linux's a6xx_gmu_shutdown, as far as these steps go -- the processor into reset, the 3D
 * core's switch off should it have come on, the GMU's RPMh channel put to sleep (its votes dropped),
 * its stream sent nowhere.
 */
static void
gmu_stop(void)
{
	uint32_t slumber[3] = { 0, 0, 0 };

	if (core_runs && core_is_on) {
		say("apollo gpu: the 3D core before it is stopped: status %#x, ring read up to %u of %u\n", rd(gpu.base + RBBM_STATUS),
		    rd(gpu.base + CP_RB_RPTR), ring_at);
		core_bus_halt(core_stuck);
	}
	if (core_is_on) {
		put(&gmu, "host2gmu_intr_set", GMU_HOST2GMU_INTR_SET, GMU_CORE_RELEASE);
		core_is_on = false;
	}
	if (gmu_talks) {
		/* a6xx_gmu_notify_slumber: the GMU drops its votes and the 3D core's power itself */
		hfi_send("\"slumber\"", HFI_PREPARE_SLUMBER, slumber, 3);
		if (!wait_for(&gmu, GMU_AO_GPU_CX_BUSY_STATUS, 1u << 23, 0, 100)) {
			get(&gmu, "ao_gpu_cx_busy_status", GMU_AO_GPU_CX_BUSY_STATUS);
		}
		get(&gpucc, "gx_gdscr", GPUCC_GX_GDSCR);
		gmu_talks = false;
	}
	if (gmu_runs) {
		put(&gmu, "cm3_sysreset", GMU_CM3_SYSRESET, 1);
		gmu_runs = gmu_talks = false;
		if (rd(gpucc.base + GPUCC_GX_GDSCR) & GDSC_POWER_ON) {
			put(&gpucc, "gx_gdscr", GPUCC_GX_GDSCR, rd(gpucc.base + GPUCC_GX_GDSCR) | GDSC_SW_COLLAPSE);
			wait_for(&gpucc, GPUCC_GX_GDSCR, GDSC_POWER_ON, 0, 100);
		}
	}
	if (zap_is_loaded) {
		say("apollo gpu: the zap shader: %s\n", zap_unload() ? "TrustZone has given it up" :
		    "TRUSTZONE DOES NOT GIVE IT UP; the next kernel will find it there");
		zap_is_loaded = false;
	}
	if (rsc_awake) {
		put(&gmu, "rscc_control_req", GMU_RSCC_CONTROL_REQ, 1);
		if (!wait_for(&rscc, RSCC_RSC_STATUS0_DRV0, RSCC_ASLEEP, RSCC_ASLEEP, 100)) {
			say("apollo gpu: the GMU's RPMh channel does not go to sleep\n");
		}
		get(&rscc, "rsc_status0_drv0", RSCC_RSC_STATUS0_DRV0);
		put(&gmu, "rscc_control_req", GMU_RSCC_CONTROL_REQ, 0);
		rsc_awake = false;
	}
	if (gmu_space_is_on) {
		space_faulted(&gmu_space);
		space_off(&gmu_space, GMU_MATCH);
		gmu_space_is_on = false;
	}
	if (core_space_is_on) {
		space_faulted(&core_space);
		space_off(&core_space, CORE_MATCH);
		core_space_is_on = core_runs = false;
	}
}

static void put_back(void);
bool apollo_gpu_done(uint32_t fence);

/* ---- the GPU kept on, for the system to draw with --------------------------------------------------- */

/*
 * The service ApolloGPU (iokit/Kernel/hakios.apollo_gpu.cpp, a copy of hakios.gpu.cpp) keeps the buffers and the
 * clients; what is the GPU's is here, as in hakios.adreno.c's section of the same name (which has the
 * long explanations): addresses in the 3D core's SMMU context, the ring, and the word by which the core
 * says how far it has got (a "fence": a submission's number, written by the core when it is through with
 * it). The caller holds one lock around all of these.
 */
#define GPU_CHIP_ID             0x06050002ull   /* dt: qcom,chipid */
#define GPU_GMEM_SIZE           0x120000u       /* a6xx_catalog.c: SZ_1M + SZ_128K */
#define GPU_HIGHEST_BANK_BIT    16              /* ubwc_4_0_hbb16 */
#define GPU_UBWC_SWIZZLE        6               /* UBWC_SWIZZLE_ENABLE_LVL2 | LVL3 */
#define GPU_ADDRESSES_START     0x20000000ull   /* above everything the steps use; below 4 GB, as Linux keeps them */
#define GPU_ADDRESSES_SIZE      0xe0000000ull
#define GPU_ADDRESS_GRANULE     0x10000ull      /* what one allocation is rounded up to */
/* every address handed out can be mapped: tables for all of them (the last level's 2 MB each, the next's 1 GB, the root) */
_Static_assert(SPACE_TABLES >= ((GPU_ADDRESSES_START + GPU_ADDRESSES_SIZE) >> 21) + ((GPU_ADDRESSES_START + GPU_ADDRESSES_SIZE) >> 30) + 1,
    "SPACE_TABLES does not cover GPU_ADDRESSES_START + GPU_ADDRESSES_SIZE");
#define GPU_FENCE_AT            0x40            /* in the scratch page */
#define GPU_TIMES_AT            0x400           /* ... and when the core began and ended each of its last submissions */
#define GPU_TIMES               64
#define CP_ALWAYS_ON_COUNTER    0x0980          /* ticks of the 19.2 MHz crystal, 64 bits: for the core to read itself */
#define CP_REG_TO_MEM           0x3e
#define CP_SCRATCH_REG2         0x0885
#define CP_IB1_BASE             GPU_WORD(0x0928)
#define CP_IB1_REM_SIZE         GPU_WORD(0x092a)
#define CP_IB2_BASE             GPU_WORD(0x092b)
#define CP_IB2_REM_SIZE         GPU_WORD(0x092d)
#define CP_INDIRECT_BUFFER      0x3f
#define EVENT_INVALIDATE_DEPTH  0x18
#define EVENT_INVALIDATE_COLOR  0x19
/* for a core that has stopped answering (gpu_recover=1: apollo_gpu_recover() below) */
#define RBBM_SW_RESET_CMD       GPU_WORD(0x0043)        /* a6xx.xml; 1 holds the 3D core's own half (GX) in reset */
#define RBBM_INT_CLEAR_CMD      GPU_WORD(0x0037)        /* the bits of rbbm_int_0_status written here are forgotten */
#define GMU_PWR_COL_KEEPALIVE   GMU_WORD(0x50c3)        /* a6xx_gmu.xml; 1: the GMU keeps the core powered */
#define GMU_AHB_FENCE_STATUS_CLR GMU_WORD(0x9314)
#define GMU_AO_AHB_FENCE_CTRL   GMU_WORD(0x9310)
#define GPU_RECOVERIES          64

static bool gpu_kept;
static lck_mtx_t *gpu_times_lock;       /* gpu_times(): the submitting thread, the governor's and apollo_gpu_worked() all add up */
static lck_mtx_t *gpu_hfi_lock;         /* the GMU's message queue and the governor's vote: apart from the clients' lock, which a vote must not hold up */
static uint32_t gpu_fence;              /* the number of the last submission */
static uint32_t gpu_timed;              /* ... of the last whose times have been added up */
static uint64_t gpu_ticks;              /* how long the core has worked on the submissions added up */
static uint64_t gov_ticks;              /* ... and in the speed governor's window */
static uint32_t gpu_timed_count;
static uint32_t gpu_timed_by[7];        /* ... how many of them under 0.1, 0.25, 0.5, 1, 2, 4 ms, and longer */
static uint32_t gpu_exact = 1;          /* boot-arg gpu_exact=0: the core does not wait to be idle before it notes the time */
static uint32_t gpu_recover;            /* boot-arg gpu_recover=1: a core that has stopped answering is reset and given work again */
static uint32_t gpu_recoveries;
uint32_t apollo_gpu_log;                /* boot-arg gpu_log=1: a line for every submission and the time the core spent on it */
uint32_t apollo_gpu_va;                 /* boot-arg gpu_va: 0 the default, 1 describe the addresses, 2 also a granule's guard after each buffer */
extern void hakios_tl(const char *format, ...) __printflike(1, 2);
static struct {
	uint64_t start, size;
} gpu_free[1024];                       /* the addresses not in use, in order */
static unsigned gpu_free_count;
static uint64_t gpu_va_used;
static uint32_t gpu_va_live;
static unsigned gpu_va_holes;
static uint64_t gpu_va_lost;
static uint32_t gpu_va_refused;

static uint64_t
gpu_address_span(uint64_t size)
{
	uint64_t span = (size + (GPU_ADDRESS_GRANULE - 1)) & ~(GPU_ADDRESS_GRANULE - 1);

	return apollo_gpu_va >= 2 ? span + GPU_ADDRESS_GRANULE : span;
}

extern void ApolloGPUStart(void);
extern void ApolloGPUStop(void);        /* takes the clients' lock, then calls apollo_gpu_stop() */

bool
apollo_gpu_ready(void)
{
	return gpu_kept;
}

/* a run that ends in order gives the GPU back (loader_return.c calls this first, from a thread that may wait) */
void
apollo_adreno_stop(void)
{
	if (gpu_kept) {
		ApolloGPUStop();
	}
}

void
apollo_gpu_stop(void)
{
	uint32_t waited = 0;

	if (!gpu_kept) {
		return;
	}
	gpu_kept = false;                       /* (nothing more is taken) */
	lck_mtx_lock(gpu_hfi_lock);             /* (a vote of the governor's that is on its way is let finish) */
	lck_mtx_unlock(gpu_hfi_lock);
	quiet = false;
	/* what the core has been given, it finishes first: half a second at the most; then its bus is halted for good */
	while (!apollo_gpu_done(gpu_fence) && waited < 5000) {
		sleep_us(100);
		waited++;
	}
	core_stuck = !apollo_gpu_done(gpu_fence);
	say("apollo gpu: the run is about to end: the GPU is given back (%s after %u us)\n",
	    core_stuck ? "STILL AT WORK" : "it had finished what it was given", 100 * waited);
	put_back();
}

/* the answers to msm's GET_PARAM (Linux's adreno_get_param for this GPU; the numbers are msm_drm.h's) */
bool
apollo_gpu_parameter(uint32_t which, uint64_t *value)
{
	switch (which) {
	case 0x01:                              /* GPU_ID: none, the chip ID says it */
	case 0x06:                              /* GMEM_BASE */
	case 0x08:                              /* PP_PGTABLE */
	case 0x09:                              /* FAULTS */
	case 0x0a:                              /* SUSPENDS */
	case 0x11:                              /* RAYTRACING */
		*value = 0;
		return true;
	case 0x02:                              /* GMEM_SIZE */
		*value = GPU_GMEM_SIZE;
		return true;
	case 0x03:                              /* CHIP_ID */
		*value = GPU_CHIP_ID;
		return true;
	case 0x04:                              /* MAX_FREQ */
		*value = core_khz * 1000ull;
		return true;
	case 0x05:                              /* TIMESTAMP */
		*value = mach_absolute_time();
		return true;
	case 0x07:                              /* PRIORITIES */
		*value = 1;
		return true;
	case 0x10:                              /* HIGHEST_BANK_BIT */
		*value = GPU_HIGHEST_BANK_BIT;
		return true;
	case 0x12:                              /* UBWC_SWIZZLE */
		*value = GPU_UBWC_SWIZZLE;
		return true;
	case 0x13:                              /* MACROTILE_MODE */
		*value = 1;
		return true;
	case 0x0e:                              /* VA_START */
	case 0x0f:                              /* VA_SIZE */
		if (apollo_gpu_va == 0) {
			return false;                   /* msm refuses these to a client that shares the GPU's own page tables */
		}
		*value = which == 0x0e ? GPU_ADDRESSES_START : GPU_ADDRESSES_SIZE;
		return true;
	default:
		return false;
	}
}

/* addresses for a buffer of so many bytes; 0: none left. They are the buffer's until apollo_gpu_address_free(). */
uint64_t
apollo_gpu_address(uint64_t size)
{
	size = gpu_address_span(size);
	for (unsigned i = 0; i < gpu_free_count; i++) {
		if (gpu_free[i].size >= size) {
			uint64_t address = gpu_free[i].start;

			gpu_free[i].start += size;
			gpu_free[i].size -= size;
			if (gpu_free[i].size == 0) {
				memmove(&gpu_free[i], &gpu_free[i + 1], (--gpu_free_count - i) * sizeof(gpu_free[0]));
			}
			gpu_va_used += size;
			gpu_va_live++;
			return address;
		}
	}
	gpu_va_refused++;
	return 0;
}

void
apollo_gpu_address_free(uint64_t address, uint64_t size)
{
	unsigned at;

	size = gpu_address_span(size);
	gpu_va_used -= size;
	gpu_va_live--;
	for (at = 0; at < gpu_free_count && gpu_free[at].start < address; at++) {
	}
	if (at > 0 && gpu_free[at - 1].start + gpu_free[at - 1].size == address) {
		gpu_free[at - 1].size += size;
		if (at < gpu_free_count && address + size == gpu_free[at].start) {
			gpu_free[at - 1].size += gpu_free[at].size;
			memmove(&gpu_free[at], &gpu_free[at + 1], (--gpu_free_count - at) * sizeof(gpu_free[0]));
		}
	} else if (at < gpu_free_count && address + size == gpu_free[at].start) {
		gpu_free[at].start = address;
		gpu_free[at].size += size;
	} else if (gpu_free_count < sizeof(gpu_free) / sizeof(gpu_free[0])) {
		memmove(&gpu_free[at + 1], &gpu_free[at], (gpu_free_count++ - at) * sizeof(gpu_free[0]));
		gpu_free[at].start = address;
		gpu_free[at].size = size;
	} else {
		gpu_va_lost += size;            /* no room to remember the hole: counted, not handed out again */
	}
	if (gpu_free_count > gpu_va_holes) {
		gpu_va_holes = gpu_free_count;
	}
}

/* what the 3D core's address space holds, for whoever prints the GPU's numbers (gpu_va=1) */
void
apollo_gpu_space(uint64_t *used, uint32_t *live, uint64_t *largest, unsigned *holes, unsigned *most, uint64_t *lost,
    uint32_t *refused)
{
	*largest = 0;
	for (unsigned i = 0; i < gpu_free_count; i++) {
		if (gpu_free[i].size > *largest) {
			*largest = gpu_free[i].size;
		}
	}
	*used = gpu_va_used;
	*live = gpu_va_live;
	*holes = gpu_free_count;
	*most = gpu_va_holes;
	*lost = gpu_va_lost;
	*refused = gpu_va_refused;
}

/* one 4K page of ours at an address of the 3D core's; after the last of a buffer's, apollo_gpu_mapped() */
bool
apollo_gpu_map(uint64_t address, uint64_t phys, bool read_only)
{
	return space_map(&core_space, address, phys, PTE_CORE | (read_only ? PTE_READ_ONLY : 0));
}

void
apollo_gpu_mapped(void)
{
	space_commit(&core_space);
}

/*
 * The pages taken out of the 3D core's space, and the SMMU made to forget what it had of them. false: it has
 * not said so within 10 ms -- it may still translate the addresses to the old pages, so the caller keeps the
 * memory the GPU's for good (as it keeps a buffer the core is not done with) and the addresses unused.
 */
bool
apollo_gpu_unmap(uint64_t address, uint64_t size)
{
	uint32_t context = smmu_context(&core_space);
	uint64_t began, patience;

	for (uint64_t offset = 0; offset < size; offset += 4096) {
		space_map(&core_space, address + offset, 0, 0);
	}
	space_commit(&core_space);
	wr(smmu.base + context + SMMU_CB_TLBIASID, core_space.context);
	wr(smmu.base + context + SMMU_CB_TLBSYNC, 1);
	nanoseconds_to_absolutetime(10 * NSEC_PER_MSEC, &patience);
	began = mach_absolute_time();
	while (rd(smmu.base + context + SMMU_CB_TLBSTATUS) & 1) {
		if (mach_absolute_time() - began >= patience) {
			say("apollo gpu: THE SMMU DOES NOT FINISH FORGETTING %#llx..%#llx (tlbstatus %#x): the memory stays the GPU's\n",
			    address, address + size - 1, rd(smmu.base + context + SMMU_CB_TLBSTATUS));
			return false;
		}
		sleep_us(1);
	}
	return true;
}

/* memory that the 3D core and a program see without this processor's cache in between: out of that cache */
void
apollo_gpu_to_memory(void *start, uint64_t size)
{
	to_memory((vm_offset_t)start, size);
}

bool
apollo_gpu_done(uint32_t fence)
{
	volatile uint32_t *word = (volatile uint32_t *)(core_scratch.memory + GPU_FENCE_AT);

	to_memory((vm_offset_t)word, 4);
	return (int32_t)(*word - fence) >= 0;
}

uint32_t
apollo_gpu_last(void)
{
	return gpu_fence;
}

/*
 * The submissions given to the core and not done yet: the last one's number less the last done (the fence word the core
 * writes). No lock, so it may be one off; for the hitch tracer (hakios.apollo_hitch.c), which calls it from a timer interrupt.
 */
uint32_t
apollo_gpu_in_flight(void)
{
	volatile uint32_t *word;
	int32_t ahead;

	if (!gpu_kept || core_scratch.memory == 0) {
		return 0;
	}
	word = (volatile uint32_t *)(core_scratch.memory + GPU_FENCE_AT);
	to_memory((vm_offset_t)word, 4);
	ahead = (int32_t)(gpu_fence - *word);
	return ahead > 0 ? (uint32_t)ahead : 0;
}

/* the core is to write down what time it is: at one end of submission `fence` */
static void
ring_time(uint32_t fence, uint32_t end)
{
	ring_command(CP_REG_TO_MEM, 3);
	ring(CP_ALWAYS_ON_COUNTER | 2u << 18 | 1u << 30);       /* two words, a 64-bit register */
	ring(CORE_SCRATCH_ADDRESS + GPU_TIMES_AT + 16 * (fence % GPU_TIMES) + 8 * end);
	ring(0);
}


/* add up the times of the submissions the core has finished since this was last done (gpu_times_lock held) */
static void
gpu_times_locked(void)
{
	volatile uint64_t *times = (volatile uint64_t *)(core_scratch.memory + GPU_TIMES_AT);
	uint32_t done;

	to_memory(core_scratch.memory, 4096);
	done = *(volatile uint32_t *)(core_scratch.memory + GPU_FENCE_AT);
	if (done - gpu_timed > GPU_TIMES) {
		gpu_timed = done - GPU_TIMES;           /* (the older ones have been written over) */
	}
	while (gpu_timed != done) {
		uint32_t slot = ++gpu_timed % GPU_TIMES;

		if (gpu_timed != 0 && times[2 * slot + 1] > times[2 * slot]) {
			static const uint32_t under[6] = { 1920, 4800, 9600, 19200, 38400, 76800 };     /* (19.2 ticks a microsecond) */
			uint64_t ticks = times[2 * slot + 1] - times[2 * slot];
			unsigned by = 0;

			if (apollo_gpu_log) {
				hakios_tl("tl: q %u took %llu\n", gpu_timed, ticks * 10 / 192);
			}
			while (by < 6 && ticks >= under[by]) {
				by++;
			}
			gpu_timed_by[by]++;
			gpu_ticks += ticks;
			__atomic_fetch_add(&gov_ticks, ticks, __ATOMIC_RELAXED);
			gpu_timed_count++;
		}
	}
}

static void
gpu_times(void)
{
	lck_mtx_lock(gpu_times_lock);
	gpu_times_locked();
	lck_mtx_unlock(gpu_times_lock);
}

/* how long the 3D core has worked since this was last asked, on how many submissions, and how many took under ... */
void
apollo_gpu_worked(uint64_t *microseconds, uint32_t *submissions, uint32_t by[7])
{
	lck_mtx_lock(gpu_times_lock);
	gpu_times_locked();
	*microseconds = gpu_ticks * 10 / 192;
	*submissions = gpu_timed_count;
	memcpy(by, gpu_timed_by, sizeof(gpu_timed_by));
	bzero(gpu_timed_by, sizeof(gpu_timed_by));
	gpu_ticks = 0;
	gpu_timed_count = 0;
	lck_mtx_unlock(gpu_times_lock);
}

/*
 * The speed governor (apollo_gpu_dcvs, on by default): the 3D core works on the window server's frames in bursts,
 * and how much of each tenth of a second it spent working is known from the times it writes down itself (ring_time()).
 * Over 65% of a window busy: straight to the step that would make it ~60% busy (the next frame must not wait); under
 * 30% for three windows running: one step down, but not below apollo_gpu_floor (default 3) while 30+ submissions a
 * second come (the floor goes 300 ms after the last one); GPU sensor 70 C or more: top step 4. A step is the GMU's perf table (step_core_power(): 305, 400, 441.6, 490, 525, 587 MHz),
 * asked for as Linux does (HFI GX_BW_PERF_VOTE), from the submitting thread, with the caller's lock held.
 */
#define GOV_WINDOW_US           100000
#define GOV_UP_PERCENT          65
#define GOV_DOWN_PERCENT        30
#define GOV_DOWN_WINDOWS        3
#define GOV_TARGET_PERCENT      60              /* a jump up aims at this much busy at the new speed */
#define GOV_ACTIVE_SUBMITS      30              /* submissions a second that count as frames being drawn */
#define GOV_IDLE_US             300000          /* this long without a submission drops the floor */
#define GOV_HOT                 700             /* tenths of a degree: from here the top step is 4 */
#define GOV_HOT_LEVEL           4
#define GOV_TSENS0_PHYS         0x0c263000UL    /* as hakios.apollo_cpufreq.c: tsens@c263000, sensor 15 the GPU */
#define GOV_TSENS_STATUS(n)     (0xa0 + 4 * (n))
#define GOV_TSENS_VALID         (1u << 21)

static uint32_t gpu_dcvs = 1, gpu_max_level = 6, gpu_floor = 3, gov_level, gov_low, gov_changes, gov_submits;
static uint64_t gov_since, gov_last_submit, gov_temp_at;
static bool gov_floor_on;
static int32_t gov_temp = -2730;
static vm_offset_t gov_tm0;
static const uint32_t gov_khz[7] = { 0, 305000, 400000, 441600, 490000, 525000, 587000 };

/* the GPU sensor in tenths of a degree, read at most once a second (-2730: none) */
static int32_t
gov_gpu_temp(uint64_t now)
{
	uint64_t ns;

	if (gov_tm0 == 0) {
		return -2730;
	}
	absolutetime_to_nanoseconds(now - gov_temp_at, &ns);
	if (gov_temp_at != 0 && ns < 1000000000ull) {
		return gov_temp;
	}
	gov_temp_at = now;
	gov_temp = -2730;
	for (unsigned again = 0; again < 16; again++) {
		uint32_t status = *(volatile uint32_t *)(gov_tm0 + GOV_TSENS_STATUS(15));

		if (status & GOV_TSENS_VALID) {
			int32_t tenths = (int32_t)(status << 20) >> 20;

			gov_temp = tenths > 1500 || tenths < -400 ? -2730 : tenths;
			break;
		}
	}
	return gov_temp;
}

/*
 * One evaluation of the governor, from its own thread every GOV_WINDOW_US (gpu_governor_thread): the submit path only
 * notes that a submission came (gov_last_submit, gov_submits) and the core's busy ticks are added to gov_ticks by
 * gpu_times(); the decision and the HFI vote, which waits for the GMU's answer, are made here. gpu_hfi_lock held.
 */
static void
gpu_governor(void)
{
	uint64_t now = mach_absolute_time(), nanos, gap, last = __atomic_load_n(&gov_last_submit, __ATOMIC_RELAXED);
	uint32_t busy, level, top, floor_level, submits, vote[4] = { 0, 1, 0, HFI_NO_INDEX };
	uint64_t ticks;

	gpu_times();            /* (the busy ticks of what the core has finished) */
	/* frames are being drawn while submissions keep coming; 300 ms without one and the floor is gone */
	if (last) {
		absolutetime_to_nanoseconds(now - last, &gap);
		if (gap > GOV_IDLE_US * 1000ull) {
			gov_floor_on = false;
		}
	}
	submits = __atomic_exchange_n(&gov_submits, 0, __ATOMIC_RELAXED);
	ticks = __atomic_exchange_n(&gov_ticks, 0, __ATOMIC_RELAXED);
	if (gov_since == 0) {
		gov_since = now;
		return;
	}
	absolutetime_to_nanoseconds(now - gov_since, &nanos);
	if (nanos == 0) {
		return;
	}
	busy = (uint32_t)(ticks * 10 / 192 * 100 / (nanos / 1000 + 1));        /* ticks of 19.2 MHz: microseconds */
	if (gpu_floor && (uint64_t)submits * 1000000000ull >= (uint64_t)GOV_ACTIVE_SUBMITS * nanos) {
		gov_floor_on = true;
	}
	gov_since = now;

	top = gpu_max_level;
	if (gov_gpu_temp(now) >= GOV_HOT && top > GOV_HOT_LEVEL) {
		top = GOV_HOT_LEVEL;
	}
	floor_level = gov_floor_on ? (gpu_floor < top ? gpu_floor : top) : 1;
	level = gov_level;
	if (busy > GOV_UP_PERCENT && level < top) {
		/* the speed at which this much work would take about 60% of a window, the next step up from it */
		uint64_t want = (uint64_t)gov_khz[level] * busy / GOV_TARGET_PERCENT;
		uint32_t n = level + 1;

		while (n < top && gov_khz[n] < want) {
			n++;
		}
		level = n;
		gov_low = 0;
	} else if (busy < GOV_DOWN_PERCENT && level > floor_level && ++gov_low >= GOV_DOWN_WINDOWS) {
		level--;
		gov_low = 0;
	} else if (busy >= GOV_DOWN_PERCENT) {
		gov_low = 0;
	}
	if (level > top) {
		level = top;
	}
	if (level < floor_level) {
		level = floor_level;
	}
	if (level == gov_level) {
		return;
	}
	vote[2] = level;
	if (hfi_send("the speed wanted", HFI_GX_BW_PERF_VOTE, vote, 4)) {
		if (gov_changes++ < 20 || gov_changes % 200 == 0) {
			say("apollo gpu: the 3D core %s to %u MHz (%u%% busy%s; change %u)\n", level > gov_level ? "up" : "down",
			    gov_khz[level] / 1000, busy, gov_floor_on ? ", floor on" : "", gov_changes);
		}
		gov_level = level;
		core_khz = gov_khz[level];
	}
}

static void
gpu_governor_thread(__unused void *parameter, __unused wait_result_t result)
{
	for (;;) {
		delay_for_interval(GOV_WINDOW_US, NSEC_PER_USEC);
		if (!gpu_kept || !gpu_dcvs) {
			continue;               /* (a thread of the kernel does not end: it waits for the next run) */
		}
		lck_mtx_lock(gpu_hfi_lock);
		if (gpu_kept && !core_stuck) {          /* (apollo_gpu_stop() and the recovery take this lock too) */
			gpu_governor();
		}
		lck_mtx_unlock(gpu_hfi_lock);
	}
}

uint64_t apollo_gpu_phase[4];           /* the last submission's ticks: waiting for ring room, adding up times, writing the ring, the doorbell */

/*
 * Command buffers for the 3D core to run, in order: each an address and a count of words. As Linux's
 * a6xx_submit puts them in the ring. Returns the submission's number; 0: the ring is full and the core is
 * not reading it.
 *
 * The caller holds the clients' lock (ApolloGPU's gGPULock) all the while, so a full ring holds up every
 * other client, and the display's driver's waits, until there is room: that wait is a second at the most
 * (it was two), the same second after which a wait for a submission takes the core to have stopped
 * answering (gpuWait()); a ring that is still full after it is taken the same way. Letting go of the lock
 * while waiting would mean checking the submission's buffers again afterwards (they may have been given
 * back meanwhile): not done.
 */
uint32_t
apollo_gpu_submit(const uint64_t *addresses, const uint32_t *words, uint32_t count)
{
	uint32_t wanted = 26 + 4 * count, room = 0;
	uint64_t t0 = mach_absolute_time(), t1, t2, t3;

	for (int i = 0; i < 10000; i++) {
		room = (rd(gpu.base + CP_RB_RPTR) + RING_WORDS - ring_at - 1) % RING_WORDS;
		if (room >= wanted) {
			break;
		}
		sleep_us(100);
	}
	t1 = mach_absolute_time();
	if (room < wanted) {
		apollo_gpu_phase[0] = t1 - t0;
		return 0;
	}
	gpu_times();
	t2 = mach_absolute_time();
	/* the governor is not run from here (it waits for the GMU): the thread notes what it needs */
	__atomic_store_n(&gov_last_submit, t2, __ATOMIC_RELAXED);
	__atomic_fetch_add(&gov_submits, 1, __ATOMIC_RELAXED);
	if (++gpu_fence == 0) {
		gpu_fence = 1;
	}
	ring_time(gpu_fence, 0);
	ring_command(CP_EVENT_WRITE, 1);
	ring(EVENT_INVALIDATE_DEPTH);
	ring_command(CP_EVENT_WRITE, 1);
	ring(EVENT_INVALIDATE_COLOR);
	for (uint32_t i = 0; i < count; i++) {
		ring_command(CP_INDIRECT_BUFFER, 3);
		ring((uint32_t)addresses[i]);
		ring((uint32_t)(addresses[i] >> 32));
		ring(words[i]);
	}
	if (gpu_exact) {
		ring_command(CP_WAIT_FOR_IDLE, 0);      /* the time is the pipeline's, not the command processor's, which is ahead of it */
	}
	ring_time(gpu_fence, 1);
	ring_register(CP_SCRATCH_REG2, gpu_fence);
	ring_command(CP_EVENT_WRITE, 4);
	ring(EVENT_CACHE_FLUSH);
	ring(CORE_SCRATCH_ADDRESS + GPU_FENCE_AT);
	ring(0);
	ring(gpu_fence);
	t3 = mach_absolute_time();
	ring_send();
	apollo_gpu_phase[0] = t1 - t0;
	apollo_gpu_phase[1] = t2 - t1;
	apollo_gpu_phase[2] = t3 - t2;
	apollo_gpu_phase[3] = mach_absolute_time() - t3;
	return gpu_fence;
}

/* a submission does not come back: what the 3D core and its SMMU context say */
void
apollo_gpu_report(const char *why)
{
	uint32_t context = smmu_context(&core_space);

	say("apollo gpu: %s: submission %u is the last, the core has done %u; ring: read up to %u, written up to %u; status "
	    "%#x, interrupts %#x; command processor: fault %#x, interrupts %#x, protection %#x\n", why, gpu_fence,
	    *(volatile uint32_t *)(core_scratch.memory + GPU_FENCE_AT), rd(gpu.base + CP_RB_RPTR), ring_at,
	    rd(gpu.base + RBBM_STATUS), rd(gpu.base + RBBM_INT_0_STATUS), rd(gpu.base + CP_HW_FAULT),
	    rd(gpu.base + CP_INTERRUPT_STATUS), rd(gpu.base + CP_PROTECT_STATUS));
	say("apollo gpu:   in the first command buffer at %#llx, %u words to go; in the second at %#llx, %u to go\n",
	    (uint64_t)rd(gpu.base + CP_IB1_BASE + 4) << 32 | rd(gpu.base + CP_IB1_BASE), rd(gpu.base + CP_IB1_REM_SIZE),
	    (uint64_t)rd(gpu.base + CP_IB2_BASE + 4) << 32 | rd(gpu.base + CP_IB2_BASE), rd(gpu.base + CP_IB2_REM_SIZE));
	step = 0;
	if (space_faulted(&core_space)) {
		wr(smmu.base + context + SMMU_CB_FSR, SMMU_FSR_ALL);    /* so that the next one shows */
	}
}

/*
 * boot-arg gpu_recover=1: the 3D core is put back where steps 20 and 21 left it, and everything it had been
 * given is given up, so that the next submission is its own again (hakios.adreno.c's hakios_gpu_recover()
 * has the sources and the reasons; the a650's GMU is the same kind of GMU, and the sequence is the same):
 * the command processor halted, the GMU told to keep the core powered, the bus halted, the reset line of the
 * core's own half (GX) pulled and let go, the ring emptied, and steps 20 and 21 again. Not Linux's, which
 * takes the whole CX half down for a part with a real GMU; so it is behind a boot-arg.
 */
static bool apollo_gpu_recover_locked(void);

bool
apollo_gpu_recover(void)
{
	bool ok;

	if (!gpu_kept) {
		return false;
	}
	lck_mtx_lock(gpu_hfi_lock);             /* (no vote of the governor's while the GMU is being started again) */
	lck_mtx_lock(gpu_times_lock);
	ok = apollo_gpu_recover_locked();
	lck_mtx_unlock(gpu_times_lock);
	lck_mtx_unlock(gpu_hfi_lock);
	return ok;
}

static bool
apollo_gpu_recover_locked(void)
{
	uint32_t fence_status;

	if (!gpu_kept || !gpu_recover) {
		return false;
	}
	if (gpu_recoveries >= GPU_RECOVERIES) {
		say("apollo gpu: the 3D core has been recovered %u times in this run already: no more\n", gpu_recoveries);
		return false;
	}
	gpu_recoveries++;
	step = 0;
	last_step = 0;                          /* (no line of the steps' waits for the loader any more) */

	wr(gpu.base + CP_SQE_CNTL, 3);
	wr(gmu.base + GMU_PWR_COL_KEEPALIVE, 1);
	core_bus_halt(true);
	wr(gpu.base + RBBM_SW_RESET_CMD, 1);
	rd(gpu.base + RBBM_SW_RESET_CMD);
	sleep_us(101);
	wr(gpu.base + RBBM_SW_RESET_CMD, 0);
	rd(gpu.base + RBBM_SW_RESET_CMD);
	sleep_us(1);
	fence_status = rd(gmu.base + GMU_AHB_FENCE_STATUS);
	wr(gmu.base + GMU_AHB_FENCE_STATUS_CLR, 7);
	wr(gmu.base + GMU_AO_AHB_FENCE_CTRL, 0);
	ring_at = ring_sent = rd(gpu.base + CP_RB_RPTR) % RING_WORDS;
	wr(gpu.base + CP_RB_WPTR, ring_at);
	wr(gpu.base + RBBM_INT_CLEAR_CMD, rd(gpu.base + RBBM_INT_0_STATUS));
	say("apollo gpu: the 3D core's half has been reset: status %#x, interrupts %#x, ring at %u; the GMU sees it busy: "
	    "%#x, and powered: %#x (bits 6 and 7 clear); it had dropped writes of ours: %#x\n", rd(gpu.base + RBBM_STATUS),
	    rd(gpu.base + RBBM_INT_0_STATUS), ring_at, rd(gmu.base + GMU_AO_GPU_CX_BUSY_STATUS),
	    rd(gmu.base + GMU_SPTPRAC_PWR_CLK_STATUS), fence_status);

	quiet = false;
	if (!step_core_start() || !step_core_commands()) {
		say("apollo gpu: THE 3D CORE DOES NOT TAKE COMMANDS AGAIN after the reset: it is given up for this run\n");
		wr(gmu.base + GMU_PWR_COL_KEEPALIVE, 0);
		quiet = true;
		return false;
	}
	wr(gmu.base + GMU_PWR_COL_KEEPALIVE, 0);
	quiet = true;

	/* everything the core had been given is given up: its word for how far it has got is the last submission's */
	bzero((void *)core_scratch.memory, 4096);
	*(volatile uint32_t *)(core_scratch.memory + GPU_FENCE_AT) = gpu_fence;
	to_memory(core_scratch.memory, 4096);
	gpu_timed = gpu_fence;
	say("apollo gpu: THE 3D CORE IS RECOVERED (%u time%s in this run); submissions up to %u are given up as done\n",
	    gpu_recoveries, gpu_recoveries == 1 ? "" : "s", gpu_fence);
	return true;
}

/*
 * The same when there is no time for asking: the kernel is on its way back to the loader (a panic, or an end
 * that did not go through apollo_adreno_stop()). Only the writes that keep the GPU block out of memory that is
 * no longer its to use.
 */
void
apollo_adreno_quiesce(void)
{
	if (gmu_runs && gmu.base != 0) {
		wr(gmu.base + GMU_CM3_SYSRESET, 1);
		gmu_runs = false;
	}
	if (core_runs && gbif.base != 0 && gpu.base != 0) {
		core_bus_halt(true);
		core_runs = false;
	}
	if (smmu.base != 0 && smmu_page != 0) {
		if (gmu_space_is_on) {
			wr(smmu.base + smmu_context(&gmu_space) + SMMU_CB_SCTLR, 0);
			gmu_space_is_on = false;
		}
		if (core_space_is_on) {
			wr(smmu.base + smmu_context(&core_space) + SMMU_CB_SCTLR, 0);
			core_space_is_on = false;
		}
	}
}

/* ---- the sequence ----------------------------------------------------------------------------------------- */

static bool (*const steps[])(void) = {
	NULL, step_outside, step_fuses, step_gpucc, step_cx_power, step_clocks, step_first_words,
	step_rscc, step_smmu, step_smmu_context, step_gmu_memory, step_trustzone, step_zap_signature,
	step_gmu_space, step_gmu_firmware, step_gmu_start, step_gmu_queues, step_core_power,
	step_core_space, step_zap_load, step_core_start, step_core_commands, step_frame,
};
#define STEPS ((uint32_t)(sizeof(steps) / sizeof(steps[0]) - 1))

/* everything the steps changed, back as it was found, in reverse (CX last: the clocks before their power) */
static void
put_back(void)
{
	step = 0;
	gmu_stop();
	if (undo_count == 0) {
		return;
	}
	say("apollo gpu: putting back the %u registers the steps changed\n", undo_count);
	while (undo_count > 0) {
		struct undo *u = &undo[--undo_count];

		put(u->b, u->name, u->offset, u->value);
		if (u->b == &gpucc && u->offset == GPUCC_CX_GDSCR) {
			for (int i = 0; i < 500 && (rd(gpucc.base + GPUCC_CX_GDSCR) & GDSC_POWER_ON); i++) {
				sleep_us(100);
			}
			say("apollo gpu: CX is %s\n", rd(gpucc.base + GPUCC_CX_GDSCR) & GDSC_POWER_ON ? "still on" : "off again");
		}
	}
}

void
apollo_adreno_init(void)
{
	uint32_t done = 0;

	if (!PE_parse_boot_argn("apollo_gpu", &last_step, sizeof(last_step)) || last_step == 0) {
		return;
	}
	PE_parse_boot_argn("apollo_gpu_pause", &pause_ms, sizeof(pause_ms));
	PE_parse_boot_argn("apollo_gpu_keep", &keep, sizeof(keep));
	PE_parse_boot_argn("apollo_gpu_speed", &speed_index, sizeof(speed_index));
	PE_parse_boot_argn("apollo_gpu_use", &use, sizeof(use));
	PE_parse_boot_argn("gpu_protect", &gpu_protect, sizeof(gpu_protect));    /* before step 18 maps the ring */
	if (last_step > STEPS) {
		say("apollo gpu: apollo_gpu=%u: there are %u steps so far\n", last_step, STEPS);
		last_step = STEPS;
	}
	if (use) {
		pause_ms = 0;                   /* nothing here is new any more: no waiting for the loader's notes */
		if (last_step > 21) {
			last_step = 21;         /* (no picture: the system draws) */
		}
	}
	say("apollo gpu: the Adreno 650, steps 1 to %u\n", last_step);
	for (step = 1; step <= last_step; step++) {
		if (!steps[step]()) {
			say("apollo gpu: STEP %u FAILED: stopping there\n", step);
			break;
		}
		say("apollo gpu: step %u done\n", step);
		done = step;
	}
	say("apollo gpu: %s: steps 1 to %u done\n", done == last_step ? "ALL DONE" : "stopped", done);
	if (use && done >= 21 && core_runs) {
		gpu_free[0].start = GPU_ADDRESSES_START;
		gpu_free[0].size = GPU_ADDRESSES_SIZE;
		gpu_free_count = 1;
		bzero((void *)core_scratch.memory, 4096);
		to_memory(core_scratch.memory, 4096);
		PE_parse_boot_argn("gpu_exact", &gpu_exact, sizeof(gpu_exact));
		PE_parse_boot_argn("gpu_log", &apollo_gpu_log, sizeof(apollo_gpu_log));
		PE_parse_boot_argn("gpu_recover", &gpu_recover, sizeof(gpu_recover));
		PE_parse_boot_argn("gpu_va", &apollo_gpu_va, sizeof(apollo_gpu_va));    /* before any client exists */
		PE_parse_boot_argn("apollo_gpu_dcvs", &gpu_dcvs, sizeof(gpu_dcvs));
		PE_parse_boot_argn("apollo_gpu_max", &gpu_max_level, sizeof(gpu_max_level));
		gpu_max_level = gpu_max_level < 1 ? 1 : gpu_max_level > 6 ? 6 : gpu_max_level;
		PE_parse_boot_argn("apollo_gpu_floor", &gpu_floor, sizeof(gpu_floor));    /* 0: no floor while frames come */
		gpu_floor = gpu_floor > 6 ? 6 : gpu_floor;
		gov_tm0 = ml_io_map(GOV_TSENS0_PHYS, 0x1000);
		gov_level = speed_index >= 1 && speed_index <= 6 ? speed_index : 2;
		{
			lck_grp_t *group = lck_grp_alloc_init("apollo_gpu", LCK_GRP_ATTR_NULL);

			gpu_times_lock = lck_mtx_alloc_init(group, LCK_ATTR_NULL);
			gpu_hfi_lock = lck_mtx_alloc_init(group, LCK_ATTR_NULL);
		}
		gpu_kept = true;
		if (gpu_dcvs) {
			thread_t governor;

			if (kernel_thread_start(gpu_governor_thread, NULL, &governor) == KERN_SUCCESS) {
				thread_set_thread_name(governor, "apollo gpu governor");
				thread_deallocate(governor);
			}
		}
		quiet = true;
		step = 0;
		say("apollo gpu: THE GPU STAYS ON for the system to draw with: the service ApolloGPU\n");
		ApolloGPUStart();
		return;
	}
	if (use) {
		say("apollo gpu: not kept: the steps did not get as far as a 3D core that takes commands; the system draws "
		    "with its software renderer\n");
	}
	if (keep) {
		say("apollo gpu: apollo_gpu_keep=1: what the steps turned on stays on (but the GMU is stopped)\n");
		step = 0;
		gmu_stop();
	} else {
		put_back();
	}
}
