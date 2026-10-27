/*
 * The display timing of the Xiaomi Mi 10T (apollo), read and, with a boot-arg, changed -- with boot-arg apollo_display=1
 * (without it nothing here runs) and, optionally, apollo_display_hz=N (60 to 144: the refresh rate to switch the panel to).
 *
 * The phone's UEFI left the display controller (the SM8250's DPU) running: a video-mode DSI panel, DSC-compressed, driven
 * by the controller's timing engine INTF_1. The registers of that timing engine are at physical 0x0ae6b800, 0x2c0 long
 * (the device tree's DSI ctrl0 "mdp_intf_base" 0xae6b800; Linux's catalog dpu_6_0_sm8250.h has intf_1 at 0xae01000 +
 * 0x6a800, the same). The offsets are those of Linux's drivers/gpu/drm/msm/disp/dpu1/dpu_hw_intf.c: TIMING_ENGINE_EN
 * 0x000, HSYNC_CTL 0x008 (bits 31:16 the period of a line in pixel clocks, bits 15:0 the width of the sync pulse),
 * VSYNC_PERIOD_F0 0x00c (the frame in pixel clocks: lines * the line's period), VSYNC_PULSE_WIDTH_F0 0x014,
 * DISPLAY_V_START_F0 0x01c, DISPLAY_V_END_F0 0x024, ACTIVE_V_START_F0 0x02c, ACTIVE_V_END_F0 0x034 (all of them in pixel
 * clocks as well: line * period + skew, so a line number is the value divided by the period), DISPLAY_HCTL 0x03c,
 * ACTIVE_HCTL 0x040, FRAME_LINE_COUNT_EN 0x0a8 (Linux writes 3 to count both frames and lines), FRAME_COUNT 0x0ac,
 * LINE_COUNT 0x0b0 and STATUS 0x26c (bit 0: the timing engine is running).
 *
 * The panel (the device tree's mdss_dsi_j3s_37_02_0a_dsc_video, whose only timing node is timing@0) is 1080x2400 at 144 Hz,
 * with a vertical front porch of 28 lines, back porch of 26 and a pulse of 2: 2456 lines to a frame. Its other rates (120,
 * 90 and 60, from qcom,dsi-supported-dfps-list) are made by a longer vertical front porch alone ("dfps_immediate_porch_mode_vfp"),
 * the pixel clock staying as it is. That is what apollo_display_hz=N does here: the line time is kept, the frame gets
 * the lines that make N, and the extra ones are all front porch.
 *
 * A change of the INTF's registers takes effect when the control block that drives it flushes it: the CTL blocks are at
 * 0x0ae02000 + 0x200 * n (n = 0 to 5, 0x1e0 long each); CTL_TOP is at 0x014, CTL_FLUSH at 0x018, CTL_INTF_ACTIVE at 0x0f4
 * and CTL_INTF_FLUSH at 0x110. On this generation INTF_1 is flushed by bit 1 of CTL_INTF_FLUSH together with bit 31 of
 * CTL_FLUSH. The CTL that drives INTF_1 is found by reading CTL_TOP and CTL_INTF_ACTIVE of all six (and logging them): the
 * one whose CTL_INTF_ACTIVE has bit 1 set, else the first whose CTL_TOP is not zero. If none is found, the timing is not
 * changed.
 *
 * What the framebuffer driver gets: the measured refresh rate (apollo_display_hz) and a wait for the moment the scan is
 * outside the picture (apollo_display_wait_scanout). The panel's memory is scanned out line by line; a frame copied into
 * it while the scan is inside the picture shows as a tear where the scan crosses the copy. A copy that begins right
 * after the scan has left the picture, and is faster than the scan is, is never crossed by it.
 *
 * Toward tear-free composition (the page flip instead of the copy):
 *  - apollo_dpu_dump=1: a read-only dump of the DPU as the UEFI left it (the source pipes, the CTLs, the layer mixers, and the
 *    SMMU stream of the display), with the pipe that scans the boot framebuffer marked.
 *  - apollo_display_flip(phys, stride) / apollo_display_flip_latched(): the pipe's SRC0_ADDR is written and the pipe's bit
 *    in CTL_FLUSH set; the hardware takes the new address at the next frame's start and clears the bit.
 *  - apollo_flip_test=S: 20 s after the init, two buffers (red, green) are flipped between for S seconds and timed.
 * The catalog offsets of Linux (catalog/dpu_6_0_sm8250.h) are relative to 0x0ae01000 (its ctl_0 is at 0x1000, and the CTL
 * here is at 0x0ae02000); the device tree's mdss_mdp reg "mdp_phys" is 0x0ae00000, size 0x84208, and its qcom,sde-sspp-off
 * (0x5000 0x7000 0x9000 0xb000 0x25000 0x27000 0x29000 0x2b000), qcom,sde-ctl-off (0x2000 + 0x200 n) and qcom,sde-mixer-off
 * (0x45000 + 0x1000 n) are relative to 0x0ae00000, i.e. the catalog's plus 0x1000: the offsets below are those.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <pexpert/pexpert.h>
#include <console/video_console.h>
#include <sys/systm.h>
#include <vm/memory_types.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
/*
 * The large maps of the dump and the flips (the MDP's 532 KB, the SMMU's) go into the kernel's map, not the io submap:
 * that one is small and every driver's registers stay in it for good -- with these in it too, hakios.apollo_cs35l41.c's
 * 384 KB no longer fitted and the kernel panicked on the first sound (25 October, "kmem_alloc ... failed with 3").
 */
extern vm_offset_t ml_io_map_unmappable(vm_offset_t phys_addr, vm_size_t size, unsigned int flags);
extern uint64_t kvtophys(vm_offset_t va);       /* pmap_paddr_t */
extern kern_return_t thread_terminate(thread_t thread);
extern void thread_set_thread_name(thread_t thread, const char *name);
extern struct vc_info vinfo;                    /* the boot framebuffer, as the text console knows it */

/* into the loader's log as well (as the other apollo drivers) */
extern void loader_log_putc(char c);

void apollo_display_init(void);
uint32_t apollo_display_hz(void);
bool apollo_display_wait_scanout(uint64_t max_ns);
void apollo_display_stats(uint32_t *waits, uint64_t *wait_ns_total);
bool apollo_display_flip(uint64_t phys, uint32_t stride);
bool apollo_display_flip_latched(void);
int apollo_scanout_get(unsigned *count, vm_offset_t va[3], uint64_t pa[3], uint32_t *stride);
bool apollo_scanout_flip(unsigned index, bool gpu);
bool apollo_scanout_latched(void);
int apollo_scanout_take(void);
void apollo_scanout_release(unsigned index);
bool apollo_scanout_gpu_info(unsigned index, uint64_t *phys, uint32_t *stride, uint32_t *width, uint32_t *height,
    uint64_t *bytes, vm_offset_t *va);
int apollo_scanout_acquire(uint32_t timeout_ms);
void apollo_scanout_mark(unsigned index, uint32_t swap);
int apollo_scanout_take_mark(uint32_t swap, uint32_t surface);
void apollo_scanout_wait(uint32_t timeout_us);
unsigned apollo_scanout_front(void);

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

/* ---- the registers ---------------------------------------------------------------------------------- */

#define INTF_PHYS               0x0ae6b800UL
#define INTF_SIZE               0x2c0UL
#define INTF_TIMING_EN          0x000
#define INTF_HSYNC_CTL          0x008
#define INTF_VSYNC_PERIOD       0x00c
#define INTF_VSYNC_PULSE        0x014
#define INTF_DISPLAY_V_START    0x01c
#define INTF_DISPLAY_V_END      0x024
#define INTF_ACTIVE_V_START     0x02c
#define INTF_ACTIVE_V_END       0x034
#define INTF_DISPLAY_HCTL       0x03c
#define INTF_ACTIVE_HCTL        0x040
#define INTF_FRAME_LINE_EN      0x0a8
#define INTF_FRAME_COUNT        0x0ac
#define INTF_LINE_COUNT         0x0b0
#define INTF_STATUS             0x26c
#define INTF_STATUS_RUNNING     0x1u

#define CTL_PHYS                0x0ae02000UL
#define CTL_STRIDE              0x200UL
#define CTL_SIZE                0x1e0UL
#define CTL_COUNT               6
#define CTL_TOP                 0x014
#define CTL_FLUSH               0x018
#define CTL_INTF_ACTIVE         0x0f4
#define CTL_INTF_FLUSH          0x110
#define CTL_FLUSH_INTF_BIT      (1u << 31)      /* CTL_FLUSH: the interface flush register is in use */
#define CTL_INTF_FLUSH_INTF1    (1u << 1)       /* CTL_INTF_FLUSH: INTF_1 */
#define CTL_INTF_ACTIVE_INTF1   (1u << 1)

/* the panel's native frame: 2400 + 28 + 26 + 2 lines at 144 Hz */
#define NATIVE_LINES            2456u
#define NATIVE_HZ               144u

#define MEASURE_MS              1000           /* (a frame more or less in 250 ms is 4 Hz at 60: too coarse to judge by) */
#define SETTLE_MS               100

static vm_offset_t intf;                /* the mapped INTF_1; 0: the driver did not start */
static vm_offset_t ctl_base[CTL_COUNT];
static int ctl_driving = -1;            /* the CTL that drives INTF_1, -1: none found */

/* the driver's state: what was read, and what the rate measured */
static uint32_t hsync_period;           /* pixel clocks to a line */
static uint32_t frame_lines;            /* lines to a frame, as the registers have it now */
static uint32_t active_start_line, active_end_line;
static uint32_t measured_mhz;           /* the refresh rate in millihertz, measured */

static uint32_t wait_count;
static uint64_t wait_total_ns;

/* the rate in tenths of a hertz, rounded, for the log */
static uint32_t
tenths(uint32_t mhz)
{
	return (mhz + 50) / 100;
}

/* frames a second, in millihertz, as the frame counter counts them over MEASURE_MS (0 if it does not count) */
static uint32_t
measure(void)
{
	uint64_t t0, t1, ns, frames;
	uint32_t c0, c1;

	c0 = rd(intf + INTF_FRAME_COUNT);
	t0 = mach_absolute_time();
	delay_for_interval(MEASURE_MS, 1000 * NSEC_PER_USEC);
	c1 = rd(intf + INTF_FRAME_COUNT);
	t1 = mach_absolute_time();
	absolutetime_to_nanoseconds(t1 - t0, &ns);
	if (ns == 0) {
		return 0;
	}
	frames = (uint32_t)(c1 - c0);
	return (uint32_t)(frames * 1000000000000ULL / ns);
}

/* the line's period and the frame's length, as the registers have them now */
static void
read_timing(void)
{
	hsync_period = rd(intf + INTF_HSYNC_CTL) >> 16;
	if (hsync_period == 0) {
		frame_lines = active_start_line = active_end_line = 0;
		return;
	}
	frame_lines = rd(intf + INTF_VSYNC_PERIOD) / hsync_period;
	active_start_line = rd(intf + INTF_ACTIVE_V_START) / hsync_period;
	active_end_line = rd(intf + INTF_ACTIVE_V_END) / hsync_period;
	if (active_end_line == 0) {
		/* Linux leaves the active window 0 where it is the whole display (dpu_hw_intf.c): the display window then */
		active_start_line = rd(intf + INTF_DISPLAY_V_START) / hsync_period;
		active_end_line = rd(intf + INTF_DISPLAY_V_END) / hsync_period;
	}
}

static void
log_timing(void)
{
	uint32_t hsync = rd(intf + INTF_HSYNC_CTL);

	say("apollo display: timing engine %s (status %#x), frame/line counters %#x\n",
	    (rd(intf + INTF_TIMING_EN) & 1) ? "on" : "off", rd(intf + INTF_STATUS), rd(intf + INTF_FRAME_LINE_EN));
	say("apollo display: hsync period %u pixel clocks, pulse %u; vsync period %u (%u lines), pulse %u\n",
	    hsync >> 16, hsync & 0xffff, rd(intf + INTF_VSYNC_PERIOD), frame_lines, rd(intf + INTF_VSYNC_PULSE));
	say("apollo display: display v %u..%u, active v %u..%u (lines %u..%u); display hctl %#x, active hctl %#x\n",
	    rd(intf + INTF_DISPLAY_V_START), rd(intf + INTF_DISPLAY_V_END), rd(intf + INTF_ACTIVE_V_START),
	    rd(intf + INTF_ACTIVE_V_END), active_start_line, active_end_line, rd(intf + INTF_DISPLAY_HCTL),
	    rd(intf + INTF_ACTIVE_HCTL));
}

/* the line time the measurement implies, and the rate the same line time gives with the native number of lines */
static uint32_t
native_mhz(void)
{
	return (uint32_t)((uint64_t)measured_mhz * frame_lines / NATIVE_LINES);
}

static void
log_rate(void)
{
	uint32_t line_ns = 0, native;

	if (measured_mhz != 0 && frame_lines != 0) {
		line_ns = (uint32_t)(1000000000000ULL / ((uint64_t)measured_mhz * frame_lines));
	}
	native = tenths(native_mhz());
	say("apollo display: measured %u.%u Hz over %u ms; a line takes about %u ns; with %u lines that line time is %u.%u Hz\n",
	    tenths(measured_mhz) / 10, tenths(measured_mhz) % 10, MEASURE_MS, line_ns, NATIVE_LINES, native / 10, native % 10);
}

/* ---- the CTL that drives INTF_1 ---------------------------------------------------------------------- */

static void
find_ctl(void)
{
	int first_top = -1, active = -1;

	for (int n = 0; n < CTL_COUNT; n++) {
		uint32_t top, intf_active;

		ctl_base[n] = ml_io_map(CTL_PHYS + CTL_STRIDE * (uint32_t)n, CTL_SIZE);
		top = rd(ctl_base[n] + CTL_TOP);
		intf_active = rd(ctl_base[n] + CTL_INTF_ACTIVE);
		say("apollo display: ctl_%d: CTL_TOP %#x, CTL_INTF_ACTIVE %#x\n", n, top, intf_active);
		if (active < 0 && (intf_active & CTL_INTF_ACTIVE_INTF1)) {
			active = n;
		}
		if (first_top < 0 && top != 0) {
			first_top = n;
		}
	}
	ctl_driving = active >= 0 ? active : first_top;
	if (ctl_driving >= 0) {
		say("apollo display: ctl_%d drives INTF_1 (%s)\n", ctl_driving,
		    active >= 0 ? "its CTL_INTF_ACTIVE has the bit" : "the first with a CTL_TOP");
	} else {
		say("apollo display: no CTL drives INTF_1: the timing will not be changed\n");
	}
}

/* the INTF's registers written are taken over at the next frame's start, when the CTL flushes them */
static void
flush(void)
{
	wr(ctl_base[ctl_driving] + CTL_INTF_FLUSH, CTL_INTF_FLUSH_INTF1);
	wr(ctl_base[ctl_driving] + CTL_FLUSH, CTL_FLUSH_INTF_BIT);
}

/* ---- the rate -------------------------------------------------------------------------------------- */

/* switch to `hz` by the vertical front porch alone: the line time stays, the frame has the lines that make `hz` */
static void
set_rate(uint32_t hz)
{
	uint64_t lines;
	uint32_t old_period, new_lines, low, high, got;

	if (!(rd(intf + INTF_STATUS) & INTF_STATUS_RUNNING)) {
		say("apollo display: refusing to change the rate: the timing engine is not running\n");
		return;
	}
	if (measured_mhz < 30000 || measured_mhz > 150000) {
		say("apollo display: refusing to change the rate: the measured %u.%u Hz is not between 30 and 150\n",
		    tenths(measured_mhz) / 10, tenths(measured_mhz) % 10);
		return;
	}
	if ((uint64_t)measured_mhz * 100 >= (uint64_t)hz * 1000 * 97 && (uint64_t)measured_mhz * 100 <= (uint64_t)hz * 1000 * 103) {
		say("apollo display: the panel runs at %u Hz already (measured %u.%u): nothing to change\n", hz,
		    tenths(measured_mhz) / 10, tenths(measured_mhz) % 10);
		return;
	}
	if (ctl_driving < 0) {
		say("apollo display: refusing to change the rate: no CTL drives INTF_1\n");
		return;
	}
	/* round(lines * measured / N) */
	lines = ((uint64_t)frame_lines * measured_mhz + (uint64_t)hz * 500) / ((uint64_t)hz * 1000);
	new_lines = (uint32_t)lines;
	low = NATIVE_LINES - 2;
	high = NATIVE_LINES * NATIVE_HZ / 60 + 50;
	if (new_lines < active_end_line + 2) {
		say("apollo display: refusing to change the rate: %u lines would leave no front porch (the picture ends at line %u)\n",
		    new_lines, active_end_line);
		return;
	}
	if (new_lines < low || new_lines > high) {
		say("apollo display: refusing to change the rate: %u lines is not between %u and %u\n", new_lines, low, high);
		return;
	}
	/* the line time must be that of a 144 Hz frame of 2456 lines, within 3% */
	if (native_mhz() < NATIVE_HZ * 1000 * 97 / 100 || native_mhz() > NATIVE_HZ * 1000 * 103 / 100) {
		say("apollo display: refusing to change the rate: the line time is not that of a native %u Hz panel\n",
		    NATIVE_HZ);
		return;
	}

	old_period = rd(intf + INTF_VSYNC_PERIOD);
	say("apollo display: %u Hz: %u lines (was %u), vsync period %u (was %u)\n", hz, new_lines, frame_lines,
	    new_lines * hsync_period, old_period);
	wr(intf + INTF_VSYNC_PERIOD, new_lines * hsync_period);
	flush();
	delay_for_interval(SETTLE_MS, 1000 * NSEC_PER_USEC);
	measured_mhz = measure();
	read_timing();
	got = measured_mhz;
	say("apollo display: after the change vsync period reads %u (%u lines)\n", rd(intf + INTF_VSYNC_PERIOD), frame_lines);
	log_rate();
	if (got < hz * 1000 * 95 / 100 || got > hz * 1000 * 105 / 100) {
		say("apollo display: %u.%u Hz is not %u Hz: the change is undone\n", tenths(got) / 10, tenths(got) % 10, hz);
		wr(intf + INTF_VSYNC_PERIOD, old_period);
		flush();
		delay_for_interval(SETTLE_MS, 1000 * NSEC_PER_USEC);
		measured_mhz = measure();
		read_timing();
		log_rate();
	} else {
		say("apollo display: the panel runs at %u.%u Hz\n", tenths(got) / 10, tenths(got) % 10);
	}
}

/* ---- for the framebuffer driver --------------------------------------------------------------------- */

/* the refresh rate, in whole hertz; 0 if the driver did not start */
uint32_t
apollo_display_hz(void)
{
	if (intf == 0) {
		return 0;
	}
	return (measured_mhz + 500) / 1000;
}

/*
 * Wait until the scan is outside the picture (before its first line or from its end on), for at most max_ns. True if
 * it was found so, or got so; false if the driver did not start or the time ran out.
 */
bool
apollo_display_wait_scanout(uint64_t max_ns)
{
	uint64_t start, limit, now;
	bool found = false;

	if (intf == 0 || active_end_line == 0) {
		return false;
	}
	start = mach_absolute_time();
	nanoseconds_to_absolutetime(max_ns, &limit);
	for (;;) {
		uint32_t line = rd(intf + INTF_LINE_COUNT);

		if (line < active_start_line || line >= active_end_line) {
			found = true;
			break;
		}
		now = mach_absolute_time();
		if (now - start >= limit) {
			break;
		}
		/*
		 * Not a spin for the whole of the picture: the scan's lines go at a known pace, so sleep until shortly
		 * before it leaves the picture (a frame's time / its lines), and look again.
		 */
		if (measured_mhz != 0 && frame_lines != 0) {
			uint64_t line_ns = 1000000000000ULL / ((uint64_t)measured_mhz * frame_lines);
			uint64_t left_ns = (uint64_t)(active_end_line - line) * line_ns;

			if (left_ns > 300000) {
				delay_for_interval((uint32_t)((left_ns - 200000) / 1000), NSEC_PER_USEC);
			}
		}
	}
	now = mach_absolute_time();
	absolutetime_to_nanoseconds(now - start, &limit);
	wait_count++;
	wait_total_ns += limit;
	return found;
}

void
apollo_display_stats(uint32_t *waits, uint64_t *wait_ns_total)
{
	if (waits != NULL) {
		*waits = wait_count;
	}
	if (wait_ns_total != NULL) {
		*wait_ns_total = wait_total_ns;
	}
}

/* ---- the DPU: source pipes, CTLs, mixers (apollo_dpu_dump=1), and the page flip -------------------------- */

/* the MDP as the device tree has it (mdss_mdp reg "mdp_phys"): 0x0ae00000, 0x84208 long; mapped to the page after */
#define MDP_PHYS                0x0ae00000UL
#define MDP_SIZE                0x85000UL

/*
 * Source pipes: the device tree's qcom,sde-sspp-off (relative to MDP_PHYS: Linux's dpu_6_0_sm8250.h sm8250_sspp, sspp_0 to
 * sspp_3 = VIG0..3 at 0x4000, 0x6000, 0x8000, 0xa000 and sspp_8 to sspp_11 = DMA0..3 at 0x24000, 0x26000, 0x28000, 0x2a000,
 * all plus the catalog's base 0x1000). The bit of each in CTL_FLUSH is dpu_hw_ctl.c's dpu_hw_ctl_update_pending_flush_sspp():
 * VIG0 0, VIG1 1, VIG2 2, VIG3 18, DMA0 11, DMA1 12, DMA2 24, DMA3 25.
 */
#define SSPP_COUNT              8
static const struct {
	const char *name;
	uint32_t offset;
	uint32_t flush_bit;
} sspp_table[SSPP_COUNT] = {
	{ "VIG0", 0x05000, 0 }, { "VIG1", 0x07000, 1 }, { "VIG2", 0x09000, 2 }, { "VIG3", 0x0b000, 18 },
	{ "DMA0", 0x25000, 11 }, { "DMA1", 0x27000, 12 }, { "DMA2", 0x29000, 24 }, { "DMA3", 0x2b000, 25 },
};
/* the registers of a source pipe: dpu_hw_sspp.c (SSPP_SRC_SIZE 0x00, SSPP_SRC_XY 0x08, SSPP_OUT_SIZE 0x0c, SSPP_OUT_XY 0x10,
 * SSPP_SRC0_ADDR 0x14, SSPP_SRC_YSTRIDE0 0x24, SSPP_SRC_FORMAT 0x30, SSPP_SRC_UNPACK_PATTERN 0x34, SSPP_SRC_OP_MODE 0x38);
 * 0x04 (the image size of the source) is not in Linux's file: the vendor's sde_hw_sspp.c names it SSPP_SRC_IMG_SIZE */
#define SSPP_SRC_SIZE           0x00    /* height << 16 | width, of the rectangle read */
#define SSPP_SRC_IMG_SIZE       0x04
#define SSPP_SRC_XY             0x08
#define SSPP_OUT_SIZE           0x0c
#define SSPP_OUT_XY             0x10
#define SSPP_SRC0_ADDR          0x14    /* 32 bits: the SMMU's input address (a physical one if it passes it) */
#define SSPP_SRC_YSTRIDE0       0x24    /* [15:0] the stride of plane 0, [31:16] of plane 1 (dpu_hw_sspp_setup_sourceaddress, _setup_rects) */
#define SSPP_SRC_FORMAT         0x30
#define SSPP_SRC_UNPACK_PATTERN 0x34    /* element k (memory order) in bits 8k+1:8k: 0 G, 1 B, 2 R, 3 alpha (dpu_hw_sspp.c: C0_G_Y 0, C1_B_Cb 1, C2_R_Cr 2, C3_ALPHA 3) */
#define SSPP_SRC_OP_MODE        0x38

/* the CTL's registers, dpu_hw_ctl.c */
#define CTL_START               0x01c
#define CTL_MERGE_3D_ACTIVE     0x0e4
#define CTL_DSC_ACTIVE          0x0e8
#define CTL_WB_ACTIVE           0x0ec
#define CTL_CWB_ACTIVE          0x0f0
#define CTL_CDM_ACTIVE          0x0f8
#define CTL_FETCH_PIPE_ACTIVE   0x0fc
#define CTL_MERGE_3D_FLUSH      0x100
#define CTL_DSC_FLUSH           0x104
#define CTL_PIPE_ACTIVE         0x12c
#define CTL_LAYER_ACTIVE        0x130
#define CTL_INTF_MASTER         0x134
/* CTL_LAYER(lm): lm 0..4 at 4 * lm, lm 5 at 0x024; _EXT 0x40 + 4 lm, _EXT2 0x70 + 4 lm, _EXT3 0xa0 + 4 lm, _EXT4 0xb8 + 4 lm */
static inline uint32_t
ctl_layer(uint32_t lm)
{
	return lm == 5 ? 0x024 : 4 * lm;
}

/* the layer mixers: the device tree's qcom,sde-mixer-off 0x45000 + 0x1000 n (Linux's lm_0 .. lm_5 at 0x44000 + 0x1000 n, plus 0x1000) */
#define LM_COUNT                6
#define LM_PHYS_OFF             0x45000
#define LM_OP_MODE              0x00    /* dpu_hw_lm.c */
#define LM_OUT_SIZE             0x04
#define LM_BG_SRC_SEL_V12       0x14
#define LM_BORDER_COLOR_0_V12   0x1c
#define LM_BORDER_COLOR_1_V12   0x20
#define LM_STAGE0               0x20    /* sdm845_lm_sblk blendstage_base: 0x20, 0x38, 0x50 ... (dpu_hw_catalog.c), 0x18 apart */
#define LM_STAGE_STEP           0x18
#define LM_BLEND0_OP            0x00    /* from a stage's base: LM_BLEND0_OP 0x00, LM_BLEND0_FG_SRC_SEL_V12 0x04, _CONST_ALPHA_V12 0x08 */
#define LM_BLEND0_FG_SRC_SEL    0x04
#define LM_BLEND0_CONST_ALPHA   0x08

/* the display's SMMU stream: the device tree's smmu_sde_unsec_cb iommus <&apps_smmu 0x820 0x402> (secure: 0x821 0x400) */
#define SMMU_PHYS               0x15000000UL
#define SMMU_SIZE               0x100000UL
#define SMMU_SCR0               0x000   /* bit 0 CLIENTPD, bit 3 EXIDENABLE, bit 10 USFCFG (fault on an unmatched stream) */
#define SMMU_IDR0               0x020   /* [7:0] number of stream match registers */
#define SMMU_IDR1               0x024   /* [31] 64 KB pages, [30:28] log2(pages)-1, [7:0] number of context banks */
#define SMMU_SMR(n)             (0x800 + 4 * (n))       /* bit 31 valid, [30:16] mask, [14:0] stream (as hakios.apollo_pcie.c) */
#define SMMU_S2CR(n)            (0xc00 + 4 * (n))       /* [17:16] 0 translate, 1 bypass, 2 fault; [7:0] context */
#define SMMU_CB_SCTLR           0x00    /* bit 0 M: translation on */
#define SMMU_CB_TTBR0           0x20
#define SMMU_CB_TCR             0x30
#define SMMU_CB_FSR             0x58
#define MDSS_STREAM             0x820
#define MDSS_STREAM_MASK        0x402
#define MDSS_SEC_STREAM         0x821
#define MDSS_SEC_MASK           0x400

#define FLIP_TIMEOUT_US         100000  /* a flip that has not latched by then is a failure */

static vm_offset_t mdp;                 /* the mapped MDP, 0: not yet */
static int sspp_found = -1;             /* the pipe that scans the boot framebuffer, -1: none */
static bool smmu_blocks;                /* the display's stream translates (or faults): flips are refused */
static uint64_t fb_phys;                /* the boot framebuffer's physical address, 0: not found */
static uint32_t flip_stride;            /* the stride of the pipe now, as the flip keeps it */
static uint32_t boot_stride;            /* ... and the boot framebuffer's, which apollo_display_quiesce() puts back */
static uint32_t flip_bit;               /* the pipe's bit in CTL_FLUSH */
static uint64_t flip_last_us;           /* the last flip's time to latch, microseconds, ~0: it did not */
static uint32_t flip_frames_before, flip_frames_after;

/*
 * The boot framebuffer's physical address. The text console's vinfo (the structure MiniIOMobileFramebuffer takes
 * v_baseaddr from) has the kernel's address of it in v_baseaddr and the physical one in v_physaddr (video_console.c,
 * initialize_screen: it was given by the loader, bit 0 of the boot args' Video.v_baseAddr was set or the VM was not up);
 * kvtophys() of v_baseaddr is the cross-check.
 */
static uint64_t
framebuffer_phys(void)
{
	uint64_t from_vinfo = vinfo.v_physaddr, from_va = 0;

	if (vinfo.v_baseaddr != 0) {
		from_va = kvtophys((vm_offset_t)vinfo.v_baseaddr);
	}
	say("apollo dpu: framebuffer: vinfo v_baseaddr %#lx, v_physaddr %#llx, kvtophys(v_baseaddr) %#llx; PE_state v_baseAddr %#lx\n",
	    (unsigned long)vinfo.v_baseaddr, from_vinfo, from_va, (unsigned long)PE_state.video.v_baseAddr);
	if (from_va != 0) {
		if (from_vinfo != 0 && from_vinfo != from_va) {
			say("apollo dpu: framebuffer: v_physaddr and kvtophys() differ: the pipe match uses kvtophys()\n");
		}
		return from_va;
	}
	return from_vinfo;
}

/* read-only: the stream of the display in the application processors' SMMU; sets smmu_blocks */
static void
smmu_look(void)
{
	vm_offset_t base = ml_io_map_unmappable(SMMU_PHYS, SMMU_SIZE, VM_WIMG_IO);
	uint32_t scr0 = rd(base + SMMU_SCR0), idr0 = rd(base + SMMU_IDR0), idr1 = rd(base + SMMU_IDR1);
	uint32_t matches = idr0 & 0xff, contexts = idr1 & 0xff, page = (idr1 >> 31) ? 0x10000 : 0x1000, pages = 2u << (idr1 >> 28 & 7);
	uint32_t found = 0;

	say("apollo dpu: smmu: SCR0 %#x (CLIENTPD %u, USFCFG %u), %u match registers, %u contexts, pages of %#x\n", scr0, scr0 & 1,
	    scr0 >> 10 & 1, matches, contexts, page);
	if (scr0 & 1u << 3) {
		say("apollo dpu: smmu: 16-bit stream ids, not read\n");
		return;
	}
	for (uint32_t n = 0; n < matches && SMMU_SMR(n) < 0xc00; n++) {
		uint32_t smr = rd(base + SMMU_SMR(n)), s2cr = rd(base + SMMU_S2CR(n));
		uint32_t stream = smr & 0x7fff, mask = smr >> 16 & 0x7fff, type = s2cr >> 16 & 3;
		bool unsec, sec;

		if (!(smr >> 31)) {
			continue;
		}
		/* the entry covers the display's stream if the ids agree outside both masks (the device tree's mask bits are "don't care") */
		unsec = ((stream ^ MDSS_STREAM) & ~mask & ~MDSS_STREAM_MASK) == 0;
		sec = ((stream ^ MDSS_SEC_STREAM) & ~mask & ~MDSS_SEC_MASK) == 0;
		if (!unsec && !sec) {
			continue;
		}
		found++;
		say("apollo dpu: smmu: smr %u: stream %#x mask %#x (%s) s2cr %#x: %s, context %u\n", n, stream, mask,
		    unsec ? "mdss unsecure" : "mdss secure", s2cr, type == 0 ? "translate" : type == 1 ? "bypass" : "fault", s2cr & 0xff);
		if (type == 0 && (s2cr & 0xff) < contexts) {
			vm_offset_t cb = base + (pages + (s2cr & 0xff)) * page;
			uint32_t sctlr = rd(cb + SMMU_CB_SCTLR);

			say("apollo dpu: smmu: context %u: SCTLR %#x (M %u), TTBR0 %#llx, TCR %#x, FSR %#x\n", s2cr & 0xff, sctlr, sctlr & 1,
			    (uint64_t)rd(cb + SMMU_CB_TTBR0) | (uint64_t)rd(cb + SMMU_CB_TTBR0 + 4) << 32, rd(cb + SMMU_CB_TCR),
			    rd(cb + SMMU_CB_FSR));
			if (unsec && (sctlr & 1)) {
				smmu_blocks = true;     /* an address of ours would go through that context's tables */
			}
		} else if (type == 2 && unsec) {
			smmu_blocks = true;
		}
	}
	if (found == 0) {
		say("apollo dpu: smmu: no match register covers streams %#x/%#x: %s\n", MDSS_STREAM,
		    (scr0 & 1) ? "the SMMU is bypassed for clients (CLIENTPD)" :
		    (scr0 >> 10 & 1) ? "unmatched streams fault (USFCFG)" : "unmatched streams pass through");
		if (!(scr0 & 1) && (scr0 >> 10 & 1)) {
			smmu_blocks = true;
		}
	}
	say("apollo dpu: smmu: the display's stream %s\n", smmu_blocks ? "TRANSLATES: flips are refused" : "is not translated");
}

static void
dump_sspp(void)
{
	for (int i = 0; i < SSPP_COUNT; i++) {
		vm_offset_t b = mdp + sspp_table[i].offset;

		say("apollo dpu: %s@%#lx size %#x img %#x xy %#x out %#x/%#x addr %#x stride %#x fmt %#x unpack %#x op %#x%s\n",
		    sspp_table[i].name, MDP_PHYS + sspp_table[i].offset, rd(b + SSPP_SRC_SIZE), rd(b + SSPP_SRC_IMG_SIZE),
		    rd(b + SSPP_SRC_XY), rd(b + SSPP_OUT_SIZE), rd(b + SSPP_OUT_XY), rd(b + SSPP_SRC0_ADDR), rd(b + SSPP_SRC_YSTRIDE0),
		    rd(b + SSPP_SRC_FORMAT), rd(b + SSPP_SRC_UNPACK_PATTERN), rd(b + SSPP_SRC_OP_MODE),
		    i == sspp_found ? "  <== scans the boot framebuffer" : "");
	}
}

static void
dump_ctl(void)
{
	for (uint32_t n = 0; n < CTL_COUNT; n++) {
		vm_offset_t c = mdp + (CTL_PHYS - MDP_PHYS) + CTL_STRIDE * n;

		say("apollo dpu: ctl_%u%s: top %#x flush %#x intf_active %#x intf_flush %#x merge3d %#x/%#x dsc %#x/%#x wb %#x cwb %#x cdm %#x "
		    "fetch_pipe %#x pipe %#x layer_active %#x intf_master %#x\n", n, (int)n == ctl_driving ? " (drives INTF_1)" : "",
		    rd(c + CTL_TOP), rd(c + CTL_FLUSH), rd(c + CTL_INTF_ACTIVE), rd(c + CTL_INTF_FLUSH), rd(c + CTL_MERGE_3D_ACTIVE),
		    rd(c + CTL_MERGE_3D_FLUSH), rd(c + CTL_DSC_ACTIVE), rd(c + CTL_DSC_FLUSH), rd(c + CTL_WB_ACTIVE),
		    rd(c + CTL_CWB_ACTIVE), rd(c + CTL_CDM_ACTIVE), rd(c + CTL_FETCH_PIPE_ACTIVE), rd(c + CTL_PIPE_ACTIVE),
		    rd(c + CTL_LAYER_ACTIVE), rd(c + CTL_INTF_MASTER));
		for (uint32_t lm = 0; lm < LM_COUNT; lm++) {
			uint32_t l = rd(c + ctl_layer(lm)), e = rd(c + 0x40 + 4 * lm), e2 = rd(c + 0x70 + 4 * lm), e3 = rd(c + 0xa0 + 4 * lm),
			    e4 = rd(c + 0xb8 + 4 * lm);

			if (l | e | e2 | e3 | e4) {
				say("apollo dpu: ctl_%u layer(lm_%u) %#x ext %#x ext2 %#x ext3 %#x ext4 %#x\n", n, lm, l, e, e2, e3, e4);
			}
		}
	}
}

static void
dump_lm(void)
{
	for (uint32_t n = 0; n < LM_COUNT; n++) {
		vm_offset_t l = mdp + LM_PHYS_OFF + 0x1000 * n;
		uint32_t out = rd(l + LM_OUT_SIZE);

		if (out == 0 && rd(l + LM_OP_MODE) == 0) {
			continue;
		}
		say("apollo dpu: lm_%u: op_mode %#x out_size %#x bg_src_sel %#x border %#x/%#x; stage0 op %#x fg_src %#x alpha %#x, "
		    "stage1 op %#x fg_src %#x, stage2 op %#x fg_src %#x\n", n, rd(l + LM_OP_MODE), out, rd(l + LM_BG_SRC_SEL_V12),
		    rd(l + LM_BORDER_COLOR_0_V12), rd(l + LM_BORDER_COLOR_1_V12), rd(l + LM_STAGE0 + LM_BLEND0_OP),
		    rd(l + LM_STAGE0 + LM_BLEND0_FG_SRC_SEL), rd(l + LM_STAGE0 + LM_BLEND0_CONST_ALPHA),
		    rd(l + LM_STAGE0 + LM_STAGE_STEP + LM_BLEND0_OP), rd(l + LM_STAGE0 + LM_STAGE_STEP + LM_BLEND0_FG_SRC_SEL),
		    rd(l + LM_STAGE0 + 2 * LM_STAGE_STEP + LM_BLEND0_OP), rd(l + LM_STAGE0 + 2 * LM_STAGE_STEP + LM_BLEND0_FG_SRC_SEL));
	}
}

/* map the MDP, find the pipe that scans the boot framebuffer, look at the SMMU; the dump on request. Needs find_ctl() done. */
static void
dpu_setup(bool dump)
{
	mdp = ml_io_map_unmappable(MDP_PHYS, MDP_SIZE, VM_WIMG_IO);
	fb_phys = framebuffer_phys();
	for (int i = 0; i < SSPP_COUNT; i++) {
		if (fb_phys != 0 && rd(mdp + sspp_table[i].offset + SSPP_SRC0_ADDR) == (uint32_t)fb_phys && fb_phys < (1ull << 32)) {
			sspp_found = i;
			break;
		}
	}
	if (sspp_found >= 0) {
		vm_offset_t b = mdp + sspp_table[sspp_found].offset;

		flip_bit = 1u << sspp_table[sspp_found].flush_bit;
		flip_stride = rd(b + SSPP_SRC_YSTRIDE0) & 0xffff;
		boot_stride = flip_stride;
		say("apollo dpu: %s's SRC0_ADDR is the framebuffer %#llx (stride %u, flush bit %u)\n", sspp_table[sspp_found].name,
		    fb_phys, flip_stride, sspp_table[sspp_found].flush_bit);
	} else {
		say("apollo dpu: no source pipe has SRC0_ADDR %#llx: no flips\n", fb_phys);
	}
	if (dump) {
		dump_sspp();
		dump_ctl();
		dump_lm();
	}
	smmu_look();
}

static bool
flip_ready(void)
{
	return mdp != 0 && sspp_found >= 0 && ctl_driving >= 0 && !smmu_blocks;
}

/*
 * Page flip: the pipe's SRC0_ADDR (and SRC_YSTRIDE0 if the stride differs) is written, and last the pipe's bit in the
 * driving CTL's CTL_FLUSH (dpu_hw_ctl_trigger_flush(): DPU_REG_WRITE(CTL_FLUSH, pending_flush_mask), the pipe's bit from
 * dpu_hw_ctl_update_pending_flush_sspp()); the hardware takes the registers at the frame's start and clears the bit.
 * The address is 32 bits (SRC0_ADDR), so the buffer must be below 4 GB. False: nothing was written.
 */
bool
apollo_display_flip(uint64_t phys, uint32_t stride)
{
	vm_offset_t b, c;

	if (!flip_ready()) {
		return false;
	}
	if (phys >= (1ull << 32) || (phys & 0xff) != 0 || stride == 0 || stride > 0xffff) {
		say("apollo dpu: flip refused: address %#llx (below 4 GB, 256-byte aligned) or stride %u unusable\n", phys, stride);
		return false;
	}
	b = mdp + sspp_table[sspp_found].offset;
	c = ctl_base[ctl_driving];
	flip_frames_before = rd(intf + INTF_FRAME_COUNT);
	wr(b + SSPP_SRC0_ADDR, (uint32_t)phys);
	if (stride != flip_stride) {
		wr(b + SSPP_SRC_YSTRIDE0, (rd(b + SSPP_SRC_YSTRIDE0) & 0xffff0000u) | stride);
		flip_stride = stride;
	}
	__asm__ volatile ("dsb sy" : : : "memory");
	wr(c + CTL_FLUSH, flip_bit);    /* last */
	return true;
}

/*
 * The run's end (osfmk/arm64/loader_return.c, after a panic too: no lock, no sleep, no log): the pipe back on the boot
 * framebuffer, which the loader draws its screen into and reads screenshots from. Latched at the next frame start.
 */
void
apollo_display_quiesce(void)
{
	if (!flip_ready() || fb_phys == 0 || fb_phys >= (1ull << 32)) {
		return;
	}
	wr(mdp + sspp_table[sspp_found].offset + SSPP_SRC0_ADDR, (uint32_t)fb_phys);
	if (boot_stride != 0) {
		/* (frames the GPU drew have a stride of 64-byte multiples, apollo_gpu_present=1: the boot framebuffer's is not) */
		vm_offset_t b = mdp + sspp_table[sspp_found].offset;

		wr(b + SSPP_SRC_YSTRIDE0, (rd(b + SSPP_SRC_YSTRIDE0) & 0xffff0000u) | boot_stride);
	}
	__asm__ volatile ("dsb sy" : : : "memory");
	wr(ctl_base[ctl_driving] + CTL_FLUSH, flip_bit);
}

/* the flush bit has been cleared by the hardware: the last flip has taken effect */
bool
apollo_display_flip_latched(void)
{
	if (!flip_ready()) {
		return false;
	}
	return (rd(ctl_base[ctl_driving] + CTL_FLUSH) & flip_bit) == 0;
}

/* wait for the latch of the flip just made; the time in microseconds (or ~0 if it did not in FLIP_TIMEOUT_US) */
static uint64_t
flip_wait_latch(void)
{
	uint64_t t0 = mach_absolute_time(), now, us;

	for (;;) {
		bool done = apollo_display_flip_latched();

		now = mach_absolute_time();
		absolutetime_to_nanoseconds(now - t0, &us);
		us /= 1000;
		if (done) {
			flip_frames_after = rd(intf + INTF_FRAME_COUNT);
			flip_last_us = us;
			return us;
		}
		if (us >= FLIP_TIMEOUT_US) {
			flip_frames_after = rd(intf + INTF_FRAME_COUNT);
			flip_last_us = ~0ull;
			return ~0ull;
		}
	}
}

/* ---- the self-test (apollo_flip_test=S) --------------------------------------------------------------- */

#define TEST_DELAY_MS           20000
#define TEST_HOLD_MS            3000
#define TEST_HEIGHT_MAX         4096

static uint32_t flip_test_seconds;

struct flip_stats {
	uint32_t requested, in_frame, missed;
	uint64_t min_us, max_us, sum_us;
	uint32_t max_frames;
};

static void
stats_clear(struct flip_stats *st)
{
	st->requested = st->in_frame = st->missed = st->max_frames = 0;
	st->min_us = ~0ull;
	st->max_us = st->sum_us = 0;
}

/* a flip, its latch awaited; false if it did not latch (a miss), and the time and frames counted */
static bool
timed_flip(struct flip_stats *st, uint64_t phys, uint32_t stride)
{
	uint64_t us, frame_us = measured_mhz ? 1000000000ull / measured_mhz : 16667;
	uint32_t frames;

	st->requested++;
	if (!apollo_display_flip(phys, stride)) {
		st->missed++;
		return false;
	}
	us = flip_wait_latch();
	if (us == ~0ull) {
		st->missed++;
		return false;
	}
	frames = flip_frames_after - flip_frames_before;
	if (frames > st->max_frames) {
		st->max_frames = frames;
	}
	if (us <= frame_us + frame_us / 10) {
		st->in_frame++;
	}
	st->sum_us += us;
	if (us < st->min_us) {
		st->min_us = us;
	}
	if (us > st->max_us) {
		st->max_us = us;
	}
	return true;
}

static void
stats_log(const char *what, const struct flip_stats *st)
{
	uint32_t done = st->requested - st->missed;

	say("apollo flip test: %s: %u flips requested, %u latched within one frame, latch %llu/%llu/%llu us (min/mean/max), "
	    "%u missed, at most %u frames passed\n", what, st->requested, st->in_frame, done ? st->min_us : 0,
	    done ? st->sum_us / done : 0, st->max_us, st->missed, st->max_frames);
}

/* the 32-bit pixel of a colour in the pipe's byte order (SRC_UNPACK_PATTERN), alpha at its maximum */
static uint32_t
pixel(uint32_t pattern, uint32_t component /* 0 G, 1 B, 2 R */)
{
	uint32_t value = 0;

	for (uint32_t k = 0; k < 4; k++) {
		uint32_t element = pattern >> (8 * k) & 3;

		if (element == 3 || element == component) {
			value |= 0xffu << (8 * k);
		}
	}
	return value;
}

static void
fill_buffer(vm_offset_t buffer, size_t bytes, uint32_t value)
{
	uint32_t *p = (uint32_t *)buffer;

	for (size_t i = 0; i < bytes / 4; i++) {
		p[i] = value;
	}
	for (vm_offset_t line = buffer; line < buffer + bytes; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__asm__ volatile ("dsb sy");
}

/* the original picture back, its latch verified */
static void
restore_original(uint32_t stride)
{
	struct flip_stats st;

	stats_clear(&st);
	if (timed_flip(&st, fb_phys, stride)) {
		say("apollo flip test: the boot framebuffer %#llx is back, latched in %llu us\n", fb_phys, flip_last_us);
	} else {
		say("apollo flip test: the boot framebuffer %#llx was written back but the flip did not latch in %u ms\n", fb_phys,
		    FLIP_TIMEOUT_US / 1000);
	}
	/* two frames before the buffers are let go: the scan may still be reading the old one */
	delay_for_interval(100, NSEC_PER_MSEC);
}

/*
 * The two buffers of the flips, taken while the kernel starts: 20 s in, 10 MB in one piece below 4 GB (the pipe's
 * addresses are 32 bits) were no longer to be had (25 October) -- as the Wi-Fi's DMA pool, they are taken at boot.
 */
static vm_offset_t flip_buf[2];
static uint64_t flip_phys[2];
static size_t flip_bytes;
static uint32_t flip_gpu_stride;        /* apollo_gpu_present=1: the stride of frames the GPU draws (0: none are), see flip_reserve() */
static uint32_t flip_height;
static bool flip_reserved;      /* the buffers are the display reservation's: never freed */

/*
 * They come out of the display's own reservation: the UEFI keeps 0x9c000000 + 0x2400000 for it (apolloPkg's
 * MemoryMapLib.c, "Display Reserved"; Android's tree: cont_splash_region@9c000000, 0x2300000), the kernel never has it,
 * and the boot framebuffer (fb_phys) takes only its first frame -- two more fit behind it. Write-combined, as the
 * panel's memory is: nothing is held back in the processors' caches. (In one piece below 4 GB from the kernel's
 * memory there was none, at 20 s or at boot: 25 October.)
 */
#define DISPLAY_RESERVED_PHYS   0x9c000000ull
#define DISPLAY_RESERVED_SIZE   0x2400000ull

static void
flip_reserve(void)
{
	vm_offset_t b = mdp + sspp_table[sspp_found].offset;
	uint32_t stride = rd(b + SSPP_SRC_YSTRIDE0) & 0xffff, height = rd(b + SSPP_SRC_SIZE) >> 16;

	if (stride == 0 || height == 0 || height > TEST_HEIGHT_MAX || fb_phys != DISPLAY_RESERVED_PHYS) {
		say("apollo flip: the boot framebuffer is at %#llx, not at the start of the display's reservation: no buffers\n", fb_phys);
		return;
	}
	flip_bytes = round_page((size_t)stride * height);
	/*
	 * apollo_gpu_present=1: the GPU draws the frames too (the window server scales into them with Mesa). A render target's
	 * pitch is a multiple of 64 bytes (Mesa's fd_resource_from_handle() refuses any other; RB_BLIT_DST_PITCH and RB_2D_DST_PITCH
	 * of a6xx.xml are in units of 64 bytes): 1080 * 4 = 4320 is 67.5 of them, so a frame the GPU has drawn is flipped to with
	 * stride 4352, and each of the three frames gets the room for that (10444800 bytes, three of them 31.3 MB of the 37.7 MB).
	 * The CPU's frames (the boot stride) go on being drawn in the same places, and every flip says which stride it is.
	 */
	{
		uint32_t gpu_present = 0;

		PE_parse_boot_argn("apollo_gpu_present", &gpu_present, sizeof(gpu_present));
		if (gpu_present == 1) {
			uint32_t gpu_stride = (stride + 63) & ~63u;

			if (3 * (uint64_t)round_page((size_t)gpu_stride * height) <= DISPLAY_RESERVED_SIZE) {
				flip_gpu_stride = gpu_stride;
				flip_bytes = round_page((size_t)gpu_stride * height);
			} else {
				say("apollo flip: three frames of the GPU's stride %u do not fit the display's reservation\n", gpu_stride);
			}
		}
		flip_height = height;
	}
	if (3 * (uint64_t)flip_bytes > DISPLAY_RESERVED_SIZE) {
		say("apollo flip: three frames of %lu bytes do not fit the display's reservation\n", (unsigned long)flip_bytes);
		return;
	}
	for (uint32_t i = 0; i < 2; i++) {
		flip_phys[i] = DISPLAY_RESERVED_PHYS + (uint64_t)(i + 1) * flip_bytes;
		flip_buf[i] = ml_io_map_unmappable((vm_offset_t)flip_phys[i], flip_bytes, VM_WIMG_WCOMB);
		if (flip_buf[i] == 0) {
			say("apollo flip: buffer %u at %#llx cannot be mapped\n", i, flip_phys[i]);
			return;
		}
	}
	flip_reserved = true;
	say("apollo flip: two buffers of %lu bytes in the display's reservation, at %#llx and %#llx\n", (unsigned long)flip_bytes,
	    flip_phys[0], flip_phys[1]);
}

static void
flip_test_thread(void *arg, wait_result_t wr_)
{
	vm_offset_t buf[2] = { 0, 0 };
	uint64_t phys[2] = { 0, 0 };
	vm_offset_t b;
	uint32_t stride, height, pattern, colours[2], seconds = flip_test_seconds, orig_stride;
	size_t bytes = 0;
	struct flip_stats st;
	uint32_t i;

	(void)arg;
	(void)wr_;
	delay_for_interval(TEST_DELAY_MS, NSEC_PER_MSEC);
	if (!flip_ready()) {
		say("apollo flip test: not run: no pipe on the framebuffer, no CTL, or the display's SMMU stream translates\n");
		goto out;
	}
	b = mdp + sspp_table[sspp_found].offset;
	orig_stride = stride = rd(b + SSPP_SRC_YSTRIDE0) & 0xffff;
	height = rd(b + SSPP_SRC_SIZE) >> 16;
	pattern = rd(b + SSPP_SRC_UNPACK_PATTERN);
	if (stride == 0 || height == 0 || height > TEST_HEIGHT_MAX || fb_phys == 0) {
		say("apollo flip test: not run: the pipe's stride %u / height %u are odd\n", stride, height);
		goto out;
	}
	bytes = round_page((size_t)stride * height);
	/* in one piece and below 4 GB (the pipe's address is 32 bits) */
	for (i = 0; i < 2; i++) {
		if (flip_buf[i] == 0 || flip_bytes < bytes) {
			say("apollo flip test: not run: buffer %u was not taken at boot\n", i);
			goto out;
		}
		buf[i] = flip_buf[i];
		phys[i] = flip_phys[i];
		if (phys[i] + bytes > (1ull << 32) || (phys[i] & 0xff)) {
			say("apollo flip test: not run: buffer %u at %#llx is out of the pipe's reach\n", i, phys[i]);
			goto out;
		}
	}
	colours[0] = pixel(pattern, 2);         /* red */
	colours[1] = pixel(pattern, 0);         /* green */
	fill_buffer(buf[0], bytes, colours[0]);
	fill_buffer(buf[1], bytes, colours[1]);
	say("apollo flip test: red %#x at %#llx, green %#x at %#llx (%lu bytes each, stride %u, unpack %#x); the panel is about to "
	    "flash red and green for %u s\n", colours[0], phys[0], colours[1], phys[1], (unsigned long)bytes, stride, pattern, seconds);
	delay_for_interval(TEST_HOLD_MS, NSEC_PER_MSEC);

	/* half the time: once a second */
	stats_clear(&st);
	say("apollo flip test: once a second for %u s, first flip now\n", seconds / 2);
	for (i = 0; i < seconds / 2; i++) {
		if (!timed_flip(&st, phys[i & 1], stride)) {
			say("apollo flip test: flip %u did not latch within %u ms: aborted\n", i, FLIP_TIMEOUT_US / 1000);
			stats_log("paced", &st);
			goto restore;
		}
		delay_for_interval(1000, NSEC_PER_MSEC);
	}
	stats_log("paced", &st);

	/* the other half: one flip per frame, each after the latch of the one before */
	say("apollo flip test: one flip per frame for %u s\n", seconds - seconds / 2);
	for (uint32_t s = 0; s < seconds - seconds / 2; s++) {
		uint64_t t0 = mach_absolute_time(), now, ns;
		uint32_t flip = 0;

		stats_clear(&st);
		for (;;) {
			if (!timed_flip(&st, phys[flip++ & 1], stride)) {
				say("apollo flip test: a flip did not latch within %u ms: aborted\n", FLIP_TIMEOUT_US / 1000);
				stats_log("per frame", &st);
				goto restore;
			}
			now = mach_absolute_time();
			absolutetime_to_nanoseconds(now - t0, &ns);
			if (ns >= 1000000000ull) {
				break;
			}
		}
		stats_log("per frame", &st);
	}
restore:
	restore_original(orig_stride);
out:
	for (i = 0; i < 2; i++) {
		if (buf[i] != 0 && !flip_reserved) {
			kmem_free(kernel_map, buf[i], bytes);
		}
	}
	thread_terminate(current_thread());
}

/* ---- the frames of the triple buffering (apollo_flip=1) -------------------------------------------------- */

/*
 * MiniIOMobileFramebuffer draws into one of three frames and flips the pipe to it: frame 0 is the boot framebuffer (fb_phys;
 * the caller already has it mapped, as fBase, and passes that address in va[0] of apollo_scanout_get), frames 1 and 2 are
 * flip_buf[0..1] (the display reservation behind it, mapped by flip_reserve). One thread (the presenter) calls these.
 * scan_front: the frame the panel scans, as far as we have seen it latch; scan_pending: the one flipped to but not yet seen
 * latched, or SCAN_NONE.
 */
#define SCAN_NONE       0xffffffffu
static unsigned scan_front;
static unsigned scan_pending = SCAN_NONE;

/*
 * The state machine of the three frames is here, behind one lock: the presenter (MiniIOMobileFramebuffer's show()) and the
 * window server's calls (ApolloGPU's kApolloScanout, apollo_gpu_present=1) both move it. A frame is FREE when it is neither
 * scan_front nor scan_pending nor scan_reserved: reserved is a frame that someone is drawing into -- the presenter's CPU
 * copy (apollo_scanout_take) or the GPU, for the window server (apollo_scanout_acquire) -- from the moment it is handed out
 * until the flip to it (apollo_scanout_flip) or its release. The flip to a frame makes it scan_pending and then scan_front;
 * the one that was in front before it is free again once the new one has latched.
 */
static LCK_GRP_DECLARE(scan_lck_grp, "apollo-scanout");
static LCK_MTX_DECLARE(scan_lock, &scan_lck_grp);
static uint32_t scan_reserved;          /* a bit per frame */
static int scan_mark = -1;              /* a frame the GPU has drawn the next swap into (apollo_scanout_mark), or -1 */
static uint32_t scan_mark_swap;         /* ... and the number of that swap (bit 31 set: the ID of the swap's layer 0 surface) */

/* the frames: 3 and their addresses (va[0] in: frame 0's address, as the caller maps it; count 0: flips are not available) */
int
apollo_scanout_get(unsigned *count, vm_offset_t va[3], uint64_t pa[3], uint32_t *stride)
{
	*count = 0;
	if (!flip_ready() || !flip_reserved || va[0] == 0) {
		return 0;
	}
	va[1] = flip_buf[0];
	va[2] = flip_buf[1];
	pa[0] = fb_phys;
	pa[1] = flip_phys[0];
	pa[2] = flip_phys[1];
	*stride = boot_stride;
	lck_mtx_lock(&scan_lock);
	scan_front = 0;
	scan_pending = SCAN_NONE;
	lck_mtx_unlock(&scan_lock);
	*count = 3;
	return 3;
}

/* with the lock held */
static bool
scan_latched(void)
{
	if (scan_pending == SCAN_NONE) {
		return true;
	}
	if (!apollo_display_flip_latched()) {
		return false;
	}
	scan_front = scan_pending;
	scan_pending = SCAN_NONE;
	return true;
}

bool
apollo_scanout_latched(void)
{
	bool latched;

	lck_mtx_lock(&scan_lock);
	latched = scan_latched();
	lck_mtx_unlock(&scan_lock);
	return latched;
}

/* with the lock held: a frame that is not scanned, not waiting for its latch and nobody's; -1: none */
static int
scan_free(void)
{
	scan_latched();
	for (unsigned i = 0; i < 3; i++) {
		if (i != scan_front && i != scan_pending && !(scan_reserved & (1u << i))) {
			return (int)i;
		}
	}
	return -1;
}

/*
 * A frame for the presenter to draw into (the CPU's copy), reserved until the flip to it; -1: none is free now (the one
 * flipped to last has not latched, and the window server holds the third).
 */
int
apollo_scanout_take(void)
{
	int index;

	lck_mtx_lock(&scan_lock);
	index = scan_free();
	if (index >= 0) {
		scan_reserved |= 1u << index;
	}
	lck_mtx_unlock(&scan_lock);
	return index;
}

/* a reserved frame given back without a flip (the window server abandoned it, or the presenter could not flip) */
void
apollo_scanout_release(unsigned index)
{
	if (index > 2) {
		return;
	}
	lck_mtx_lock(&scan_lock);
	scan_reserved &= ~(1u << index);
	if (scan_mark == (int)index) {
		scan_mark = -1;
	}
	lck_mtx_unlock(&scan_lock);
}

/*
 * Frame `index` as the GPU's target (apollo_gpu_present=1, ApolloGPU's kApolloScanout): where it is, its stride (a multiple
 * of 64: see flip_reserve()), its size, and, if it has one, its address in the kernel (frame 0's is the loader's, which only
 * MiniIOMobileFramebuffer knows: 0). False: the GPU does not draw the frames (no boot-arg, no flips).
 */
bool
apollo_scanout_gpu_info(unsigned index, uint64_t *phys, uint32_t *stride, uint32_t *width, uint32_t *height, uint64_t *bytes,
    vm_offset_t *va)
{
	if (index > 2 || flip_gpu_stride == 0 || !flip_reserved || !flip_ready()) {
		return false;
	}
	*phys = index == 0 ? fb_phys : flip_phys[index - 1];
	*stride = flip_gpu_stride;
	*width = boot_stride / 4;
	*height = flip_height;
	*bytes = flip_bytes;
	*va = index == 0 ? 0 : flip_buf[index - 1];
	return true;
}

/*
 * A free frame for the window server to have the GPU draw into, reserved for it (apollo_scanout_release gives it back, the
 * flip to it ends the reservation); waits up to timeout_ms for one -- the swaps that are ahead of this frame free them -- and
 * returns -1 if there is none then. Not for the presenter's thread (it is the one that frees them).
 */
int
apollo_scanout_acquire(uint32_t timeout_ms)
{
	uint64_t began = mach_absolute_time(), ns;

	for (;;) {
		int index;

		lck_mtx_lock(&scan_lock);
		index = flip_gpu_stride != 0 && flip_reserved ? scan_free() : -1;
		if (index >= 0) {
			scan_reserved |= 1u << index;
		}
		lck_mtx_unlock(&scan_lock);
		if (index >= 0) {
			return index;
		}
		absolutetime_to_nanoseconds(mach_absolute_time() - began, &ns);
		if (flip_gpu_stride == 0 || !flip_reserved || ns / NSEC_PER_MSEC >= timeout_ms) {
			return -1;
		}
		delay_for_interval(500, NSEC_PER_USEC);
	}
}

/* the next SwapEnd with number `swap` is of a frame the GPU has drawn into `index` (taken by apollo_scanout_take_mark) */
void
apollo_scanout_mark(unsigned index, uint32_t swap)
{
	if (index > 2) {
		return;
	}
	lck_mtx_lock(&scan_lock);
	if (scan_mark >= 0 && scan_mark != (int)index) {
		scan_reserved &= ~(1u << scan_mark);    /* (a frame marked and never swapped) */
	}
	scan_mark = (int)index;
	scan_mark_swap = swap;
	lck_mtx_unlock(&scan_lock);
}

/* present(): the frame that swap `swap` was drawn into by the GPU, or -1; the reservation goes on until the flip or the release */
int
apollo_scanout_take_mark(uint32_t swap, uint32_t surface)
{
	int index;

	lck_mtx_lock(&scan_lock);
	index = scan_mark;
	scan_mark = -1;
	/* (the mark is of a swap number, or, with bit 31, of the surface that is the swap's layer 0: userland/src/eagl.c, scan_frame()) */
	if (index >= 0 && scan_mark_swap != swap &&
	    !((scan_mark_swap & 0x80000000u) != 0 && surface != 0 && (scan_mark_swap & 0x7fffffffu) == surface)) {
		scan_reserved &= ~(1u << index);        /* (some other swap: the mark is stale) */
		index = -1;
	}
	lck_mtx_unlock(&scan_lock);
	return index;
}

/*
 * flip to frame `index`, which was drawn by the CPU (gpu false: the boot stride) or by the GPU (the stride of its rows);
 * refused while the one before has not latched (then nothing is written, and the frame is still reserved)
 */
bool
apollo_scanout_flip(unsigned index, bool gpu)
{
	bool flipped = false;

	lck_mtx_lock(&scan_lock);
	if (index <= 2 && index != scan_front && scan_latched() && (!gpu || flip_gpu_stride != 0) &&
	    apollo_display_flip(index == 0 ? fb_phys : flip_phys[index - 1], gpu ? flip_gpu_stride : boot_stride)) {
		scan_pending = index;
		scan_reserved &= ~(1u << index);
		flipped = true;
	}
	lck_mtx_unlock(&scan_lock);
	return flipped;
}

/* sleep (1 ms steps, then a short spin for the last 2 ms) until the pending flip has latched, or timeout_us has passed */
void
apollo_scanout_wait(uint32_t timeout_us)
{
	uint64_t t0 = mach_absolute_time(), ns;

	while (!apollo_scanout_latched()) {
		absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
		if (ns / 1000 >= timeout_us) {
			return;
		}
		if (timeout_us - ns / 1000 > 2000) {
			delay_for_interval(1, NSEC_PER_MSEC);
		}
	}
}

unsigned
apollo_scanout_front(void)
{
	return scan_front;
}

/* ---- start ------------------------------------------------------------------------------------------- */

void
apollo_display_init(void)
{
	uint32_t enable = 0, hz = 0, dump = 0, test = 0, flip_mode = 0;
	vm_offset_t regs;

	if (!PE_parse_boot_argn("apollo_display", &enable, sizeof(enable)) || enable != 1) {
		return;
	}
	regs = ml_io_map(INTF_PHYS, INTF_SIZE);
	if (rd(regs + INTF_FRAME_LINE_EN) != 3) {
		wr(regs + INTF_FRAME_LINE_EN, 3);
	}
	intf = regs;
	read_timing();
	log_timing();
	if (hsync_period == 0 || frame_lines == 0) {
		say("apollo display: the line period reads 0: INTF_1 is not set up, left alone\n");
		intf = 0;
		return;
	}
	if (!(rd(intf + INTF_STATUS) & INTF_STATUS_RUNNING)) {
		say("apollo display: the timing engine is not running\n");
	}
	measured_mhz = measure();
	log_rate();
	find_ctl();
	/* before any change of the rate: the DPU as the UEFI left it */
	PE_parse_boot_argn("apollo_dpu_dump", &dump, sizeof(dump));
	PE_parse_boot_argn("apollo_flip_test", &test, sizeof(test));
	PE_parse_boot_argn("apollo_flip", &flip_mode, sizeof(flip_mode));
	if (dump == 1 || test != 0 || flip_mode == 1) {
		dpu_setup(dump == 1);
	}
	if ((test != 0 || flip_mode == 1) && flip_ready()) {
		flip_reserve();
	}
	if (test != 0 && test <= 600) {
		thread_t thread;

		flip_test_seconds = test < 2 ? 2 : test;
		if (kernel_thread_start(flip_test_thread, NULL, &thread) == KERN_SUCCESS) {
			thread_set_thread_name(thread, "apollo flip test");
			thread_deallocate(thread);
			say("apollo flip test: %u s of flips will start %u s from now\n", flip_test_seconds, TEST_DELAY_MS / 1000);
		}
	} else if (test != 0) {
		say("apollo display: apollo_flip_test=%u is not between 1 and 600: ignored\n", test);
	}
	if (PE_parse_boot_argn("apollo_display_hz", &hz, sizeof(hz))) {
		if (hz < 60 || hz > 144) {
			say("apollo display: apollo_display_hz=%u is not between 60 and 144: ignored\n", hz);
		} else {
			set_rate(hz);
		}
	}
}
