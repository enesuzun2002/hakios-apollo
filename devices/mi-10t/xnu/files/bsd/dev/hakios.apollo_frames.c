/*
 * Frame-pacing statistics of the Xiaomi Mi 10T (apollo), with boot-arg apollo_frames=1 (without it nothing is counted or
 * logged: the hooks return at once). Instrumentation only, no behaviour change.
 *
 * Where the numbers are taken (iokit/Kernel/MiniIOMobileFramebuffer.cpp):
 *   - apollo_frames_swap(): IOMobileFramebuffer::present(), the SwapEnd of CoreAnimation: when a frame arrives, and how
 *     long that call was blocked because the panel's queue was full (fb_queue).
 *   - apollo_frames_vsync(): IOMobileFramebufferUserClient::vsync(), one tick of the display link's timer thread-call
 *     (refreshHz(), 60), delivered to backboardd.
 *   - apollo_frames_shown(): the end of IOMobileFramebuffer::show(), the presenter thread's work for one frame: the wait
 *     for the GPU (surface drawn), the wait for the panel's scan to leave the picture (apollo_display_wait_scanout) and
 *     the CPU copy/scale of the layers into the panel's memory (drawLayer, plus record()).
 * Every 2 seconds in which a frame was presented, one line "apollo frames: ..." is logged.
 */
#include <kern/clock.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>

/* into the loader's log as well (as the other apollo drivers) */
extern void loader_log_putc(char c);

void apollo_frames_swap(uint64_t blocked_abs);
void apollo_frames_vsync(void);
void apollo_frames_shown(uint64_t gpu_abs, uint64_t scan_abs, uint64_t copy_abs, uint32_t on_scan);

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

#define WINDOW_NS       (2ULL * NSEC_PER_SEC)

static int enabled = -1;                        /* -1: boot-arg not read yet */

/* swaps (any thread: atomics; the maxima may race, which is fine for statistics) */
static uint64_t sw_count, sw_blocked_ns, sw_blocked_max_ns, sw_last_abs;
static uint64_t sw_gap_sum_ns, sw_gap_max_ns, sw_gap_20, sw_gap_34, sw_gaps;
static uint64_t sw_blocked_n;
static uint64_t vsync_ticks;

/* presented frames (the presenter thread only) */
static uint64_t pr_count, pr_since_abs, pr_last_abs;
static uint64_t pr_gap_sum_ns, pr_gap_max_ns, pr_gap_20, pr_gap_34, pr_gaps;
static uint64_t pr_gpu_ns, pr_scan_ns, pr_copy_ns, pr_total_max_ns, pr_gpu_max_ns, pr_scan_max_ns, pr_copy_max_ns;
static uint64_t pr_on_scan;

static bool
on(void)
{
	if (enabled < 0) {
		uint32_t value = 0;

		enabled = PE_parse_boot_argn("apollo_frames", &value, sizeof(value)) && value == 1;
		if (enabled) {
			say("apollo frames: statistics on (every 2 s in which a frame was presented)\n");
		}
	}
	return enabled > 0;
}

static inline uint64_t
ns_of(uint64_t abs)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(abs, &ns);
	return ns;
}

static inline void
max_to(uint64_t *where, uint64_t value)
{
	if (value > *where) {
		*where = value;
	}
}

void
apollo_frames_vsync(void)
{
	if (on()) {
		__atomic_fetch_add(&vsync_ticks, 1, __ATOMIC_RELAXED);
	}
}

void
apollo_frames_swap(uint64_t blocked_abs)
{
	uint64_t now, last, gap, blocked;

	if (!on()) {
		return;
	}
	now = mach_absolute_time();
	last = sw_last_abs;
	sw_last_abs = now;
	__atomic_fetch_add(&sw_count, 1, __ATOMIC_RELAXED);
	if (last != 0) {
		gap = ns_of(now - last);
		if (gap < 500 * NSEC_PER_MSEC) {        /* (a pause with nothing moving is not a late frame) */
			__atomic_fetch_add(&sw_gap_sum_ns, gap, __ATOMIC_RELAXED);
			__atomic_fetch_add(&sw_gaps, 1, __ATOMIC_RELAXED);
			__atomic_fetch_add(&sw_gap_20, gap > 20 * NSEC_PER_MSEC, __ATOMIC_RELAXED);
			__atomic_fetch_add(&sw_gap_34, gap > 34 * NSEC_PER_MSEC, __ATOMIC_RELAXED);
			max_to(&sw_gap_max_ns, gap);
		}
	}
	if (blocked_abs != 0) {
		blocked = ns_of(blocked_abs);
		__atomic_fetch_add(&sw_blocked_ns, blocked, __ATOMIC_RELAXED);
		__atomic_fetch_add(&sw_blocked_n, 1, __ATOMIC_RELAXED);
		max_to(&sw_blocked_max_ns, blocked);
	}
}

void
apollo_frames_shown(uint64_t gpu_abs, uint64_t scan_abs, uint64_t copy_abs, uint32_t on_scan)
{
	uint64_t now, gpu, scan, copy, total, window;

	if (!on()) {
		return;
	}
	now = mach_absolute_time();
	gpu = ns_of(gpu_abs);
	scan = ns_of(scan_abs);
	copy = ns_of(copy_abs);
	total = gpu + scan + copy;
	if (pr_since_abs == 0) {
		pr_since_abs = now;
	}
	pr_count++;
	pr_gpu_ns += gpu;
	pr_scan_ns += scan;
	pr_copy_ns += copy;
	pr_on_scan += on_scan;
	max_to(&pr_gpu_max_ns, gpu);
	max_to(&pr_scan_max_ns, scan);
	max_to(&pr_copy_max_ns, copy);
	max_to(&pr_total_max_ns, total);
	if (pr_last_abs != 0) {
		uint64_t gap = ns_of(now - pr_last_abs);

		if (gap < 500 * NSEC_PER_MSEC) {
			pr_gap_sum_ns += gap;
			pr_gaps++;
			pr_gap_20 += gap > 20 * NSEC_PER_MSEC;
			pr_gap_34 += gap > 34 * NSEC_PER_MSEC;
			max_to(&pr_gap_max_ns, gap);
		}
	}
	pr_last_abs = now;
	window = ns_of(now - pr_since_abs);
	if (window < WINDOW_NS) {
		return;
	}
	{
		uint64_t n = pr_count, g = pr_gaps ? pr_gaps : 1, sn = __atomic_exchange_n(&sw_count, 0, __ATOMIC_RELAXED);
		uint64_t sg = __atomic_exchange_n(&sw_gaps, 0, __ATOMIC_RELAXED);
		uint64_t sgsum = __atomic_exchange_n(&sw_gap_sum_ns, 0, __ATOMIC_RELAXED);
		uint64_t sgmax = __atomic_exchange_n(&sw_gap_max_ns, 0, __ATOMIC_RELAXED);
		uint64_t sg20 = __atomic_exchange_n(&sw_gap_20, 0, __ATOMIC_RELAXED);
		uint64_t sg34 = __atomic_exchange_n(&sw_gap_34, 0, __ATOMIC_RELAXED);
		uint64_t sbn = __atomic_exchange_n(&sw_blocked_n, 0, __ATOMIC_RELAXED);
		uint64_t sb = __atomic_exchange_n(&sw_blocked_ns, 0, __ATOMIC_RELAXED);
		uint64_t sbmax = __atomic_exchange_n(&sw_blocked_max_ns, 0, __ATOMIC_RELAXED);
		uint64_t ticks = __atomic_exchange_n(&vsync_ticks, 0, __ATOMIC_RELAXED);

		say("apollo frames: presented %llu in %llu ms = %llu.%02llu fps; interval mean %llu.%02llu max %llu.%02llu ms, "
		    ">20 ms: %llu, >34 ms: %llu; present path per frame (mean/max us): total %llu/%llu = gpu-wait %llu/%llu + "
		    "scanout-wait %llu/%llu (%llu on scan) + copy %llu/%llu; vsync ticks %llu (%llu/s)\n",
		    n, window / NSEC_PER_MSEC, n * 1000000000ULL / window, (n * 100000000000ULL / window) % 100,
		    pr_gap_sum_ns / g / NSEC_PER_MSEC, (pr_gap_sum_ns / g / 10000) % 100,
		    pr_gap_max_ns / NSEC_PER_MSEC, (pr_gap_max_ns / 10000) % 100, pr_gap_20, pr_gap_34,
		    (pr_gpu_ns + pr_scan_ns + pr_copy_ns) / n / 1000, pr_total_max_ns / 1000, pr_gpu_ns / n / 1000,
		    pr_gpu_max_ns / 1000, pr_scan_ns / n / 1000, pr_scan_max_ns / 1000, pr_on_scan, pr_copy_ns / n / 1000,
		    pr_copy_max_ns / 1000, ticks, ticks * 1000000000ULL / window);
		say("apollo frames: swaps in %llu (CA arrival) interval mean %llu.%02llu max %llu.%02llu ms, >20 ms: %llu, "
		    ">34 ms: %llu; swap blocked on a full queue %llu times, %llu us total, max %llu us\n", sn,
		    sgsum / (sg ? sg : 1) / NSEC_PER_MSEC, (sgsum / (sg ? sg : 1) / 10000) % 100, sgmax / NSEC_PER_MSEC,
		    (sgmax / 10000) % 100, sg20, sg34, sbn, sb / 1000, sbmax / 1000);
	}
	pr_count = pr_since_abs = 0;
	pr_gap_sum_ns = pr_gap_max_ns = pr_gap_20 = pr_gap_34 = pr_gaps = 0;
	pr_gpu_ns = pr_scan_ns = pr_copy_ns = pr_total_max_ns = pr_gpu_max_ns = pr_scan_max_ns = pr_copy_max_ns = 0;
	pr_on_scan = 0;
}
