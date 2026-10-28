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
 *     the CPU copy/scale of the layers into the panel's memory (drawLayer, plus record()). With apollo_flip=1 the flips are
 *     queued (bsd/dev/hakios.apollo_display.c, apollo_scanout_flip): "scanout-wait" is then only the call that queues one,
 *     near 0; where the waits went is in the third line.
 *   - apollo_frames_flip(): hakios.apollo_display.c's triple buffering: each flip written at once (nothing pending) or queued
 *     behind a pending one; when the queued ones were written (after the frame start: the "apollo flip" thread's promptness,
 *     a flip written before the next frame start is shown from it); pending flips written over by a newer one (latest wins);
 *     every latch, and the time from the flip's request to its latch (what a frame waits for the panel once it is drawn: under
 *     a refresh, half of one on average, is the floor; more than one means frames wait in a queue); queued frames replaced or
 *     taken back by a newer one (never shown); the presenter finding no frame.
 *   - apollo_frames_acquire(): the window server's wait for a frame to have the GPU draw into (kScanoutAcquire; eagl.c gives
 *     up after 50 ms, and the processor draws that swap): the wait the panel puts on CoreAnimation's own thread.
 * Every 2 seconds in which a frame was presented, three lines "apollo frames: ..." are logged.
 * With apollo_hitch=1 as well, the swaps and the windows feed the hitch tracer (bsd/dev/hakios.apollo_hitch.c): what the
 * processors ran in a gap between two swaps, and over the windows.
 */
#include <kern/clock.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.apollo_input.h>  /* apollo_frames_flip(), apollo_frames_acquire() and their kinds; apollo_hitch_* */
#include <sys/systm.h>

/* into the loader's log as well (as the other apollo drivers) */
extern void loader_log_putc(char c);

void apollo_frames_swap(uint64_t blocked_abs);
void apollo_frames_vsync(void);
void apollo_frames_shown(uint64_t gpu_abs, uint64_t scan_abs, uint64_t copy_abs, uint32_t on_scan);
#define FLIP_KINDS      APOLLO_FRAMES_FLIP_KINDS
#define FLIP_APPLIED    APOLLO_FRAMES_FLIP_APPLIED
#define FLIP_LATCHED    APOLLO_FRAMES_FLIP_LATCHED

static int say(const char *format, ...) __printflike(1, 2);

static int
say(const char *format, ...)
{
	char line[512];
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

/* flips (under hakios.apollo_display.c's scan_lock: one at a time; taken by the report with atomics) and the window server's waits */
static uint64_t fl_count[FLIP_KINDS], fl_sum_ns[FLIP_KINDS], fl_max_ns[FLIP_KINDS];
static uint64_t aq_count, aq_sum_ns, aq_max_ns, aq_waited, aq_failed;

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
apollo_frames_flip(unsigned what, uint64_t ns)
{
	if (!on() || what >= FLIP_KINDS) {
		return;
	}
	__atomic_fetch_add(&fl_count[what], 1, __ATOMIC_RELAXED);
	__atomic_fetch_add(&fl_sum_ns[what], ns, __ATOMIC_RELAXED);
	max_to(&fl_max_ns[what], ns);
}

void
apollo_frames_acquire(uint64_t wait_abs, bool got)
{
	uint64_t ns;

	if (!on()) {
		return;
	}
	ns = ns_of(wait_abs);
	__atomic_fetch_add(&aq_count, 1, __ATOMIC_RELAXED);
	__atomic_fetch_add(&aq_sum_ns, ns, __ATOMIC_RELAXED);
	__atomic_fetch_add(&aq_waited, ns > 500 * NSEC_PER_USEC, __ATOMIC_RELAXED);
	__atomic_fetch_add(&aq_failed, !got, __ATOMIC_RELAXED);
	max_to(&aq_max_ns, ns);
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
		apollo_hitch_swap(last, now);           /* (apollo_hitch=1: a gap over apollo_hitch_ms is looked into) */
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
		apollo_hitch_window(true, 0);
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
	{
		uint64_t c[FLIP_KINDS], sum[FLIP_KINDS], mx[FLIP_KINDS];
		uint64_t an = __atomic_exchange_n(&aq_count, 0, __ATOMIC_RELAXED);
		uint64_t asum = __atomic_exchange_n(&aq_sum_ns, 0, __ATOMIC_RELAXED);
		uint64_t amax = __atomic_exchange_n(&aq_max_ns, 0, __ATOMIC_RELAXED);
		uint64_t awaited = __atomic_exchange_n(&aq_waited, 0, __ATOMIC_RELAXED);
		uint64_t afailed = __atomic_exchange_n(&aq_failed, 0, __ATOMIC_RELAXED);

		for (unsigned k = 0; k < FLIP_KINDS; k++) {
			c[k] = __atomic_exchange_n(&fl_count[k], 0, __ATOMIC_RELAXED);
			sum[k] = __atomic_exchange_n(&fl_sum_ns[k], 0, __ATOMIC_RELAXED);
			mx[k] = __atomic_exchange_n(&fl_max_ns[k], 0, __ATOMIC_RELAXED);
		}
		say("apollo frames: flips %llu at once + %llu over a pending one + %llu queued (written %llu/%llu us after the frame "
		    "start, mean/max), %llu queued replaced and %llu taken back (not shown), %llu failed; %llu latched (%llu/s), "
		    "asked -> latched %llu/%llu us; presenter found no frame %llu times; window server's acquire %llu, waited "
		    "%llu/%llu us (mean/max), %llu over 0.5 ms, %llu none in time\n", c[APOLLO_FRAMES_FLIP_AT_ONCE],
		    c[APOLLO_FRAMES_FLIP_SUPERSEDED], c[APOLLO_FRAMES_FLIP_QUEUED],
		    sum[FLIP_APPLIED] / (c[FLIP_APPLIED] ? c[FLIP_APPLIED] : 1) / 1000, mx[FLIP_APPLIED] / 1000,
		    c[APOLLO_FRAMES_FLIP_REPLACED], c[APOLLO_FRAMES_FLIP_TAKEN_BACK], c[APOLLO_FRAMES_FLIP_FAILED],
		    c[FLIP_LATCHED], c[FLIP_LATCHED] * 1000000000ULL / window,
		    sum[FLIP_LATCHED] / (c[FLIP_LATCHED] ? c[FLIP_LATCHED] : 1) / 1000, mx[FLIP_LATCHED] / 1000,
		    c[APOLLO_FRAMES_FLIP_NONE_FREE], an, asum / (an ? an : 1) / 1000, amax / 1000, awaited, afailed);
	}
	apollo_hitch_window(false, window);
	pr_count = pr_since_abs = 0;
	pr_gap_sum_ns = pr_gap_max_ns = pr_gap_20 = pr_gap_34 = pr_gaps = 0;
	pr_gpu_ns = pr_scan_ns = pr_copy_ns = pr_total_max_ns = pr_gpu_max_ns = pr_scan_max_ns = pr_copy_max_ns = 0;
	pr_on_scan = 0;
}
