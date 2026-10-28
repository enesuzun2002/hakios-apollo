/*
 * The hitch tracer of the Xiaomi Mi 10T (apollo), with boot-args apollo_frames=1 and apollo_hitch=1 (without them nothing is
 * started, and the hooks below return at once). Instrumentation only, no behaviour change.
 *
 * apollo_frames=1 (bsd/dev/hakios.apollo_frames.c) showed that the animations hitch with single gaps of 65 to 235 ms between
 * two frames, and that those gaps are already there when CoreAnimation hands the frames to the kernel (its "swaps in N (CA
 * arrival)" line), not in the display's path nor, on the whole, in the GPU's. What iOS -- or our own kernel threads -- were
 * doing in such a gap is what this looks for:
 *
 *   - The sampler: every apollo_hitch_us (2000) microseconds, in the timer interrupt of one little core (apollo_hitch_cpu,
 *     0; osfmk/arm64/hakios.apollo_hitch_cpu.c's apollo_hitch_timer), it notes for each of the 8 processors what runs there:
 *     idle, or a thread -- by its process's name (p_comm), or for one of the kernel's own threads by the name
 *     thread_set_thread_name gave it ("apollo flip", "apollo buttons", the Wi-Fi's "wifi-irq" ...; "kernel_task" if none) --
 *     and the GPU's state: how many submissions are in flight (hakios.apollo_adreno.c's apollo_gpu_in_flight) and how many
 *     threads wait for one in ApolloGPU's gpuWait (hakios.apollo_gpu.cpp's apollo_gpu_waiters). A ring keeps the last
 *     RING samples (2 s at 2 ms: a gap is looked at within milliseconds of its end, so the 600 ms that matter are there).
 *     Being an interrupt, it is never late for want of being scheduled, and on its own core it sees the thread it interrupted.
 *     How the other cores' threads are read without locks, and why that is safe, is in hakios.apollo_hitch_cpu.c.
 *   - Detection: apollo_frames_swap() (the CA arrival) hands every interval between two swaps longer than apollo_hitch_ms
 *     (50) ms -- and under 500 ms: a pause with nothing moving is not a hitch -- to apollo_hitch_swap(), which queues it for
 *     the logger thread ("apollo hitch"); that one takes the samples of the interval out of the ring and says, in three
 *     lines, the gap, the processes busiest over all processors, the GPU's state, and per processor its three most seen.
 *     At most apollo_hitch_rate (10) gaps a minute are told; the others are counted, and their number said with the next.
 *   - Steady CPU use: apollo_frames' 2-second windows (those in which frames were presented) are added up, and every
 *     apollo_hitch_windows (5) windows one line says which processes used the processors in them, and what the sampler
 *     itself costs.
 *
 * The cost when on: a tick reads, for each processor, the processor's active thread (a plain load) and its id (one
 * ml_nofault_copy of 8 bytes: a page translation and a copy, well under a microsecond); only when a processor runs another
 * thread than at the previous tick is the name looked up again (four more such copies and a hash). Some 2 to 10 us a tick,
 * i.e. 0.1 to 0.5% of the sampler's core; the line about steady use says what it really was.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>         /* assert_wait, thread_block, thread_wakeup */
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.apollo_input.h>
#include <sys/proc_internal.h>       /* struct proc's p_comm: only its offset is used */
#include <sys/systm.h>
#include <sys/user.h>                /* struct uthread's pth_name: only its offset is used */

/* into the loader's log as well (as the other apollo drivers) */
extern void loader_log_putc(char c);

/* osfmk/arm64/hakios.apollo_hitch_cpu.c */
extern int apollo_hitch_cpu(unsigned cpu, uint64_t *tid, uintptr_t *thread);
extern bool apollo_hitch_thread(uintptr_t thread, uintptr_t *proc, uintptr_t *uthread, bool *kernel);
extern size_t apollo_hitch_peek(uintptr_t from, void *to, size_t size);
extern bool apollo_hitch_timer(unsigned cpu, uint64_t interval_ns, void (*tick)(uint64_t now));
/* the threads in ApolloGPU's gpuWait, read without its lock (hakios.apollo_gpu.cpp; apollo_gpu_in_flight: the header) */
extern uint32_t apollo_gpu_waiters(void);
extern kern_return_t thread_terminate(thread_t thread);

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

#define CPUS            8
#define RING            1024            /* samples kept: a power of two */
#define NAMES           512             /* the names told apart (ids); more fall into ID_OTHER */
#define NAME_CHARS      32
#define HASH_SLOTS      1024            /* a power of two, twice NAMES */
#define GAP_MAX_NS      (500 * NSEC_PER_MSEC)
#define QUEUE           8               /* gaps waiting for the logger */

/* the ids that are not a thread's */
#define ID_OFF          0               /* the processor is off line (or not there) */
#define ID_IDLE         1               /* its idle thread */
#define ID_OTHER        2               /* a name that did not fit in the table */
#define ID_GONE         3               /* a thread that was gone before its name could be read */
#define ID_FIRST        4

struct sample {
	uint64_t at;                    /* mach_absolute_time of the tick */
	uint16_t who[CPUS];             /* a name's id for each processor */
	uint8_t gpu_flight;             /* submissions given to the GPU and not done */
	uint8_t gpu_wait;               /* threads in gpuWait */
};

static bool hitch_on;
static uint32_t hitch_us = 2000, hitch_ms = 50, hitch_cpu, hitch_windows = 5, hitch_rate = 10, hitch_threads;

/* the sampler's (the timer interrupt of one processor: one writer, no lock) */
static struct sample ring[RING];
static uint32_t ring_head;              /* samples written; the one in the making is ring[ring_head % RING] */
static char names[NAMES][NAME_CHARS];
static uint32_t name_count = ID_FIRST;  /* published with a release store once the name is in names[] */
static uint16_t name_hash[HASH_SLOTS];
static uint32_t name_samples[NAMES];    /* for each id, the processor-samples it was seen in (all processors, all time) */
static uint64_t cache_tid[CPUS];        /* the thread each processor ran at the previous tick ... */
static uint16_t cache_id[CPUS];         /* ... and its name's id */
static uint64_t tick_count, tick_abs;   /* the ticks and the time they took */

/* the gaps queued for the logger, and the windows' report (under hitch_lock) */
static LCK_GRP_DECLARE(apollo_hitch_grp, "apollo-hitch");
static LCK_SPIN_DECLARE(hitch_lock, &apollo_hitch_grp);
static struct {
	uint64_t from, to;
	uint32_t skipped;               /* gaps not told since the one before */
} queue[QUEUE];
static unsigned queue_head, queue_tail;
static uint64_t minute_abs;
static uint32_t minute_told, skipped;
static char hitch_event;                /* the logger sleeps on it */

/* the frame windows (the presenter thread only, which calls apollo_hitch_window), and the report handed to the logger */
static bool win_begun;
static uint32_t win_base[NAMES], win_base_ticks;
static uint64_t win_base_tick_abs, win_ns;
static uint32_t win_acc[NAMES], win_acc_ticks, win_count;
static uint64_t win_acc_tick_abs, win_acc_tick_n;
static uint32_t report[NAMES], report_ticks, report_windows;
static uint64_t report_ns, report_tick_abs, report_tick_n;
static bool report_ready;

/* the logger's own */
static struct sample frozen[RING];
static uint16_t counts[CPUS][NAMES];
static uint32_t totals[NAMES], these[NAMES];

static inline uint64_t
ns_of(uint64_t abs)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(abs, &ns);
	return ns;
}

/* -------------------------------------------------------------------------------------------------------------------- */
/* the sampler, in the timer interrupt */

/* a name's id: looked up by its hash, added if new (only the sampler adds: no lock) */
static uint16_t
name_id(const char *name)
{
	uint32_t hash = 2166136261u, slot;

	for (const char *c = name; *c != 0; c++) {
		hash = (hash ^ (uint8_t)*c) * 16777619u;
	}
	for (unsigned probe = 0; probe < HASH_SLOTS; probe++) {
		uint16_t id;

		slot = (hash + probe) & (HASH_SLOTS - 1);
		id = name_hash[slot];
		if (id == 0) {
			if (name_count >= NAMES) {
				return ID_OTHER;
			}
			id = (uint16_t)name_count;
			strlcpy(names[id], name, NAME_CHARS);
			name_hash[slot] = id;
			__atomic_store_n(&name_count, name_count + 1, __ATOMIC_RELEASE);
			return id;
		}
		if (strncmp(names[id], name, NAME_CHARS) == 0) {
			return id;
		}
	}
	return ID_OTHER;
}

/* a thread's name, through its uthread's pth_name (thread_set_thread_name, pthread_setname_np); empty if it has none */
static void
thread_name(uintptr_t uthread, char *name, size_t size)
{
	uintptr_t text = 0;

	name[0] = 0;
	if (apollo_hitch_peek(uthread + offsetof(struct uthread, pth_name), &text, sizeof(text)) == sizeof(text) && text != 0 &&
	    apollo_hitch_peek(text, name, size - 1) == size - 1) {
		name[size - 1] = 0;
	} else {
		name[0] = 0;
	}
}

/*
 * The name a running thread is counted under: a kernel thread's own name ("kernel_task" if it has none); else its process's
 * p_comm, and with apollo_hitch_threads=1 the thread's name after a slash (backboardd's render server apart from the rest).
 */
static uint16_t
resolve(uintptr_t thread)
{
	char key[NAME_CHARS], tname[NAME_CHARS];
	uintptr_t proc = 0, uthread = 0;
	bool kernel = false;

	if (!apollo_hitch_thread(thread, &proc, &uthread, &kernel)) {
		return ID_GONE;
	}
	if (kernel) {
		thread_name(uthread, key, sizeof(key));
		if (key[0] == 0) {
			strlcpy(key, "kernel_task", sizeof(key));
		}
		return name_id(key);
	}
	if (proc == 0 || apollo_hitch_peek(proc + offsetof(struct proc, p_comm), key, MAXCOMLEN + 1) != MAXCOMLEN + 1) {
		return ID_GONE;
	}
	key[MAXCOMLEN] = 0;
	if (key[0] == 0) {
		return ID_GONE;
	}
	if (hitch_threads) {
		thread_name(uthread, tname, sizeof(tname));
		if (tname[0] != 0) {
			strlcat(key, "/", sizeof(key));
			strlcat(key, tname, sizeof(key));
		}
	}
	return name_id(key);
}

static void
hitch_tick(uint64_t now)
{
	uint32_t head = ring_head;
	struct sample *s = &ring[head % RING];

	for (unsigned cpu = 0; cpu < CPUS; cpu++) {
		uint64_t tid = 0;
		uintptr_t thread = 0;
		uint16_t id;

		switch (apollo_hitch_cpu(cpu, &tid, &thread)) {
		case 0:
			id = ID_IDLE;
			break;
		case 1:
			if (tid != 0 && tid == cache_tid[cpu]) {
				id = cache_id[cpu];
			} else {
				id = resolve(thread);
				cache_tid[cpu] = id == ID_GONE ? 0 : tid;
				cache_id[cpu] = id;
			}
			break;
		default:
			id = ID_OFF;
			break;
		}
		s->who[cpu] = id;
		name_samples[id]++;
	}
	{
		uint32_t flight = apollo_gpu_in_flight(), waiters = apollo_gpu_waiters();

		s->gpu_flight = (uint8_t)(flight > 255 ? 255 : flight);
		s->gpu_wait = (uint8_t)(waiters > 255 ? 255 : waiters);
	}
	s->at = now;
	__atomic_store_n(&ring_head, head + 1, __ATOMIC_RELEASE);
	tick_count++;
	tick_abs += mach_absolute_time() - now;
}

/* -------------------------------------------------------------------------------------------------------------------- */
/* the hooks, from hakios.apollo_frames.c */

/* apollo_frames_swap(): two swaps of CoreAnimation's, at last_abs and now_abs */
void
apollo_hitch_swap(uint64_t last_abs, uint64_t now_abs)
{
	uint64_t gap;
	bool wake = false;

	if (!hitch_on || last_abs == 0) {
		return;
	}
	gap = ns_of(now_abs - last_abs);
	if (gap <= (uint64_t)hitch_ms * NSEC_PER_MSEC || gap >= GAP_MAX_NS) {
		return;
	}
	lck_spin_lock(&hitch_lock);
	if (minute_abs == 0 || ns_of(now_abs - minute_abs) >= 60 * NSEC_PER_SEC) {
		minute_abs = now_abs;
		minute_told = 0;
	}
	if (minute_told >= hitch_rate || queue_head - queue_tail >= QUEUE) {
		skipped++;
	} else {
		minute_told++;
		queue[queue_head % QUEUE].from = last_abs;
		queue[queue_head % QUEUE].to = now_abs;
		queue[queue_head % QUEUE].skipped = skipped;
		skipped = 0;
		queue_head++;
		wake = true;
	}
	lck_spin_unlock(&hitch_lock);
	if (wake) {
		thread_wakeup((event_t)&hitch_event);
	}
}

/* apollo_frames_shown(): a 2-second window of presented frames begins (begin) or has been told (!begin) */
void
apollo_hitch_window(bool begin, uint64_t window_ns)
{
	uint32_t head;

	if (!hitch_on) {
		return;
	}
	head = __atomic_load_n(&ring_head, __ATOMIC_ACQUIRE);
	if (begin) {
		memcpy(win_base, name_samples, sizeof(win_base));
		win_base_ticks = head;
		win_base_tick_abs = tick_abs;
		win_begun = true;
		return;
	}
	if (!win_begun) {
		return;                 /* (the tracer started in the middle of this window) */
	}
	win_begun = false;
	for (unsigned id = 0; id < NAMES; id++) {
		win_acc[id] += name_samples[id] - win_base[id];
	}
	win_acc_ticks += head - win_base_ticks;
	win_acc_tick_abs += tick_abs - win_base_tick_abs;
	win_acc_tick_n += head - win_base_ticks;
	win_ns += window_ns;
	if (++win_count < hitch_windows) {
		return;
	}
	lck_spin_lock(&hitch_lock);
	if (!report_ready) {
		memcpy(report, win_acc, sizeof(report));
		report_ticks = win_acc_ticks;
		report_windows = win_count;
		report_ns = win_ns;
		report_tick_abs = win_acc_tick_abs;
		report_tick_n = win_acc_tick_n;
		report_ready = true;
	}
	lck_spin_unlock(&hitch_lock);
	memset(win_acc, 0, sizeof(win_acc));
	win_acc_ticks = win_count = 0;
	win_acc_tick_abs = win_acc_tick_n = 0;
	win_ns = 0;
	thread_wakeup((event_t)&hitch_event);
}

/* -------------------------------------------------------------------------------------------------------------------- */
/* the logger */

static const char *
name_of(uint16_t id)
{
	switch (id) {
	case ID_OFF:
		return "off";
	case ID_IDLE:
		return "idle";
	case ID_OTHER:
		return "(other)";
	case ID_GONE:
		return "(gone)";
	default:
		return id < __atomic_load_n(&name_count, __ATOMIC_ACQUIRE) ? names[id] : "?";
	}
}

/* "a.b%" of part in whole */
#define PCT(part, whole)        (uint32_t)((uint64_t)(part) * 100 / (whole)), (uint32_t)((uint64_t)(part) * 1000 / (whole) % 10)

/* the ids of the `want` largest of values[] (skipping idle and off if busy_only), largest first; returns how many */
static unsigned
top_of(const uint32_t *values, unsigned want, bool busy_only, uint16_t *ids)
{
	unsigned found = 0;

	for (uint16_t id = 0; id < NAMES; id++) {
		unsigned at;

		if (values[id] == 0 || (busy_only && (id == ID_IDLE || id == ID_OFF))) {
			continue;
		}
		for (at = found; at > 0 && values[ids[at - 1]] < values[id]; at--) {
			if (at < want) {
				ids[at] = ids[at - 1];
			}
		}
		if (at < want) {
			ids[at] = id;
			if (found < want) {
				found++;
			}
		}
	}
	return found;
}

static void
tell_hitch(uint64_t from, uint64_t to, uint32_t not_told)
{
	uint32_t head = __atomic_load_n(&ring_head, __ATOMIC_ACQUIRE), n = 0, flight_sum = 0, flight_max = 0, waiting = 0;
	uint64_t gap = ns_of(to - from), at = ns_of(to);
	char line[400], untold[48] = "";
	size_t used;
	uint16_t top[8];
	unsigned found;

	/* the samples of the gap, oldest first, out of the ring before the sampler writes over them */
	for (uint32_t back = head > RING ? RING : head; back > 0; back--) {
		const struct sample *s = &ring[(head - back) % RING];

		if (s->at >= from && s->at <= to && n < RING) {
			frozen[n++] = *s;
		}
	}
	if (n == 0) {
		say("apollo hitch: %llu ms without a frame (at %llu.%03llu s): no samples of it\n", gap / NSEC_PER_MSEC,
		    at / NSEC_PER_SEC, at / NSEC_PER_MSEC % 1000);
		return;
	}
	memset(counts, 0, sizeof(counts));
	memset(totals, 0, sizeof(totals));
	for (uint32_t i = 0; i < n; i++) {
		for (unsigned cpu = 0; cpu < CPUS; cpu++) {
			uint16_t id = frozen[i].who[cpu];

			counts[cpu][id]++;
			totals[id]++;
		}
		flight_sum += frozen[i].gpu_flight;
		flight_max = frozen[i].gpu_flight > flight_max ? frozen[i].gpu_flight : flight_max;
		waiting += frozen[i].gpu_wait != 0;
	}

	/* line 1: the gap, the busiest over all processors (of all processor-samples), the GPU */
	used = 0;
	found = top_of(totals, 6, true, top);
	for (unsigned k = 0; k < found && used < sizeof(line); k++) {
		used += (size_t)snprintf(line + used, sizeof(line) - used, "%s%s %u.%u%%", k ? ", " : "", name_of(top[k]),
		    PCT(totals[top[k]], n * CPUS));
	}
	if (not_told != 0) {
		snprintf(untold, sizeof(untold), ", %u not told before it", not_told);
	}
	say("apollo hitch: %llu ms without a frame (at %llu.%03llu s, %u samples%s): busiest %s; idle %u.%u%%; gpu: in flight "
	    "%u.%u mean, %u max; a thread in gpuWait in %u.%u%%\n", gap / NSEC_PER_MSEC, at / NSEC_PER_SEC,
	    at / NSEC_PER_MSEC % 1000, n, untold, found ? line : "nothing", PCT(totals[ID_IDLE], n * CPUS), flight_sum / n, flight_sum * 10 / n % 10, flight_max,
	    PCT(waiting, n));

	/* lines 2 and 3: four processors each, their three most seen */
	for (unsigned half = 0; half < CPUS; half += 4) {
		used = 0;
		for (unsigned cpu = half; cpu < half + 4 && used < sizeof(line); cpu++) {
			for (unsigned id = 0; id < NAMES; id++) {
				these[id] = counts[cpu][id];
			}
			found = top_of(these, 3, false, top);
			used += (size_t)snprintf(line + used, sizeof(line) - used, "%scpu%u:", cpu > half ? "; " : "", cpu);
			for (unsigned k = 0; k < found && used < sizeof(line); k++) {
				used += (size_t)snprintf(line + used, sizeof(line) - used, "%s %s %u%%", k ? "," : "",
				    name_of(top[k]), (uint32_t)(counts[cpu][top[k]] * 100u / n));
			}
		}
		say("apollo hitch:   %s\n", line);
	}
}

static void
tell_windows(void)
{
	uint32_t total = report_ticks * CPUS, busy;
	uint64_t cost_us = ns_of(report_tick_abs) / NSEC_PER_USEC, span_us = (uint64_t)report_tick_n * hitch_us;
	uint16_t top[10];
	unsigned found;
	char line[400];
	size_t used = 0;

	if (report_ticks == 0) {
		return;
	}
	busy = total - report[ID_IDLE] - report[ID_OFF];
	found = top_of(report, 10, true, top);
	for (unsigned k = 0; k < found && used < sizeof(line); k++) {
		used += (size_t)snprintf(line + used, sizeof(line) - used, "%s%s %u.%u%%", k ? ", " : "", name_of(top[k]),
		    PCT(report[top[k]], total));
	}
	say("apollo hitch: cpu use in %u frame windows (%llu ms, %u samples, 100%% = all %u cpus): busy %u.%u%%: %s; "
	    "the sampler took %llu us a tick (%u.%u%% of cpu%u)\n", report_windows, report_ns / NSEC_PER_MSEC, report_ticks,
	    CPUS, PCT(busy, total), found ? line : "nothing", cost_us / (report_tick_n ? report_tick_n : 1),
	    PCT(cost_us, span_us ? span_us : 1), hitch_cpu);
}

static void
hitch_logger(__unused void *parameter, __unused wait_result_t result)
{
	if (!apollo_hitch_timer(hitch_cpu, (uint64_t)hitch_us * NSEC_PER_USEC, hitch_tick)) {
		say("apollo hitch: no timer on cpu%u: the tracer is off\n", hitch_cpu);
		hitch_on = false;
		thread_terminate(current_thread());
		return;
	}
	say("apollo hitch: a sample of all %u cpus every %u us (in cpu%u's timer interrupt); gaps between CA's swaps over %u ms "
	    "are told (at most %u a minute), the cpu use every %u frame windows\n", CPUS, hitch_us, hitch_cpu, hitch_ms, hitch_rate,
	    hitch_windows);
	__atomic_store_n(&hitch_on, true, __ATOMIC_RELEASE);
	for (;;) {
		uint64_t from = 0, to = 0;
		uint32_t not_told = 0;
		bool gap = false, windows = false;

		lck_spin_lock(&hitch_lock);
		if (queue_tail != queue_head) {
			from = queue[queue_tail % QUEUE].from;
			to = queue[queue_tail % QUEUE].to;
			not_told = queue[queue_tail % QUEUE].skipped;
			queue_tail++;
			gap = true;
		} else if (report_ready) {
			windows = true;
		} else {
			assert_wait((event_t)&hitch_event, THREAD_UNINT);
			lck_spin_unlock(&hitch_lock);
			thread_block(THREAD_CONTINUE_NULL);
			continue;
		}
		lck_spin_unlock(&hitch_lock);
		if (gap) {
			tell_hitch(from, to, not_told);
		}
		if (windows) {
			tell_windows();
			lck_spin_lock(&hitch_lock);
			report_ready = false;
			lck_spin_unlock(&hitch_lock);
		}
	}
}

/* from apollo_input_init (bsd_init): boot-arg apollo_hitch=1, with apollo_frames=1 (whose hooks feed it) */
void
apollo_hitch_init(void)
{
	uint32_t value = 0, frames = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("apollo_hitch", &value, sizeof(value)) || value != 1) {
		return;
	}
	if (!PE_parse_boot_argn("apollo_frames", &frames, sizeof(frames)) || frames != 1) {
		say("apollo hitch: needs apollo_frames=1 (its swaps and windows are what it looks at): off\n");
		return;
	}
	PE_parse_boot_argn("apollo_hitch_us", &hitch_us, sizeof(hitch_us));
	PE_parse_boot_argn("apollo_hitch_ms", &hitch_ms, sizeof(hitch_ms));
	PE_parse_boot_argn("apollo_hitch_cpu", &hitch_cpu, sizeof(hitch_cpu));
	PE_parse_boot_argn("apollo_hitch_windows", &hitch_windows, sizeof(hitch_windows));
	PE_parse_boot_argn("apollo_hitch_rate", &hitch_rate, sizeof(hitch_rate));
	PE_parse_boot_argn("apollo_hitch_threads", &hitch_threads, sizeof(hitch_threads));
	hitch_us = hitch_us < 500 ? 500 : hitch_us > 50000 ? 50000 : hitch_us;
	hitch_cpu = hitch_cpu >= CPUS ? 0 : hitch_cpu;
	hitch_windows = hitch_windows == 0 ? 0xffffffffu : hitch_windows;      /* 0: never */
	strlcpy(names[ID_OFF], "off", NAME_CHARS);
	if (kernel_thread_start(hitch_logger, NULL, &thread) == KERN_SUCCESS) {
		thread_set_thread_name(thread, "apollo hitch");
		thread_deallocate(thread);
	} else {
		say("apollo hitch: no thread: off\n");
	}
}
