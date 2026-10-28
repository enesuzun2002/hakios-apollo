/*
 * The Xiaomi Mi 10T (apollo): the scheduler's side of the hitch tracer (boot-arg apollo_hitch=1, bsd/dev/hakios.apollo_hitch.c).
 * The tracer lives with the other apollo drivers in bsd/dev, which does not see the scheduler's structures (struct processor,
 * struct thread, struct thread_ro are osfmk's own); what it needs of them is here, behind plain C prototypes:
 *
 *   - apollo_hitch_cpu(): what one processor runs right now -- nothing (off line), its idle thread, or a thread, given as
 *     the thread's address and its system-wide id (thread_tid).
 *   - apollo_hitch_thread(): for a thread found that way, its task's BSD process (struct proc, whose p_comm the tracer reads)
 *     and its uthread (whose pth_name holds the name thread_set_thread_name gave a kernel thread), and whether it is one of
 *     the kernel's own (kernel_task).
 *   - apollo_hitch_task_waiting(): the threads of a task that are runnable but on no processor (waiting for one), walked
 *     through the task's thread list with the same fault-free reads.
 *   - apollo_hitch_peek(): a read of kernel memory that never faults (ml_nofault_copy).
 *   - apollo_hitch_timer(): a timer that runs the tracer's sampler every N us on one processor, in its timer interrupt.
 *
 * Nothing here takes a lock or a reference: the sampler runs in an interrupt, every 2 ms, and looks at the other processors
 * while they run. processor_array[] and the processors themselves are never freed (zalloc_permanent, or the boot processor's
 * static one), so processor->active_thread and ->idle_thread are plain loads -- racy, which is fine: the thread may have
 * been switched out a moment later, and the sample is then a moment stale. The thread itself is NOT safe to touch: by the
 * time it is looked at it may have terminated, been freed by the terminate daemon, and the zone's page given back
 * (zone_depopulate), so a plain load could fault. Every field of a thread, of its thread_ro, of the proc and of the name is
 * therefore read with ml_nofault_copy, which translates each source page first (kvtophys: the AT instruction, else a
 * lock-free walk of the kernel's tables) and copies through the physical aperture: an unmapped page ends the copy short
 * instead of faulting. What it may return for a thread that has just been freed (zeroes: the zones are ZC_ZFREE_CLEARMEM,
 * or another thread's fields if the memory was reused) is only a wrong name in a statistic -- the reads never write.
 */
#include <arm/machine_routines.h>       /* ml_nofault_copy */
#include <kern/cpu_number.h>
#include <kern/processor.h>
#include <kern/sched_prim.h>            /* thread_bind, thread_block */
#include <kern/task.h>                  /* kernel_task */
#include <kern/thread.h>
#include <kern/timer_call.h>
#include <mach/mach_types.h>

int apollo_hitch_cpu(unsigned cpu, uint64_t *tid, uintptr_t *thread);
bool apollo_hitch_thread(uintptr_t thread, uintptr_t *proc, uintptr_t *uthread, bool *kernel, uintptr_t *task);
int apollo_hitch_task_waiting(uintptr_t task, uintptr_t *threads, uint64_t *tids, uint64_t *since_abs, int max);
bool apollo_hitch_thread_tid(uintptr_t thread, uint64_t *tid);
size_t apollo_hitch_peek(uintptr_t from, void *to, size_t size);
bool apollo_hitch_timer(unsigned cpu, uint64_t interval_ns, void (*tick)(uint64_t now));

size_t
apollo_hitch_peek(uintptr_t from, void *to, size_t size)
{
	if (from < VM_MIN_KERNEL_ADDRESS) {     /* (NULL, or a field read from a zeroed thread) */
		return 0;
	}
	return ml_nofault_copy((vm_offset_t)from, (vm_offset_t)to, size);
}

static inline bool
peek_word(const void *at, uintptr_t *value)
{
	return apollo_hitch_peek((uintptr_t)at, value, sizeof(*value)) == sizeof(*value);
}

/*
 * What processor `cpu` runs: -1 it is not there or off line, 0 its idle thread (the processor is idle), 1 a thread: *thread
 * is its address (only ever handed to apollo_hitch_thread, never dereferenced by the caller) and *tid its id. On the
 * processor that takes the sampler's timer interrupt, active_thread is the thread the interrupt came in on: what that
 * processor was doing too.
 */
int
apollo_hitch_cpu(unsigned cpu, uint64_t *tid, uintptr_t *thread)
{
	processor_t processor;
	processor_state_t state;
	thread_t active;

	if (cpu >= MAX_SCHED_CPUS || (processor = processor_array[cpu]) == PROCESSOR_NULL) {
		return -1;
	}
	state = os_atomic_load(&processor->state, relaxed);
	if (state == PROCESSOR_OFF_LINE || state == PROCESSOR_PENDING_OFFLINE) {
		return -1;
	}
	active = os_atomic_load(&processor->active_thread, relaxed);
	if (active == THREAD_NULL) {
		return -1;
	}
	if (active == processor->idle_thread) {
		return 0;
	}
	*thread = (uintptr_t)active;
	if (apollo_hitch_peek((uintptr_t)&active->thread_id, tid, sizeof(*tid)) != sizeof(*tid)) {
		return -1;                      /* (gone already) */
	}
	return 1;
}

/* the id of the thread at `thread` now (the logger's check that a thread sampled earlier is still that one) */
bool
apollo_hitch_thread_tid(uintptr_t thread, uint64_t *tid)
{
	return apollo_hitch_peek((uintptr_t)&((thread_t)thread)->thread_id, tid, sizeof(*tid)) == sizeof(*tid);
}

/*
 * The process of a thread that apollo_hitch_cpu() found (thread_ro's tro_proc: NULL for a task without one) and its uthread
 * (co-allocated right after the thread: get_bsdthread_info is arithmetic, nothing is read). kernel: its task is kernel_task;
 * *task_out: the task itself (only ever handed back to apollo_hitch_task_waiting).
 * False: the thread was gone by now.
 */
bool
apollo_hitch_thread(uintptr_t thread, uintptr_t *proc, uintptr_t *uthread, bool *kernel, uintptr_t *task_out)
{
	thread_t th = (thread_t)thread;
	uintptr_t tro, task;

	if (!peek_word(&th->t_tro, &tro) || tro == 0) {
		return false;
	}
	if (!peek_word(&((struct thread_ro *)tro)->tro_task, &task) ||
	    !peek_word(&((struct thread_ro *)tro)->tro_proc, proc)) {
		return false;
	}
	*kernel = task == (uintptr_t)kernel_task;
	*uthread = (uintptr_t)get_bsdthread_info(th);
	*task_out = task;
	return true;
}

/*
 * The threads of `task` (as apollo_hitch_thread gave it) that are runnable but on no processor: state TH_RUN without TH_WAIT
 * (TH_RUN is "running or on a run queue"), and no processor's active_thread. Up to max of them: their addresses, ids, and
 * last_made_runnable_time (when they were woken or preempted: how long they have waited for a processor). -1: the task's
 * thread list could not be read. The list (task->threads, linked through thread->task_threads) is walked without the task's
 * lock, with fault-free reads only: it may change under the walk, so the walk stops at the list's head, at a thread whose
 * thread_ro does not name this task (a link into another list, or memory reused), or after 256 threads.
 */
int
apollo_hitch_task_waiting(uintptr_t task, uintptr_t *threads, uint64_t *tids, uint64_t *since_abs, int max)
{
	uintptr_t head = (uintptr_t)&((task_t)task)->threads, link, active[MAX_SCHED_CPUS];
	unsigned cpus = 0;
	int found = 0;

	for (unsigned cpu = 0; cpu < MAX_SCHED_CPUS && cpu < 16; cpu++) {
		processor_t processor = processor_array[cpu];

		active[cpus++] = processor == PROCESSOR_NULL ? 0 : (uintptr_t)os_atomic_load(&processor->active_thread, relaxed);
	}
	if (!peek_word(&((queue_head_t *)head)->next, &link)) {
		return -1;
	}
	for (unsigned walked = 0; link != head && link != 0 && walked < 256 && found < max; walked++) {
		thread_t th = (thread_t)(link - offsetof(struct thread, task_threads));
		uintptr_t tro = 0, owner = 0;
		int state = 0;
		bool running = false;

		if (!peek_word(&th->t_tro, &tro) || !peek_word(&((struct thread_ro *)tro)->tro_task, &owner) || owner != task ||
		    apollo_hitch_peek((uintptr_t)&th->state, &state, sizeof(state)) != sizeof(state)) {
			break;
		}
		if ((state & (TH_RUN | TH_WAIT | TH_TERMINATE)) == TH_RUN) {
			for (unsigned cpu = 0; cpu < cpus; cpu++) {
				running |= active[cpu] == (uintptr_t)th;
			}
			if (!running &&
			    apollo_hitch_peek((uintptr_t)&th->thread_id, &tids[found], sizeof(tids[found])) == sizeof(tids[found]) &&
			    apollo_hitch_peek((uintptr_t)&th->last_made_runnable_time, &since_abs[found], sizeof(since_abs[found])) ==
			    sizeof(since_abs[found])) {
				threads[found++] = (uintptr_t)th;
			}
		}
		if (!peek_word(&th->task_threads.next, &link)) {
			break;
		}
	}
	return found;
}

/*
 * The sampler's clock: a timer call that is local to one processor (TIMER_CALL_LOCAL: it stays on the queue of the processor
 * that armed it, and re-arming it from its own callout keeps it there) and critical (TIMER_CALL_SYS_CRITICAL: no coalescing
 * leeway). Its callout runs in that processor's timer interrupt, so the sampler is never late for want of a thread being
 * scheduled, and the thread it interrupts is what that processor was doing.
 */
static timer_call_data_t hitch_timer;
static uint64_t hitch_interval_abs, hitch_deadline;
static void (*hitch_tick)(uint64_t now);

static void
hitch_fire(__unused timer_call_param_t p0, __unused timer_call_param_t p1)
{
	uint64_t now = mach_absolute_time();

	hitch_tick(now);
	hitch_deadline += hitch_interval_abs;
	if (hitch_deadline <= now) {
		hitch_deadline = now + hitch_interval_abs;      /* (late: the ticks missed are not made up for) */
	}
	timer_call_enter(&hitch_timer, hitch_deadline, TIMER_CALL_SYS_CRITICAL | TIMER_CALL_LOCAL);
}

/* From a thread (it is bound to `cpu` for the moment it takes to arm the timer there, then let go): false, not started. */
bool
apollo_hitch_timer(unsigned cpu, uint64_t interval_ns, void (*tick)(uint64_t now))
{
	processor_t processor = cpu < MAX_SCHED_CPUS ? processor_array[cpu] : PROCESSOR_NULL;
	bool armed = false;

	if (processor == PROCESSOR_NULL || tick == NULL || hitch_tick != NULL) {
		return false;
	}
	thread_bind(processor);
	thread_block(THREAD_CONTINUE_NULL);     /* (onto that processor) */
	if (cpu_number() == (int)cpu) {
		nanoseconds_to_absolutetime(interval_ns, &hitch_interval_abs);
		hitch_tick = tick;
		timer_call_setup(&hitch_timer, hitch_fire, NULL);
		hitch_deadline = mach_absolute_time() + hitch_interval_abs;
		timer_call_enter(&hitch_timer, hitch_deadline, TIMER_CALL_SYS_CRITICAL | TIMER_CALL_LOCAL);
		armed = true;
	}
	thread_bind(PROCESSOR_NULL);
	return armed;
}
