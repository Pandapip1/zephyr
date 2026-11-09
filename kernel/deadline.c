/*
 * Copyright (c) 2018 Intel Corporation
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief EDF (Earliest Deadline First) scheduling support
 *
 * Implements the public k_thread_deadline_set() and
 * k_thread_absolute_deadline_set() APIs.  The run-queue manipulation
 * required to keep deadline ordering is delegated back to the scheduler
 * via z_sched_prio_deadline_set() so that sched.c keeps full ownership
 * of the run-queue data structures.
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel/deadline_miss.h>
#include <deadline_admission.h>
#include <kspinlock.h>
#include <ksched.h>
#include <run_q.h>
#include <zephyr/internal/syscall_handler.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/minmax.h>
#include <zephyr/logging/log.h>
#include <errno.h>

void z_impl_k_deadline_event_init(struct k_deadline_event *event)
{
	k_spinlock_key_t key = z_sched_spinlock_lock();

	event->consumer = NULL;
	event->deadline = 0;
	event->pending = false;
	event->released_idle = false;
	z_sched_spinlock_unlock(key);
}

/* Record a publication; @p not_before is valid when @p has_not_before. */
static void deadline_event_post(struct k_deadline_event *event, bool has_not_before,
				uint32_t not_before, int deadline, uint32_t budget)
{
	/* Read the clock outside the scheduler lock: it may take a lock of its own. */
	uint32_t now = k_cycle_get_32();
	struct admission_verdict verdict;
	k_spinlock_key_t key = z_sched_spinlock_lock();
	struct k_thread *thread = event->consumer;
	bool idle = (thread != NULL) && z_is_thread_suspended(thread) &&
		    ((thread->base.thread_state & ~_THREAD_SUSPENDED) == 0U);
	/* The event as it was, restored if admission refuses this publication. */
	struct k_deadline_event before = *event;
	bool feasible;
	bool refused;

	/*
	 * Never count publications, but keep the earliest deadline among those
	 * coalesced: the run that consumes them must start by it.
	 */
	if (!event->pending || (int32_t)((uint32_t)deadline - (uint32_t)event->deadline) < 0) {
		event->deadline = deadline;
	}
	if (!event->pending) {
		/*
		 * A publication that finds its consumer idle releases a job that must
		 * start by its deadline. One latched while the consumer runs belongs
		 * to that busy period: the run in progress may serve it, so only its
		 * deadline, as an ordering key, carries over to the next run.
		 */
		event->released_idle = idle;
		event->release = now;
		event->released_over = _current;
		event->has_not_before = has_not_before;
		event->not_before = not_before;
	} else if (!has_not_before) {
		/* Coalesced with work due now: no not-before bound remains. */
		event->has_not_before = false;
	} else if (event->has_not_before &&
		   (int32_t)(not_before - event->not_before) < 0) {
		event->not_before = not_before;
	}
#ifdef CONFIG_SCHED_DEADLINE_ADMISSION
	if (budget == 0U) {
		budget = event->budget;
	}
	/* Coalesced publications are one run: it must fit the largest of their budgets. */
	if (!event->pending || budget > event->pending_budget) {
		event->pending_budget = budget;
	}
#else
	ARG_UNUSED(budget);
#endif
	event->pending = true;
	feasible = z_deadline_admission_posted(event, idle, now, &verdict);
	/*
	 * A publication admission refuses is not recorded: it neither releases
	 * the consumer nor joins a pending run, so admitted work keeps its
	 * guarantee. Admitted work found infeasible without it does not stop it.
	 */
	refused = !feasible && (verdict.victim == event);
	if (refused) {
		*event = before;
	} else if (idle) {
		/* Assign the EDF key before making the consumer observable as ready. */
		thread->base.prio_deadline = event->deadline;
		z_mark_thread_as_not_suspended(thread);
		z_sched_ready_locked(thread);
	}
	if (!feasible) {
		/* Report before a released consumer can run. */
		z_sched_spinlock_unlock(key);
		z_deadline_admission_report(&verdict);
		z_reschedule_unlocked();
	} else if (idle) {
		z_reschedule_locked(key);
	} else {
		z_sched_spinlock_unlock(key);
	}
}

void z_impl_k_deadline_event_post(struct k_deadline_event *event, int deadline)
{
	deadline_event_post(event, false, 0U, deadline, 0U);
}

void k_deadline_event_post_at(struct k_deadline_event *event, uint32_t not_before, int deadline)
{
	deadline_event_post(event, true, not_before, deadline, 0U);
}

void k_deadline_event_post_budget(struct k_deadline_event *event, int deadline, uint32_t budget)
{
	deadline_event_post(event, false, 0U, deadline, budget);
}

void z_impl_k_deadline_event_reset(struct k_deadline_event *event)
{
	k_spinlock_key_t key = z_sched_spinlock_lock();

	/* Do not alter consumer scheduling state: only discard an unread event. */
	event->pending = false;
	z_deadline_admission_reset(event);
	z_sched_spinlock_unlock(key);
}

/*
 * Start a run: wait out its not-before bound, then check a run released from
 * idle against its deadline.
 */
static uint32_t deadline_event_started(struct k_deadline_event *event, int deadline,
				   bool released_idle, uint32_t release,
				   struct k_thread *released_over, bool has_not_before,
				   uint32_t not_before)
{
	uint32_t now = k_cycle_get_32();

	/* Bounded by the release lead its poster chose; the spin is this job's own demand. */
	while (has_not_before && (int32_t)(now - not_before) < 0) {
		uint32_t us = k_cyc_to_us_floor32(not_before - now);

		/* Whole microseconds as a busy wait, so simulated time advances too. */
		if (us != 0U) {
			k_busy_wait(us);
		} else {
			arch_nop();
		}
		now = k_cycle_get_32();
	}
	z_deadline_miss_job_start(event, now);
#ifdef CONFIG_SCHED_DEADLINE_STATS
	if (released_idle && now - release > event->max_latency) {
		event->max_latency = now - release;
	}
#endif
	if (released_idle && (int32_t)(now - (uint32_t)deadline) > 0) {
		struct k_deadline_miss miss = {
			.kind = K_DEADLINE_MISS_START,
			.object = event,
			.name = event->name,
			.thread = _current,
			.release = release,
			.deadline = deadline,
			.now = now,
			.running_at_release = released_over,
		};

		z_deadline_miss_report(&miss);
	}
	return now;
}

void z_impl_k_deadline_event_wait(struct k_deadline_event *event)
{
	struct k_thread *thread = k_current_get();

	__ASSERT(!arch_is_in_isr(), "deadline event wait from ISR");

	if (IS_ENABLED(CONFIG_SCHED_DEADLINE_ADMISSION) ||
	    IS_ENABLED(CONFIG_SCHED_DEADLINE_STATS)) {
		k_spinlock_key_t key = z_sched_spinlock_lock();

		z_deadline_admission_finished(event);
		z_sched_spinlock_unlock(key);
#ifdef CONFIG_SCHED_DEADLINE_STATS
		if (event->run_open) {
			uint32_t run = k_cycle_get_32() - event->run_begin;

			event->max_run = MAX(event->max_run, run);
			event->run_open = false;
		}
#endif
	}
	for (;;) {
		k_spinlock_key_t key = z_sched_spinlock_lock();

		if (event->consumer == NULL) {
			event->consumer = thread;
		} else {
			__ASSERT(event->consumer == thread,
				 "deadline event has more than one consumer");
		}

		if (event->pending) {
			int deadline = event->deadline;
			bool released_idle = event->released_idle;
			uint32_t release = event->release;
			struct k_thread *released_over = event->released_over;
			bool has_not_before = event->has_not_before;
			uint32_t not_before = event->not_before;
#ifdef CONFIG_SCHED_DEADLINE_ADMISSION
			uint32_t budget = event->pending_budget;
#else
			uint32_t budget = 0U;
#endif

			event->pending = false;
			event->has_not_before = false;
			z_sched_spinlock_unlock(key);
			/*
			 * The consumer may have run since its release, and this
			 * publication may have been latched meanwhile: run it under
			 * its own deadline, never the key of an earlier run.
			 */
			z_sched_current_deadline_update(deadline);
			uint32_t start = deadline_event_started(event, deadline, released_idle,
								release, released_over,
								has_not_before, not_before);

			if (IS_ENABLED(CONFIG_SCHED_DEADLINE_ADMISSION)) {
				key = z_sched_spinlock_lock();
				z_deadline_admission_started(event, start, budget);
				z_sched_spinlock_unlock(key);
			}
#ifdef CONFIG_SCHED_DEADLINE_STATS
			event->run_begin = start;
			event->run_open = true;
#endif
			return;
		}

		/*
		 * z_thread_halt() changes the current thread to suspended before it
		 * releases the scheduler lock.  A post therefore either observes the
		 * pending bit above or releases this pure-suspended consumer; there is
		 * no lost-publication window between the two operations.
		 */
		z_thread_halt(thread, key, false);
	}
}

#ifdef CONFIG_USERSPACE
static inline void z_vrfy_k_deadline_event_init(struct k_deadline_event *event)
{
	K_OOPS(K_SYSCALL_OBJ_INIT(event, K_OBJ_DEADLINE_EVENT));
	z_impl_k_deadline_event_init(event);
}
#include <zephyr/syscalls/k_deadline_event_init_mrsh.c>

static inline void z_vrfy_k_deadline_event_post(struct k_deadline_event *event,
						 int deadline)
{
	K_OOPS(K_SYSCALL_OBJ(event, K_OBJ_DEADLINE_EVENT));
	z_impl_k_deadline_event_post(event, deadline);
}
#include <zephyr/syscalls/k_deadline_event_post_mrsh.c>

static inline void z_vrfy_k_deadline_event_reset(struct k_deadline_event *event)
{
	K_OOPS(K_SYSCALL_OBJ(event, K_OBJ_DEADLINE_EVENT));
	z_impl_k_deadline_event_reset(event);
}
#include <zephyr/syscalls/k_deadline_event_reset_mrsh.c>

static inline void z_vrfy_k_deadline_event_wait(struct k_deadline_event *event)
{
	K_OOPS(K_SYSCALL_OBJ(event, K_OBJ_DEADLINE_EVENT));
	z_impl_k_deadline_event_wait(event);
}
#include <zephyr/syscalls/k_deadline_event_wait_mrsh.c>
#endif

/**
 * @brief Atomically update a thread's prio_deadline in the run queue.
 *
 * If the thread is currently queued the scheduler re-inserts it so that
 * run-queue ordering (including rbtree invariants) is preserved.  May be
 * called from any context; takes and releases the scheduler's spinlock
 * internally.
 *
 * @param thread   Thread whose deadline is being updated.
 * @param deadline New absolute deadline value (raw cycle counter).
 */
void z_sched_prio_deadline_set(struct k_thread *thread, int deadline)
{
	Z_SCHED_SPINLOCK {
		if (z_is_thread_queued(thread)) {
			dequeue_thread(thread);
			thread->base.prio_deadline = deadline;
			queue_thread(thread);
		} else {
			thread->base.prio_deadline = deadline;
		}
	}
}

void z_impl_k_thread_absolute_deadline_set(k_tid_t tid, int deadline)
{
	struct k_thread *thread = tid;

	/* The prio_deadline field changes the sorting order, so it must
	 * not be altered while the thread is in the run queue.  Because
	 * an rbtree will blow up if sorting invariants are violated mid-
	 * update, we delegate the dequeue/update/requeue sequence to
	 * z_sched_prio_deadline_set() which performs it under the
	 * scheduler spinlock.
	 */
	z_sched_prio_deadline_set(thread, deadline);
}

void z_impl_k_thread_deadline_set(k_tid_t tid, int deadline)
{
	/*
	 * Clamp the relative deadline to INT32_MAX / 2 (2^30 cycles) to
	 * satisfy the scheduler comparator's half-modulus invariant.
	 * This leaves a 2^30-cycle margin of tolerance for scheduler
	 * execution skew before priority inversion can occur.
	 */
	deadline = clamp(deadline, 0, INT32_MAX / 2);

	/* Use uint32_t arithmetic to avoid implementation-defined signed conversion. */
	uint32_t newdl = k_cycle_get_32() + (uint32_t)deadline;

	z_impl_k_thread_absolute_deadline_set(tid, (int)newdl);
}

int z_impl_k_thread_release_with_deadline(k_tid_t tid, int deadline)
{
	struct k_thread *thread = tid;
	k_spinlock_key_t key = z_sched_spinlock_lock();

	/*
	 * Suspension is the explicit persistent-job idle state.  Do not turn this
	 * API into a timeout wakeup: sleepers and object waiters retain their
	 * K_TIMEOUT release semantics.
	 */
	if (unlikely(!z_is_thread_suspended(thread) ||
		     (thread->base.thread_state & ~_THREAD_SUSPENDED) != 0U)) {
		z_sched_spinlock_unlock(key);
		return -EALREADY;
	}

	/* The thread is not queued while suspended, so set the ordering key before
	 * adding it to the ready queue.  This prevents an intermediate, stale
	 * deadline from being observable by the scheduler.
	 */
	thread->base.prio_deadline = deadline;
	z_mark_thread_as_not_suspended(thread);
	z_sched_ready_locked(thread);
	z_reschedule_locked(key);

	return 0;
}

#ifdef CONFIG_USERSPACE
static inline void z_vrfy_k_thread_absolute_deadline_set(k_tid_t tid, int deadline)
{
	struct k_thread *thread = tid;

	K_OOPS(K_SYSCALL_OBJ(thread, K_OBJ_THREAD));

	z_impl_k_thread_absolute_deadline_set((k_tid_t)thread, deadline);
}
#include <zephyr/syscalls/k_thread_absolute_deadline_set_mrsh.c>

static inline void z_vrfy_k_thread_deadline_set(k_tid_t tid, int deadline)
{
	struct k_thread *thread = tid;

	K_OOPS(K_SYSCALL_OBJ(thread, K_OBJ_THREAD));
	K_OOPS(K_SYSCALL_VERIFY_MSG(deadline > 0,
				    "invalid thread deadline %d",
				    (int)deadline));

	z_impl_k_thread_deadline_set((k_tid_t)thread, deadline);
}
#include <zephyr/syscalls/k_thread_deadline_set_mrsh.c>

static inline int z_vrfy_k_thread_release_with_deadline(k_tid_t tid,
						 int deadline)
{
	struct k_thread *thread = tid;

	K_OOPS(K_SYSCALL_OBJ(thread, K_OBJ_THREAD));

	return z_impl_k_thread_release_with_deadline((k_tid_t)thread, deadline);
}
#include <zephyr/syscalls/k_thread_release_with_deadline_mrsh.c>
#endif /* CONFIG_USERSPACE */

#ifdef CONFIG_SCHED_DEADLINE_STATS
void k_deadline_event_stats_get(const struct k_deadline_event *event, uint32_t *max_run,
				uint32_t *max_latency)
{
	*max_run = event->max_run;
	*max_latency = event->max_latency;
}
#endif
