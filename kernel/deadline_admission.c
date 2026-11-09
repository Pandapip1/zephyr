/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Admission control for deadline events.
 *
 * Every deadline event with a budget (its worst-case execution time per run)
 * is a job source. A release is checked before it runs: the scheduler's own
 * policy is simulated over the work it already holds, and the release is
 * feasible when every job not yet started can still start by its start-by
 * deadline. The simulation is exact for that policy:
 *
 * - A ready thread of higher priority preempts a lower one.
 * - Within one priority, the earliest deadline goes first; a running
 *   cooperative thread (meta-IRQ ones included) is not preempted by its own
 *   priority, a preemptible one is.
 * - A run started by k_deadline_event_post_at() occupies its thread from
 *   dispatch until its not-before instant, then runs its budget.
 *
 * The job set is: the run in progress (its budget less the time since it
 * started, or its whole budget if another thread preempted it), released runs
 * not yet started, admitted timed releases not yet posted, and the work being
 * admitted. Work outside admission control is bounded by one blocking term,
 * CONFIG_SCHED_DEADLINE_ADMISSION_BLOCKING_NS, taken at the start of the
 * check: a lower-priority section with interrupts or the scheduler locked,
 * and interrupt handling.
 *
 * An admitted job keeps its guarantee. When a release or an admission would
 * make some job start late, the request fails and is reported: a request is
 * never admitted by revoking work admitted before it, whatever the
 * priorities. Only when the admitted set is infeasible without the request
 * too is admitted work reported: the lowest priority among the late jobs,
 * and of those the latest deadline.
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel/deadline_miss.h>
#include <zephyr/kernel/deferred_release.h>
#include <zephyr/sys/slist.h>
#include <limits.h>
#include <kthread.h>
#include <ksched.h>
#include <deadline_admission.h>

#define BLOCKING_CYCLES \
	((uint32_t)k_ns_to_cyc_ceil64(CONFIG_SCHED_DEADLINE_ADMISSION_BLOCKING_NS))

enum {
	ADMIT_IDLE,
	/* Released and not yet started. */
	ADMIT_PENDING,
	/* Between the wait that started a run and the next wait. */
	ADMIT_RUNNING,
};

static sys_slist_t admitted;

struct sim_job {
	struct k_deadline_event *event;
	/* Times relative to the check, in cycles. */
	int32_t release;
	int32_t not_before;
	int32_t deadline;
	uint32_t remaining;
	/* Work the job brings, for reports. */
	uint32_t work;
	int prio;
	bool started;
	bool cooperative;
	bool late;
	/* Its deadline only orders it: a run latched behind the run in progress. */
	bool ordering_only;
};

struct sim {
	struct sim_job jobs[CONFIG_SCHED_DEADLINE_ADMISSION_JOBS];
	size_t count;
	/* Index of the job running when the check starts, or -1. */
	int running;
	/* CPU unavailable to every job from the start of the check. */
	int32_t blocking;
	/* A job did not fit in @ref jobs: the check cannot prove the set feasible. */
	bool overflow;
};

static int32_t rel(uint32_t t, uint32_t now)
{
	return (int32_t)(t - now);
}

static struct sim_job *sim_add(struct sim *s)
{
	if (s->count == ARRAY_SIZE(s->jobs)) {
		s->overflow = true;
		return NULL;
	}
	s->jobs[s->count] = (struct sim_job){0};
	return &s->jobs[s->count++];
}

/* @p a runs before @p b when both are ready and neither is running. */
static bool sim_before(const struct sim_job *a, const struct sim_job *b)
{
	if (a->prio != b->prio) {
		return a->prio < b->prio;
	}
	return (int32_t)(a->deadline - b->deadline) < 0;
}

/*
 * Run the scheduler's policy over the job set from time 0 and mark every job
 * that starts after its deadline. Returns true when none does.
 */
static bool sim_run(struct sim *s)
{
	int32_t t = s->blocking;
	int current = s->running;
	bool ok = true;

	/* Each step finishes a job or reaches a release: bounded by twice the jobs. */
	for (size_t step = 0U; step < 2U * ARRAY_SIZE(s->jobs) + 2U; step++) {
		int best = -1;
		int32_t next_release = INT32_MAX;

		for (size_t i = 0U; i < s->count; i++) {
			struct sim_job *j = &s->jobs[i];

			if (j->remaining == 0U) {
				continue;
			}
			if (j->release > t) {
				next_release = MIN(next_release, j->release);
				continue;
			}
			if (best < 0 || sim_before(j, &s->jobs[best])) {
				best = (int)i;
			}
		}
		/* A running cooperative job keeps the CPU against its own priority. */
		if (current >= 0 && s->jobs[current].remaining != 0U && best >= 0 &&
		    best != current && s->jobs[current].cooperative &&
		    s->jobs[best].prio >= s->jobs[current].prio) {
			best = current;
		}
		if (best < 0) {
			if (next_release == INT32_MAX) {
				break;
			}
			t = next_release;
			continue;
		}

		struct sim_job *j = &s->jobs[best];

		if (!j->started) {
			int32_t start = MAX(t, j->not_before);

			j->started = true;
			/* Dispatched early, it spins at its priority until its not-before. */
			j->remaining += (uint32_t)(start - t);
			if (start - j->deadline > 0 && !j->ordering_only) {
				j->late = true;
				ok = false;
			}
		}
		current = best;

		int32_t run = (int32_t)MIN((uint64_t)j->remaining, (uint64_t)INT32_MAX);

		if (next_release != INT32_MAX && next_release - t < run) {
			run = next_release - t;
		}
		j->remaining -= (uint32_t)run;
		t += run;
	}
	return ok;
}

/*
 * Fill @p s with the admitted job set at @p now. Without the run @p exclude's
 * publications have not started (its pending run, or the run latched behind
 * its run in progress) when @p exclude is not NULL. Caller holds the scheduler
 * lock.
 */
static void sim_collect(struct sim *s, uint32_t now, const struct k_deadline_event *exclude)
{
	struct k_deadline_event *event;

	s->count = 0U;
	s->running = -1;
	s->overflow = false;

	/* Lower-priority non-preemptible sections and interrupts: ahead of everything. */
	s->blocking = (int32_t)BLOCKING_CYCLES;

	SYS_SLIST_FOR_EACH_CONTAINER(&admitted, event, admission_node) {
		struct k_thread *consumer = event->consumer;
		struct sim_job *j;

		if (consumer == NULL) {
			continue;
		}
		if (event->admission_state == ADMIT_RUNNING) {
			j = sim_add(s);
			if (j == NULL) {
				break;
			}
			j->started = true;
			j->remaining = event->run_budget;
			if (consumer == _current) {
				uint32_t elapsed = now - event->run_start;

				j->remaining = (elapsed < event->run_budget) ?
					       event->run_budget - elapsed : 0U;
				if (s->running < 0) {
					s->running = (int)(j - s->jobs);
				}
			}
			j->deadline = rel((uint32_t)consumer->base.prio_deadline, now);
			j->work = event->run_budget;
			if (event->pending && event != exclude) {
				/* A run latched behind this one. */
				struct sim_job *next = sim_add(s);

				if (next != NULL) {
					*next = (struct sim_job){
						.event = event,
						.deadline = rel((uint32_t)event->deadline, now),
						.remaining = event->pending_budget,
						.work = event->pending_budget,
						.prio = consumer->base.prio,
						.cooperative = consumer->base.prio < 0,
						.ordering_only = true,
					};
				}
			}
		} else if (event->admission_state == ADMIT_PENDING && event != exclude) {
			j = sim_add(s);
			if (j == NULL) {
				break;
			}
			j->remaining = event->pending_budget;
			j->deadline = rel((uint32_t)event->deadline, now);
			j->not_before = event->has_not_before ?
					 MAX(0, rel(event->not_before, now)) : 0;
			j->work = event->pending_budget;
		} else {
			j = NULL;
		}
		if (j != NULL) {
			j->event = event;
			j->prio = consumer->base.prio;
			j->cooperative = consumer->base.prio < 0;
		}
		if (event->future) {
			j = sim_add(s);
			if (j == NULL) {
				break;
			}
			j->event = event;
			j->remaining = event->budget;
			j->release = MAX(0, rel(event->future_release, now));
			j->work = event->budget;
			j->not_before = MAX(j->release, rel(event->future_not_before, now));
			j->deadline = rel((uint32_t)event->future_deadline, now);
			j->prio = consumer->base.prio;
			j->cooperative = consumer->base.prio < 0;
		}
	}
}

/* The late job priority sacrifices: lowest priority, then latest deadline. */
static const struct sim_job *sim_victim(const struct sim *s)
{
	const struct sim_job *victim = NULL;

	for (size_t i = 0U; i < s->count; i++) {
		const struct sim_job *j = &s->jobs[i];

		if (!j->late) {
			continue;
		}
		if (victim == NULL || j->prio > victim->prio ||
		    (j->prio == victim->prio && j->deadline - victim->deadline > 0)) {
			victim = j;
		}
	}
	return victim;
}

/* Work ahead of @p victim: everything that ran before its start in the simulation. */
static uint32_t sim_demand(const struct sim *s, const struct sim_job *victim)
{
	uint32_t demand = (uint32_t)s->blocking;

	for (size_t i = 0U; i < s->count; i++) {
		const struct sim_job *j = &s->jobs[i];

		bool ahead = (j->prio < victim->prio) ||
			     (j->prio == victim->prio && j->deadline - victim->deadline <= 0);

		if (j != victim && ahead) {
			demand += j->work;
		}
	}
	return demand;
}

/*
 * Run the check over the current job set plus the request: @p extra, or the
 * publication of @p requester just recorded when @p extra is NULL. @p requester
 * is the event whose release or admission asks for the check, NULL for bare
 * demand. Caller holds the scheduler lock.
 *
 * An admitted job keeps its guarantee: when the request would make work late,
 * the request is what fails, reported with the deadline and the demand of the
 * job it would make late. Only when the set is infeasible without the request
 * too (admitted work already cannot be met) is that work reported.
 */
static bool admission_check(uint32_t now, const struct sim_job *extra,
			    struct k_deadline_event *requester,
			    struct admission_verdict *verdict)
{
	/* One check at a time: the simulation state is too large for small stacks. */
	static struct sim s;
	const struct sim_job *victim;

	sim_collect(&s, now, NULL);
	if (extra != NULL) {
		struct sim_job *j = sim_add(&s);

		if (j != NULL) {
			*j = *extra;
		}
	}
	if (s.overflow) {
		/* A job set larger than the check cannot be proved: refuse the request. */
		verdict->victim = requester;
		verdict->now = now;
		verdict->release = now;
		verdict->deadline = (int)now;
		verdict->demand = 0U;
		return false;
	}
	if (sim_run(&s)) {
		return true;
	}
	victim = sim_victim(&s);
	verdict->victim = victim->event;
	verdict->now = now;
	verdict->release = now + (uint32_t)victim->release;
	verdict->deadline = (int)(now + (uint32_t)victim->deadline);
	verdict->demand = sim_demand(&s, victim);
	if (victim->event == requester && requester != NULL) {
		return false;
	}
	/* Admitted work is late: is it late without the request too? */
	sim_collect(&s, now, (extra != NULL) ? NULL : requester);
	if (s.overflow || sim_run(&s)) {
		verdict->victim = requester;
		return false;
	}
	victim = sim_victim(&s);
	verdict->victim = victim->event;
	verdict->release = now + (uint32_t)victim->release;
	verdict->deadline = (int)(now + (uint32_t)victim->deadline);
	verdict->demand = sim_demand(&s, victim);
	return false;
}

static void admission_report(const struct admission_verdict *verdict, struct k_thread *running)
{
	struct k_deadline_miss miss = {
		.kind = K_DEADLINE_MISS_UNSAT,
		.object = verdict->victim,
		.name = (verdict->victim != NULL) ? verdict->victim->name : NULL,
		.thread = (verdict->victim != NULL) ? verdict->victim->consumer : NULL,
		.release = verdict->release,
		.deadline = verdict->deadline,
		.now = verdict->now,
		.running_at_release = running,
		.demand = verdict->demand,
	};

	z_deadline_miss_report(&miss);
}

void k_deadline_event_budget_set(struct k_deadline_event *event, uint32_t budget)
{
	k_spinlock_key_t key = z_sched_spinlock_lock();

	if (event->budget == 0U && budget != 0U) {
		sys_slist_append(&admitted, &event->admission_node);
	} else if (event->budget != 0U && budget == 0U) {
		(void)sys_slist_find_and_remove(&admitted, &event->admission_node);
	}
	event->budget = budget;
	z_sched_spinlock_unlock(key);
}

/*
 * A publication of @p event at @p now has been recorded; @p idle when it
 * released the consumer. Caller holds the scheduler lock. Returns false with
 * @p verdict filled when the release makes the job set infeasible.
 */
bool z_deadline_admission_posted(struct k_deadline_event *event, bool idle, uint32_t now,
				 struct admission_verdict *verdict)
{
	if (event->budget == 0U) {
		return true;
	}
	/* A timed release admitted when it was armed is now this publication. */
	bool armed = event->future;

	ARG_UNUSED(idle);
	event->future = false;
	/*
	 * A publication latched while the consumer runs is served by, or queued
	 * behind, the run in progress; its deadline only orders the next run. Its
	 * budget is still work the other admitted jobs must fit around, as is a
	 * larger budget coalesced into a pending run: every publication not
	 * admitted when it was armed is checked.
	 */
	if (event->admission_state != ADMIT_RUNNING) {
		event->admission_state = ADMIT_PENDING;
	}
	if (armed) {
		return true;
	}
	return admission_check(now, NULL, event, verdict);
}

void z_deadline_admission_started(struct k_deadline_event *event, uint32_t now, uint32_t budget)
{
	if (event->budget != 0U) {
		event->run_start = now;
		event->run_budget = budget;
		event->admission_state = ADMIT_RUNNING;
	}
}

void z_deadline_admission_finished(struct k_deadline_event *event)
{
	if (event->budget != 0U) {
		event->admission_state = event->pending ? ADMIT_PENDING : ADMIT_IDLE;
	}
}

void z_deadline_admission_reset(struct k_deadline_event *event)
{
	if (event->budget != 0U && event->admission_state == ADMIT_PENDING) {
		event->admission_state = ADMIT_IDLE;
	}
}

void z_deadline_admission_report(const struct admission_verdict *verdict)
{
	admission_report(verdict, k_current_get());
}

int k_deadline_event_admit(struct k_deadline_event *event, uint32_t release, uint32_t not_before,
			   int deadline)
{
	uint32_t now = k_cycle_get_32();
	struct admission_verdict verdict;
	bool ok = true;
	k_spinlock_key_t key = z_sched_spinlock_lock();

	if (event->budget != 0U && event->consumer != NULL) {
		event->future = false;
		event->future_release = release;
		event->future_not_before = not_before;
		event->future_deadline = deadline;
		ok = admission_check(now,
				     &(struct sim_job){
					     .event = event,
					     .release = MAX(0, rel(release, now)),
					     .not_before = MAX(0, rel(not_before, now)),
					     .deadline = rel((uint32_t)deadline, now),
					     .remaining = event->budget,
					     .work = event->budget,
					     .prio = event->consumer->base.prio,
					     .cooperative = event->consumer->base.prio < 0,
				     },
				     event, &verdict);
		/* Admitted or not, it is armed: later checks must see it. */
		event->future = true;
	}
	z_sched_spinlock_unlock(key);
	if (!ok) {
		admission_report(&verdict, k_current_get());
		return -ETIME;
	}
	return 0;
}

void k_deadline_event_admit_cancel(struct k_deadline_event *event)
{
	k_spinlock_key_t key = z_sched_spinlock_lock();

	event->future = false;
	z_sched_spinlock_unlock(key);
}

int k_deadline_demand_admit(int prio, uint32_t cycles)
{
	uint32_t now = k_cycle_get_32();
	struct admission_verdict verdict;
	bool ok;
	k_spinlock_key_t key = z_sched_spinlock_lock();

	ok = admission_check(now,
			     &(struct sim_job){
				     .release = 0,
				     .deadline = INT32_MAX,
				     .remaining = cycles,
				     .work = cycles,
				     .prio = prio,
				     .cooperative = true,
			     },
			     NULL, &verdict);
	z_sched_spinlock_unlock(key);
	if (!ok) {
		admission_report(&verdict, k_current_get());
		return -ETIME;
	}
	return 0;
}

#ifdef CONFIG_CLOCK_EVENT_SCHED
int k_deadline_event_admit_plan(struct k_deadline_event *event,
				const struct k_clock_event_sched_plan *plan, uint32_t arm_at,
				uint32_t not_before, int deadline)
{
	/* Arriving late by the rung's granularity delays the start as much. */
	uint32_t late = (uint32_t)plan->deliver_by - arm_at;
	int ret = 0;

	if (plan->caller_spin != 0U) {
		int prio = k_is_in_isr() ? INT_MIN : k_thread_priority_get(k_current_get());

		if (k_deadline_demand_admit(prio, (uint32_t)plan->caller_spin) != 0) {
			ret = -ETIME;
		}
	}
	if (plan->isr_spin != 0U &&
	    k_deadline_demand_admit(INT_MIN, (uint32_t)plan->isr_spin) != 0) {
		ret = -ETIME;
	}
	if (k_deadline_event_admit(event, arm_at, not_before + late, deadline) != 0) {
		ret = -ETIME;
	}
	return ret;
}
#endif
