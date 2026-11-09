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
#include <timeout_q.h>

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
/* Armed recurring reservations (k_deadline_event_arm(), k_deadline_event_arm_periodic()). */
static sys_slist_t reservations;

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
	/* Running when the check started. */
	bool running_at_check;
	/* When the simulation started it, without interference. */
	int32_t start;
	/*
	 * A recurring reservation, not a run: releases at any time, no two closer
	 * than its minimum inter-arrival (plus its jitter), each @ref work of
	 * thread time at its priority and its handler at interrupt level, started
	 * within @ref deadline of its release. The simulation does not run it;
	 * sim_bound() checks it.
	 */
	const struct k_deadline_reservation *res;
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
	/*
	 * What the simulation ran: segment i ran priority seg_prio[i] over
	 * [seg_start[i], seg_end[i]) (the blocking term as the highest priority);
	 * the CPU was idle outside them.
	 */
	size_t segs;
	int32_t seg_start[2U * CONFIG_SCHED_DEADLINE_ADMISSION_JOBS + 3U];
	int32_t seg_end[2U * CONFIG_SCHED_DEADLINE_ADMISSION_JOBS + 3U];
	int16_t seg_prio[2U * CONFIG_SCHED_DEADLINE_ADMISSION_JOBS + 3U];
};

/* Record that @p prio ran over [@p start, @p end), merged with a contiguous run of the same. */
static void sim_segment(struct sim *s, int32_t start, int32_t end, int prio)
{
	if (end <= start) {
		return;
	}
	if (s->segs > 0U && s->seg_end[s->segs - 1U] == start &&
	    s->seg_prio[s->segs - 1U] == (int16_t)prio) {
		s->seg_end[s->segs - 1U] = end;
		return;
	}
	if (s->segs == ARRAY_SIZE(s->seg_start)) {
		/* Cannot happen: a step adds at most one segment. Not proved if it did. */
		s->overflow = true;
		return;
	}
	s->seg_start[s->segs] = start;
	s->seg_end[s->segs] = end;
	s->seg_prio[s->segs] = (int16_t)prio;
	s->segs++;
}

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

	s->segs = 0U;
	sim_segment(s, 0, s->blocking, INT16_MIN);

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
			j->start = start;
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
		sim_segment(s, t, t + run, j->prio);
		t += run;
	}
	return ok;
}

/* Add @p res, a reservation of @p event, as a recurring job. */
static void sim_add_reservation(struct sim *s, struct k_deadline_event *event,
				const struct k_deadline_reservation *res)
{
	struct sim_job *j = sim_add(s);

	if (j == NULL) {
		return;
	}
	*j = (struct sim_job){
		.event = event,
		.deadline = (int32_t)res->deadline,
		.work = res->budget,
		.prio = event->consumer->base.prio,
		.cooperative = event->consumer->base.prio < 0,
		.res = res,
	};
}

static bool is_metairq(int prio)
{
	return (CONFIG_NUM_METAIRQ_PRIORITIES > 0) &&
	       (prio < K_HIGHEST_THREAD_PRIO + CONFIG_NUM_METAIRQ_PRIORITIES);
}

/*
 * Releasing @p a enters the scheduler (the post that releases it is a
 * scheduling point), so it takes the CPU at once from lower-priority @p b,
 * unless the policy keeps @p b running there: a cooperative @p b keeps the CPU
 * against everything but a meta-IRQ @p a.
 */
static bool preempts(const struct sim_job *a, const struct sim_job *b)
{
	return is_metairq(a->prio) || !b->cooperative;
}

/*
 * The longest run that can hold the CPU against @p a once a has been
 * released, because the policy does not switch to a there: a lower-priority
 * job @p a does not preempt, and for a cooperative @p a the same-priority one-
 * shot job that was released earlier with a later deadline (a run it would
 * have followed, which interference can make start first).
 */
static uint32_t sim_blocker(const struct sim *s, const struct sim_job *a)
{
	uint32_t blocker = 0U;

	for (size_t i = 0U; i < s->count; i++) {
		const struct sim_job *k = &s->jobs[i];
		bool blocks = false;

		if (k == a) {
			continue;
		}
		if (k->prio > a->prio) {
			blocks = !preempts(a, k);
		} else if (k->prio == a->prio && a->cooperative && a->res == NULL && k->res == NULL) {
			blocks = !k->running_at_check && k->release < a->release &&
				 k->deadline - a->deadline > 0;
		}
		if (blocks) {
			blocker = MAX(blocker, k->work);
		}
	}
	return blocker;
}

/*
 * Recurring work that can run ahead of @p a in any interval of length @p x:
 * every reservation other than a's, (floor((x + jitter) / period) + 1)
 * releases, each its handler at interrupt level and, at a's priority or above,
 * its run.
 */
static uint64_t sim_recurring(const struct sim *s, const struct sim_job *a, uint64_t x)
{
	uint64_t demand = 0U;

	for (size_t i = 0U; i < s->count; i++) {
		const struct sim_job *k = &s->jobs[i];
		uint64_t releases;

		if (k->res == NULL || k->event == a->event) {
			continue;
		}
		releases = (x + k->res->jitter) / k->res->min_interarrival + 1U;
		demand += releases * k->res->isr_budget;
		if (k->prio <= a->prio) {
			demand += releases * k->work;
		}
	}
	return demand;
}

/* One-shot work released in (@p from, @p to] that runs ahead of @p a. */
static uint64_t sim_released_ahead(const struct sim *s, const struct sim_job *a, int64_t from,
				   int64_t to)
{
	uint64_t demand = 0U;

	for (size_t i = 0U; i < s->count; i++) {
		const struct sim_job *k = &s->jobs[i];

		if (k == a || k->res != NULL || k->running_at_check) {
			continue;
		}
		if (k->release <= from || k->release > to) {
			continue;
		}
		if (k->prio < a->prio || (k->prio == a->prio && k->deadline - a->deadline <= 0)) {
			demand += k->work;
		}
	}
	return demand;
}

/*
 * Least fixed point of w = base + R(w) + W(@p from, @p origin + w), the time
 * from @p origin by which @p a starts, or a value past @p limit. W is left out
 * when @p from is negative.
 */
#define SIM_FIXED_POINT_STEPS 32U

static int64_t sim_fixed_point(const struct sim *s, const struct sim_job *a, int64_t base,
			       int64_t origin, int64_t from, int64_t limit)
{
	int64_t w = base;

	for (uint32_t step = 0U; step < SIM_FIXED_POINT_STEPS; step++) {
		int64_t next = base + (int64_t)sim_recurring(s, a, (uint64_t)MAX(w, 0)) +
			       ((from >= 0) ? (int64_t)sim_released_ahead(s, a, from, origin + w)
					    : 0);

		if (next == w || origin + next > limit) {
			return next;
		}
		w = next;
	}
	/* Not settled: not proved. */
	return limit + 1 - origin;
}

/* Work at priority @p prio or above the simulation ran in [@p from, @p to). */
static int64_t sim_level_work(const struct sim *s, int prio, int32_t from, int32_t to)
{
	int64_t work = 0;

	for (size_t i = 0U; i < s->segs; i++) {
		int32_t a = MAX(s->seg_start[i], from);
		int32_t b = MIN(s->seg_end[i], to);

		if (s->seg_prio[i] <= prio && b > a) {
			work += b - a;
		}
	}
	return work;
}

/* The simulation ran priority @p prio or above just before @p t. */
static bool sim_level_busy_before(const struct sim *s, int prio, int32_t t)
{
	for (size_t i = 0U; i < s->segs; i++) {
		if (s->seg_prio[i] <= prio && s->seg_start[i] < t && s->seg_end[i] >= t) {
			return true;
		}
	}
	return false;
}

/*
 * Latest start of one-shot job @p j, which the simulation started at s
 * without interference. The level-p busy period (p its priority) in which it
 * really starts begins at a time u at which the CPU ran nothing at p or above.
 * Interference only delays work, so the simulation, which has done at least as
 * much by any time, was idle at level p there too: u is 0 or the start of one
 * of the simulation's level-p busy periods at or before s. From such a u, j
 * starts within the least fixed point of
 *   w = L(u, s) + B + R(w) + W(s, u + w),
 * L the level-p work the simulation ran in [u, s): the bound is the largest
 * u + w over the candidates.
 */
static int64_t sim_one_shot_start(const struct sim *s, const struct sim_job *j)
{
	int64_t blocker = sim_blocker(s, j);
	int64_t latest = j->start;

	for (size_t i = 0U; i <= s->segs; i++) {
		int32_t u;
		int64_t w;

		if (i == s->segs) {
			u = 0;
		} else {
			u = s->seg_start[i];
			if (s->seg_prio[i] > j->prio || u > j->start ||
			    sim_level_busy_before(s, j->prio, u)) {
				continue;
			}
		}
		w = sim_fixed_point(s, j, sim_level_work(s, j->prio, u, j->start) + blocker, u,
				    j->start, j->deadline);
		latest = MAX(latest, (int64_t)u + w);
		if (latest > j->deadline) {
			break;
		}
	}
	return latest;
}

/*
 * With reservations, the simulation's starts are not the whole story:
 * releases of recurring jobs can come at any time. Check each job against
 * them, conservatively:
 *
 * - A one-shot job the simulation starts at s, interference-free: per busy
 *   period, sim_one_shot_start(). R is the recurring work of an interval of
 *   length w (sim_recurring(): any phase), W the one-shot work released after
 *   s that runs ahead of it, B one blocker the policy will not switch from
 *   (sim_blocker()). Delaying a preemptive schedule by interference delays a
 *   start by at most that interference; within a cooperative priority it can
 *   also reorder runs (a run delayed past a later release lets that release
 *   start first and hold the CPU), which B covers.
 * - A recurring job, released at the worst moment, starts by the least x with
 *   x = blocking + B + O + R(x): O all the one-shot work at its priority or
 *   above (each runs once, wherever it falls), R the other reservations.
 *
 * Without reservations the simulation is exact and nothing is added.
 */
static bool sim_bound(struct sim *s)
{
	bool any = false;
	bool ok = true;

	for (size_t i = 0U; i < s->count; i++) {
		any = any || (s->jobs[i].res != NULL);
	}
	if (!any) {
		return true;
	}
	for (size_t i = 0U; i < s->count; i++) {
		struct sim_job *j = &s->jobs[i];
		int64_t x;

		if (j->ordering_only || j->running_at_check || j->late) {
			continue;
		}
		if (j->res != NULL) {
			int64_t base = (int64_t)s->blocking + sim_blocker(s, j);

			for (size_t k = 0U; k < s->count; k++) {
				const struct sim_job *o = &s->jobs[k];

				if (o->res == NULL && o->prio <= j->prio) {
					base += o->work;
				}
			}
			x = sim_fixed_point(s, j, base, 0, -1, j->deadline);
		} else if (!j->started) {
			x = (int64_t)j->deadline + 1;
		} else {
			x = sim_one_shot_start(s, j);
		}
		if (x > j->deadline) {
			j->late = true;
			ok = false;
		}
	}
	return ok;
}

/* Simulate, then bound against the reservations. */
static bool sim_check(struct sim *s)
{
	bool ok = sim_run(s);

	return sim_bound(s) && ok;
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
			j->running_at_check = true;
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
		} else if (event->admission_state == ADMIT_PENDING && event != exclude &&
			   event->reservation == NULL) {
			/* An armed event's released run is one of its reservation's. */
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

	struct k_deadline_reservation *res;

	SYS_SLIST_FOR_EACH_CONTAINER(&reservations, res, node) {
		sim_add_reservation(s, res->event, res);
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

	verdict->kind = K_DEADLINE_MISS_UNSAT;
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
	if (sim_check(&s)) {
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
	if (s.overflow || sim_check(&s)) {
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
		.kind = verdict->kind,
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

void z_deadline_admission_publication(const struct k_deadline_event *event, uint32_t now,
				      int *deadline, uint32_t *budget)
{
	const struct k_deadline_reservation *res = event->reservation;

	if (res != NULL) {
		*deadline = (int)(now + res->deadline);
		*budget = res->budget;
	}
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
	if (event->reservation != NULL) {
		struct k_deadline_reservation *res = event->reservation;

		/*
		 * A release of an armed event was admitted when it was armed: it is
		 * never checked, and only a release outside the model is refused (the
		 * reservation covers one run outstanding at a time, releases no closer
		 * than its minimum inter-arrival). A periodic one's spacing is the
		 * kernel's own.
		 */
		if (event->admission_state != ADMIT_IDLE ||
		    (!res->periodic && res->released &&
		     now - res->last_release < res->min_interarrival)) {
			*verdict = (struct admission_verdict){
				.kind = K_DEADLINE_MISS_OVERRUN,
				.victim = event,
				.now = now,
				.release = now,
				.deadline = event->deadline,
			};
			return false;
		}
		res->released = true;
		res->last_release = now;
		event->future = false;
		event->admission_state = ADMIT_PENDING;
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

/*
 * A periodic reservation's release. The next one is scheduled here, at the
 * release, not when the run finishes: going back to sleep cannot fail.
 */
static void periodic_release(struct _timeout *timeout)
{
	struct k_deadline_reservation *res =
		CONTAINER_OF(timeout, struct k_deadline_reservation, timeout);
	k_spinlock_key_t key = z_sched_spinlock_lock();
	bool armed = (res->event != NULL) && (res->event->reservation == res);

	z_sched_spinlock_unlock(key);
	if (!armed) {
		return;
	}
	res->next_tick += res->period_ticks;
	(void)z_add_timeout(&res->timeout, periodic_release, K_TIMEOUT_ABS_TICKS(res->next_tick));
	/* The reservation sets the run's deadline and budget (z_deadline_admission_publication()). */
	k_deadline_event_post(res->event, 0);
}

static int arm(struct k_deadline_event *event, struct k_deadline_reservation *res, bool periodic,
	       uint32_t first)
{
	uint32_t now = k_cycle_get_32();
	struct admission_verdict verdict;
	struct sim_job candidate;
	bool ok;
	k_spinlock_key_t key;

	if (event->consumer == NULL || res->budget == 0U || res->min_interarrival == 0U ||
	    res->deadline > res->min_interarrival ||
	    (uint64_t)res->budget + res->isr_budget > res->min_interarrival) {
		/* A model that cannot hold: one run outstanding needs D <= T, and C + isr <= T. */
		return -EINVAL;
	}
	res->periodic = periodic;
	res->released = false;
	res->jitter = 0U;
	if (periodic) {
		res->period_ticks = k_cyc_to_ticks_floor64(res->min_interarrival);
		if (res->period_ticks == 0 ||
		    k_ticks_to_cyc_floor64(res->period_ticks) != res->min_interarrival) {
			return -EINVAL;
		}
		/* A release is announced up to one tick after its tick. */
		res->jitter = k_ticks_to_cyc_ceil32(1);
	}
	key = z_sched_spinlock_lock();
	if (event->reservation != NULL) {
		z_sched_spinlock_unlock(key);
		return -EINVAL;
	}
	uint32_t budget_before = event->budget;

	if (budget_before == 0U) {
		sys_slist_append(&admitted, &event->admission_node);
	}
	event->budget = res->budget;
	candidate = (struct sim_job){
		.event = event,
		.deadline = (int32_t)res->deadline,
		.work = res->budget,
		.prio = event->consumer->base.prio,
		.cooperative = event->consumer->base.prio < 0,
		.res = res,
	};
	ok = admission_check(now, &candidate, event, &verdict);
	if (ok) {
		res->event = event;
		event->reservation = res;
		sys_slist_append(&reservations, &res->node);
	} else {
		event->budget = budget_before;
		if (budget_before == 0U) {
			(void)sys_slist_find_and_remove(&admitted, &event->admission_node);
		}
	}
	z_sched_spinlock_unlock(key);
	if (!ok) {
		admission_report(&verdict, k_current_get());
		return -ETIME;
	}
	if (periodic) {
		/* The first release, then one every period from it (periodic_release()). */
		int32_t until = (int32_t)(first - k_cycle_get_32());

		res->next_tick = sys_clock_tick_get() +
				 (k_ticks_t)k_cyc_to_ticks_ceil64((uint64_t)MAX(until, 0));
		(void)z_add_timeout(&res->timeout, periodic_release,
				    K_TIMEOUT_ABS_TICKS(res->next_tick));
	}
	return 0;
}

int k_deadline_event_arm(struct k_deadline_event *event,
			 struct k_deadline_reservation *reservation)
{
	return arm(event, reservation, false, 0U);
}

int k_deadline_event_arm_periodic(struct k_deadline_event *event,
				  struct k_deadline_reservation *reservation, uint32_t first)
{
	return arm(event, reservation, true, first);
}

void k_deadline_event_disarm(struct k_deadline_event *event)
{
	k_spinlock_key_t key = z_sched_spinlock_lock();
	struct k_deadline_reservation *res = event->reservation;

	if (res != NULL) {
		(void)sys_slist_find_and_remove(&reservations, &res->node);
		event->reservation = NULL;
	}
	z_sched_spinlock_unlock(key);
	if (res != NULL && res->periodic) {
		(void)z_try_abort_timeout(&res->timeout);
	}
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
