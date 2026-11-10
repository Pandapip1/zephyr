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
/*
 * Time spent in interrupt handlers, outermost ones so each is counted once
 * (the entry checks, CONFIG_SCHED_DEADLINE_IRQ_BUDGETS; 0 without them).
 */
static uint32_t irq_time __maybe_unused;

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
	/* The jobs by deadline (dispatch rule), set at the first step of a run. */
	uint8_t order[CONFIG_SCHED_DEADLINE_ADMISSION_JOBS];
	bool sorted;
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

/*
 * The dispatch rule (docs/zephyr-dual-clock-scheduler.md, "Dispatch"): among
 * the orders that start every job by its deadline, run the highest priority
 * first; a lower priority goes first only when every such order needs it.
 *
 * Oracle: ordering by start-by deadline (EDF) starts every job in time if any
 * order does, so the set stays satisfiable after running candidate h first iff
 * h's cost fits the EDF slack of every job not yet started whose deadline is
 * earlier than h's (those with later deadlines already count h's work):
 *   S_k = d_k - t - sum of the costs of every other job with d_i <= d_k.
 * Jobs not yet released count as released now, which can only refuse more.
 */
struct guard_job {
	/* Deadline relative to now, and the CPU time the job still needs. */
	int32_t deadline;
	int32_t cost;
	int16_t prio;
	/* May be chosen now. */
	bool candidate;
	/* Has a start-by deadline still to meet. */
	bool binding;
};

/* Sort @p order (indices of @p g) by deadline. Insertion sort: n <= 12, often presorted. */
static __maybe_unused void guard_sort(const struct guard_job *g, uint8_t *order, size_t n)
{
	for (size_t i = 0U; i < n; i++) {
		order[i] = (uint8_t)i;
	}
	for (size_t i = 1U; i < n; i++) {
		uint8_t x = order[i];
		size_t j = i;

		while (j > 0U && g[order[j - 1U]].deadline - g[x].deadline > 0) {
			order[j] = order[j - 1U];
			j--;
		}
		order[j] = x;
	}
}

/*
 * The job the rule runs now, or -1 when none is a candidate. @p order lists
 * @p g by deadline; @p before is scratch of @p n entries. One sweep in
 * deadline order gives every binding job's slack and, for every job, the
 * least slack of the jobs with strictly earlier deadlines, which its cost
 * must fit.
 */
static __maybe_unused int guard_choose(const struct guard_job *g, size_t n, const uint8_t *order,
				       int32_t *before)
{
	int32_t sum = 0;
	int32_t least = INT32_MAX;
	int best = -1;
	int edf = -1;

	for (size_t i = 0U; i < n;) {
		size_t end = i;
		int32_t group = 0;
		int32_t group_least = INT32_MAX;

		/* A group of equal deadlines: each job counts the others of its group. */
		while (end < n && g[order[end]].deadline == g[order[i]].deadline) {
			group += g[order[end]].cost;
			end++;
		}
		for (size_t k = i; k < end; k++) {
			const struct guard_job *job = &g[order[k]];

			before[order[k]] = least;
			if (job->binding) {
				group_least = MIN(group_least,
						  job->deadline - (sum + group - job->cost));
			}
		}
		least = MIN(least, group_least);
		sum += group;
		i = end;
	}
	for (size_t h = 0U; h < n; h++) {
		if (!g[h].candidate) {
			continue;
		}
		if (edf < 0 || g[h].deadline - g[edf].deadline < 0 ||
		    (g[h].deadline == g[edf].deadline && g[h].prio < g[edf].prio)) {
			edf = (int)h;
		}
		bool fits = g[h].cost <= before[h];

		if (fits && (best < 0 || g[h].prio < g[best].prio ||
			     (g[h].prio == g[best].prio &&
			      g[h].deadline - g[best].deadline < 0))) {
			best = (int)h;
		}
	}
	/* No candidate keeps the set satisfiable first: the earliest deadline does if any can. */
	return (best >= 0) ? best : edf;
}

/* Under the rule, @p c may take the CPU from @p cur, which is running. */
static __maybe_unused bool sim_may_preempt(const struct sim_job *c, const struct sim_job *cur)
{
	if (c->prio != cur->prio) {
		/* Never a lower priority over a running job. */
		return c->prio < cur->prio;
	}
	return !cur->cooperative && c->deadline - cur->deadline < 0;
}

#ifndef CONFIG_SCHED_DEADLINE_DISPATCH
/*
 * Without the dispatch rule the scheduler is Zephyr's: the highest priority, the earliest
 * deadline within one; a running cooperative job keeps the CPU against its own priority.
 */
static int sim_choose(const struct sim *s, int32_t t, int current)
{
	int best = -1;

	for (size_t i = 0U; i < s->count; i++) {
		const struct sim_job *j = &s->jobs[i];

		if (j->res != NULL || j->remaining == 0U || j->release > t) {
			continue;
		}
		if (best < 0 || j->prio < s->jobs[best].prio ||
		    (j->prio == s->jobs[best].prio && j->deadline - s->jobs[best].deadline < 0)) {
			best = (int)i;
		}
	}
	if (current >= 0 && s->jobs[current].remaining != 0U && best >= 0 && best != current &&
	    s->jobs[current].cooperative && s->jobs[best].prio >= s->jobs[current].prio) {
		best = current;
	}
	return best;
}
#else
/* The job the rule runs at @p t in the simulation, or -1. */
static int sim_choose(struct sim *s, int32_t t, int current)
{
	static struct guard_job g[CONFIG_SCHED_DEADLINE_ADMISSION_JOBS];
	static int32_t slack[CONFIG_SCHED_DEADLINE_ADMISSION_JOBS];
	const struct sim_job *cur =
		(current >= 0 && s->jobs[current].remaining != 0U) ? &s->jobs[current] : NULL;

	for (size_t i = 0U; i < s->count; i++) {
		const struct sim_job *j = &s->jobs[i];
		int32_t spin = (!j->started && j->not_before > t) ? j->not_before - t : 0;

		g[i] = (struct guard_job){
			.deadline = j->deadline - t,
			.cost = (j->res == NULL) ? (int32_t)j->remaining + spin : 0,
			.prio = (int16_t)j->prio,
			.candidate = j->res == NULL && j->remaining != 0U && j->release <= t &&
				     (cur == NULL || j == cur || sim_may_preempt(j, cur)),
			.binding = j->res == NULL && j->remaining != 0U && !j->started &&
				   !j->ordering_only,
		};
	}
	if (!s->sorted) {
		/* Deadlines do not change during a check: one order for every step. */
		guard_sort(g, s->order, s->count);
		s->sorted = true;
	}
	return guard_choose(g, s->count, s->order, slack);
}
#endif

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
	s->sorted = false;
	sim_segment(s, 0, s->blocking, INT16_MIN);

	/* Each step finishes a job or reaches a release: bounded by twice the jobs. */
	for (size_t step = 0U; step < 2U * ARRAY_SIZE(s->jobs) + 2U; step++) {
		int32_t next_release = INT32_MAX;

		for (size_t i = 0U; i < s->count; i++) {
			struct sim_job *j = &s->jobs[i];

			if (j->remaining != 0U && j->release > t) {
				next_release = MIN(next_release, j->release);
			}
		}

		int best = sim_choose(s, t, current);

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
	/* An interrupt line's (no event) has no thread work, only its handler. */
	*j = (struct sim_job){
		.event = event,
		.deadline = (int32_t)res->deadline,
		.work = (event != NULL) ? res->budget : 0U,
		.prio = (event != NULL) ? event->consumer->base.prio : K_HIGHEST_THREAD_PRIO - 1,
		.cooperative = (event != NULL) && (event->consumer->base.prio < 0),
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
			/* Or chosen first by the dispatch rule for its earlier deadline. */
			blocks = !preempts(a, k) || k->deadline - a->deadline < 0;
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
		if (k->prio < a->prio || k->deadline - a->deadline <= 0) {
			/* Higher priority, or an earlier deadline the dispatch rule may run first. */
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

		if (j->ordering_only || j->running_at_check || j->late ||
		    (j->res != NULL && j->event == NULL)) {
			/* An interrupt line has no deadline of its own: only interference. */
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
static bool admission_check_run(uint32_t now, const struct sim_job *extra,
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

#ifdef CONFIG_SCHED_DEADLINE_ADMISSION_PROBE
struct k_deadline_admission_probe z_deadline_admission_probe;
#endif

static bool admission_check(uint32_t now, const struct sim_job *extra,
			    struct k_deadline_event *requester,
			    struct admission_verdict *verdict)
{
#ifdef CONFIG_SCHED_DEADLINE_ADMISSION_PROBE
	uint32_t start = k_cycle_get_32();
	bool ok = admission_check_run(now, extra, requester, verdict);
	uint32_t took = k_cycle_get_32() - start;

	z_deadline_admission_probe.check_max = MAX(z_deadline_admission_probe.check_max, took);
	z_deadline_admission_probe.check_count++;
	return ok;
#else
	return admission_check_run(now, extra, requester, verdict);
#endif
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

#ifdef CONFIG_SCHED_DEADLINE_BUDGET_ENFORCE
/*
 * Budget enforcement. A run's own time is its wall time less the time it
 * was switched out and the interrupt time while it ran. One timer, armed
 * whenever a budgeted run is switched in, expires at its remaining budget;
 * a run then found past its budget is reported once. A run switched out
 * leaves the timer armed: its expiry checks whatever runs then.
 *
 * The timer is a scheduler clock event when the service exists: it is
 * delivered at its target on the active provider (at least that provider's
 * minimum lead after it is armed), or reported itself as a late clock event
 * past the target plus CONFIG_CLOCK_EVENT_SCHED_LATE_TOLERANCE_NS. So an
 * overrun is caught within budget + minimum lead + that tolerance. Without
 * the service it is a kernel timeout, caught within budget + two ticks.
 */
#ifdef CONFIG_CLOCK_EVENT_SCHED
static struct k_clock_event budget_ce;
static bool budget_ce_bound;
#else
static struct _timeout budget_timeout;
#endif

/* The budgeted run in progress on @p thread, or NULL. */
static struct k_deadline_event *budget_run(struct k_thread *thread)
{
	struct k_deadline_event *event;

	SYS_SLIST_FOR_EACH_CONTAINER(&admitted, event, admission_node) {
		if (event->consumer == thread && event->admission_state == ADMIT_RUNNING) {
			return event;
		}
	}
	return NULL;
}

static uint32_t budget_used(const struct k_deadline_event *event, uint32_t now)
{
	return (now - event->run_start) - event->run_preempted - (irq_time - event->run_irq_mark);
}

static void budget_check(void);

#ifdef CONFIG_CLOCK_EVENT_SCHED
static void budget_ce_expired(struct k_clock_event *ce, uint32_t generation, void *user_data)
{
	ARG_UNUSED(ce);
	ARG_UNUSED(generation);
	ARG_UNUSED(user_data);
	budget_check();
}
#else
static void budget_timeout_expired(struct _timeout *timeout)
{
	ARG_UNUSED(timeout);
	budget_check();
}
#endif

static void budget_cancel(void)
{
#ifdef CONFIG_CLOCK_EVENT_SCHED
	if (budget_ce_bound) {
		k_clock_event_cancel(&budget_ce);
	}
#else
	(void)z_try_abort_timeout(&budget_timeout);
#endif
}

static void budget_arm(struct k_deadline_event *event, uint32_t now)
{
	uint32_t used = budget_used(event, now);
	uint32_t left = (used < event->run_budget) ? event->run_budget - used : 0U;

#ifdef CONFIG_CLOCK_EVENT_SCHED
	if (!budget_ce_bound) {
		budget_ce_bound = (k_clock_event_sched_init(&budget_ce, budget_ce_expired, NULL) == 0);
		__ASSERT(budget_ce_bound, "no scheduler clock-event slot for budget enforcement");
	}
	/* Inside the provider's minimum lead, the check comes at that lead. */
	uint32_t lead = k_clock_event_min_arm_cycles(&budget_ce);

	for (int attempt = 0; attempt < 2; attempt++) {
		if (k_clock_event_arm_abs(&budget_ce, now + MAX(left, lead)) == 0) {
			return;
		}
		/* The lead passed while arming: retry from the current time. */
		now = k_cycle_get_32();
		left = 0U;
	}
	__ASSERT(false, "budget clock event could not be armed");
#else
	(void)z_try_abort_timeout(&budget_timeout);
	/* A timeout never fires early; one more tick covers rounding to the tick. */
	(void)z_add_timeout(&budget_timeout, budget_timeout_expired,
			    Z_TIMEOUT_TICKS((k_ticks_t)k_cyc_to_ticks_ceil32(left) + 1));
#endif
}

static void budget_check(void)
{
	uint32_t now = k_cycle_get_32();
	k_spinlock_key_t key = z_sched_spinlock_lock();
	struct k_deadline_event *event = budget_run(_current);
	struct k_deadline_miss miss;
	bool over = false;

	if (event != NULL && !event->run_reported) {
		if (budget_used(event, now) >= event->run_budget) {
			event->run_reported = true;
			over = true;
			miss = (struct k_deadline_miss){
				.kind = K_DEADLINE_MISS_BUDGET,
				.object = event,
				.name = event->name,
				.thread = event->consumer,
				.release = event->run_start,
				.deadline = (int)(event->run_start + event->run_budget),
				.now = now,
			};
		} else {
			budget_arm(event, now);
		}
	}
	z_sched_spinlock_unlock(key);
	if (over) {
		z_deadline_miss_report(&miss);
	}
}

void z_deadline_budget_switched_out(void)
{
	struct k_deadline_event *event = budget_run(_current);

	if (event != NULL) {
		event->run_out_at = k_cycle_get_32();
	}
}

void z_deadline_budget_switched_in(void)
{
	struct k_deadline_event *event = budget_run(_current);

	if (event != NULL && !event->run_reported) {
		uint32_t now = k_cycle_get_32();

		event->run_preempted += now - event->run_out_at;
		budget_arm(event, now);
	}
}
#endif

#ifdef CONFIG_SCHED_DEADLINE_DISPATCH
/*
 * Runtime dispatch by the same rule as the simulation (guard_choose()). The
 * decision is taken at the scheduling points, where the job set changes (a
 * release, a run's start and end, an arm, a disarm), and cached: next_up()
 * reads it in constant time. Between scheduling points nothing changes the
 * set, and the guard assumes a chosen job runs its whole remaining budget.
 */
static struct k_thread *dispatch_choice;

static void dispatch_decide_run(uint32_t now)
{
	static struct guard_job g[CONFIG_SCHED_DEADLINE_ADMISSION_JOBS];
	static int32_t slack[CONFIG_SCHED_DEADLINE_ADMISSION_JOBS];
	static struct k_thread *owner[CONFIG_SCHED_DEADLINE_ADMISSION_JOBS];
	struct k_deadline_event *event;
	struct k_deadline_event *cur = NULL;
	size_t n = 0U;

	SYS_SLIST_FOR_EACH_CONTAINER(&admitted, event, admission_node) {
		if (event->consumer == _current && event->admission_state == ADMIT_RUNNING) {
			cur = event;
		}
	}
	SYS_SLIST_FOR_EACH_CONTAINER(&admitted, event, admission_node) {
		struct k_thread *consumer = event->consumer;
		bool running = event->admission_state == ADMIT_RUNNING;
		bool pending = event->admission_state == ADMIT_PENDING;

		if (consumer == NULL || n + 2U > ARRAY_SIZE(g)) {
			continue;
		}
		if (running || pending) {
			uint32_t used = (consumer == _current && running) ? now - event->run_start : 0U;
			uint32_t budget = running ? event->run_budget : event->pending_budget;
			int deadline = running ? consumer->base.prio_deadline : event->deadline;
			bool may = cur == NULL || event == cur ||
				   (consumer->base.prio != cur->consumer->base.prio ?
					    consumer->base.prio < cur->consumer->base.prio :
					    cur->consumer->base.prio >= 0 &&
						    deadline - cur->consumer->base.prio_deadline < 0);

			owner[n] = consumer;
			g[n++] = (struct guard_job){
				.deadline = (int32_t)((uint32_t)deadline - now),
				.cost = (int32_t)((used < budget) ? budget - used : 0U),
				.prio = (int16_t)consumer->base.prio,
				/* Ready, running, or about to be readied by the release being recorded. */
				.candidate = may && (z_is_thread_ready(consumer) || consumer == _current ||
						     (pending && consumer->base.thread_state ==
								 _THREAD_SUSPENDED)),
				.binding = pending,
			};
		}
		if (event->future || (event->reservation != NULL && !pending && !running)) {
			/* Releases still to come, as if released now: binding, not runnable. */
			bool armed = event->reservation != NULL;

			owner[n] = NULL;
			g[n++] = (struct guard_job){
				.deadline = armed ? (int32_t)event->reservation->deadline
						  : (int32_t)((uint32_t)event->future_deadline - now),
				.cost = (int32_t)(armed ? event->reservation->budget : event->budget),
				.prio = (int16_t)consumer->base.prio,
				.binding = true,
			};
		}
	}
	static uint8_t order[CONFIG_SCHED_DEADLINE_ADMISSION_JOBS];

	guard_sort(g, order, n);

	int best = guard_choose(g, n, order, slack);

	dispatch_choice = (best >= 0) ? owner[best] : NULL;
}

static void dispatch_decide(uint32_t now)
{
#ifdef CONFIG_SCHED_DEADLINE_ADMISSION_PROBE
	uint32_t start = k_cycle_get_32();

	dispatch_decide_run(now);

	uint32_t took = k_cycle_get_32() - start;

	z_deadline_admission_probe.decide_max = MAX(z_deadline_admission_probe.decide_max, took);
	z_deadline_admission_probe.decide_count++;
#else
	dispatch_decide_run(now);
#endif
}

/* An admitted consumer: the rule decides between these, Zephyr's priority between the rest. */
static bool dispatch_admitted(const struct k_thread *thread)
{
	struct k_deadline_event *event;

	SYS_SLIST_FOR_EACH_CONTAINER(&admitted, event, admission_node) {
		if (event->consumer == thread && (event->admission_state == ADMIT_RUNNING ||
						  event->admission_state == ADMIT_PENDING)) {
			return true;
		}
	}
	return false;
}

struct k_thread *z_deadline_dispatch(struct k_thread *best)
{
	struct k_thread *choice = dispatch_choice;

	if (choice == NULL || choice == best || best == NULL || !dispatch_admitted(best) ||
	    !(z_is_thread_ready(choice) || choice == _current)) {
		return best;
	}
	return choice;
}
#else
static inline void dispatch_decide(uint32_t now)
{
	ARG_UNUSED(now);
}
#endif

void z_deadline_admission_decide(uint32_t now)
{
	dispatch_decide(now);
}

void z_deadline_admission_started(struct k_deadline_event *event, uint32_t now, uint32_t budget)
{
	if (event->budget != 0U) {
		event->run_start = now;
		event->run_budget = budget;
		event->admission_state = ADMIT_RUNNING;
#ifdef CONFIG_SCHED_DEADLINE_BUDGET_ENFORCE
		event->run_preempted = 0U;
		event->run_irq_mark = irq_time;
		event->run_reported = false;
		budget_arm(event, now);
#endif
		dispatch_decide(now);
	}
}

void z_deadline_admission_finished(struct k_deadline_event *event)
{
	if (event->budget != 0U) {
		event->admission_state = event->pending ? ADMIT_PENDING : ADMIT_IDLE;
#ifdef CONFIG_SCHED_DEADLINE_BUDGET_ENFORCE
		budget_cancel();
#endif
		dispatch_decide(k_cycle_get_32());
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
	res->irq = -1;
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
		dispatch_decide(now);
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
		dispatch_decide(k_cycle_get_32());
	}
	z_sched_spinlock_unlock(key);
	if (res != NULL && res->periodic) {
		(void)z_try_abort_timeout(&res->timeout);
	}
}

/* The reservation of interrupt line @p irq, or NULL. Caller holds the scheduler lock or runs in
 * the line's own interrupt (reservations change only with interrupts locked).
 */
static struct k_deadline_reservation *irq_reservation(int32_t irq)
{
	struct k_deadline_reservation *res;

	SYS_SLIST_FOR_EACH_CONTAINER(&reservations, res, node) {
		if (res->event == NULL && res->irq == irq) {
			return res;
		}
	}
	return NULL;
}

int k_irq_reserve(unsigned int irq, struct k_deadline_reservation *reservation)
{
	uint32_t now = k_cycle_get_32();
	struct admission_verdict verdict;
	struct sim_job candidate;
	bool ok;
	k_spinlock_key_t key;

	if (reservation->isr_budget == 0U || reservation->min_interarrival == 0U ||
	    reservation->isr_budget > reservation->min_interarrival) {
		return -EINVAL;
	}
	reservation->event = NULL;
	reservation->irq = (int32_t)irq;
	reservation->released = false;
	reservation->periodic = false;
	reservation->jitter = 0U;
	reservation->deadline = reservation->min_interarrival;
	key = z_sched_spinlock_lock();
	if (irq_reservation((int32_t)irq) != NULL) {
		z_sched_spinlock_unlock(key);
		return -EINVAL;
	}
	candidate = (struct sim_job){
		.deadline = (int32_t)reservation->deadline,
		.prio = K_HIGHEST_THREAD_PRIO - 1,
		.res = reservation,
	};
	ok = admission_check(now, &candidate, NULL, &verdict);
	if (ok) {
		sys_slist_append(&reservations, &reservation->node);
	}
	z_sched_spinlock_unlock(key);
	if (!ok) {
		admission_report(&verdict, k_current_get());
		return -ETIME;
	}
	return 0;
}

void k_irq_unreserve(unsigned int irq)
{
	k_spinlock_key_t key = z_sched_spinlock_lock();
	struct k_deadline_reservation *res = irq_reservation((int32_t)irq);

	if (res != NULL) {
		(void)sys_slist_find_and_remove(&reservations, &res->node);
	}
	z_sched_spinlock_unlock(key);
}

#ifdef CONFIG_SCHED_DEADLINE_IRQ_BUDGETS
/*
 * Arrival and handler checks of the reserved interrupt lines, from the
 * interrupt entry (arch: _isr_wrapper). One frame per nesting level: a
 * handler's own time excludes the handlers that interrupted it.
 */
#define IRQ_NEST_MAX 8U

static struct {
	uint32_t start;
	uint32_t nested;
	struct k_deadline_reservation *res;
	int16_t irq;
} irq_nest[IRQ_NEST_MAX];
static uint8_t irq_depth;

#ifdef CONFIG_SCHED_DEADLINE_IRQ_PROBE
struct k_deadline_irq_probe z_deadline_irq_probe[CONFIG_NUM_IRQS];
/* Largest step back of k_cycle_get_32() between consecutive reads here, and where. */
uint32_t z_deadline_clock_backstep[3];

static void clock_monotonic(uint32_t now, uint32_t where)
{
	static uint32_t last;

	if ((int32_t)(now - last) < 0 && last - now > z_deadline_clock_backstep[0]) {
		z_deadline_clock_backstep[0] = last - now;
		z_deadline_clock_backstep[1] = where;
		z_deadline_clock_backstep[2] = now;
	}
	last = now;
}
#endif

static void irq_report(const struct k_deadline_reservation *res, uint32_t now)
{
	struct k_deadline_miss miss = {
		.kind = K_DEADLINE_MISS_OVERRUN,
		.object = res,
		.now = now,
		.release = now,
		.deadline = (int)now,
	};

	z_deadline_miss_report(&miss);
}

/*
 * The time is read and the nesting frame updated under one lock: a handler
 * nesting between the two would otherwise take the same frame, or be counted
 * in both its own and its parent's time. Reports are made after the lock.
 */
void z_deadline_irq_enter(int32_t irq)
{
	unsigned int key = arch_irq_lock();
	uint32_t now = k_cycle_get_32();
	struct k_deadline_reservation *res = irq_reservation(irq);
	bool report = false;

#ifdef CONFIG_SCHED_DEADLINE_IRQ_PROBE
	clock_monotonic(now, 0x10000U | (uint32_t)irq);
#endif

	if (res != NULL) {
		report = res->released && now - res->last_release < res->min_interarrival;
		res->released = true;
		res->last_release = now;
	} else {
		/* A line no admission check counts. */
		report = IS_ENABLED(CONFIG_SCHED_DEADLINE_IRQ_STRICT);
	}
#ifdef CONFIG_SCHED_DEADLINE_IRQ_PROBE
	if (irq >= 0 && irq < CONFIG_NUM_IRQS) {
		struct k_deadline_irq_probe *p = &z_deadline_irq_probe[irq];

		if (p->count != 0U) {
			p->min_gap = MIN(p->min_gap, now - p->last);
		} else {
			p->min_gap = UINT32_MAX;
		}
		p->last = now;
		p->count++;
	}
#endif
	if (irq_depth < IRQ_NEST_MAX) {
		irq_nest[irq_depth].start = now;
		irq_nest[irq_depth].nested = 0U;
		irq_nest[irq_depth].res = res;
		irq_nest[irq_depth].irq = (int16_t)irq;
	}
	irq_depth++;
	arch_irq_unlock(key);
	if (report) {
		irq_report(res, now);
	}
}

void z_deadline_irq_exit(void)
{
	unsigned int key = arch_irq_lock();
	uint32_t now = k_cycle_get_32();
	uint8_t depth = --irq_depth;
	struct k_deadline_reservation *res = NULL;
	bool report = false;

#ifdef CONFIG_SCHED_DEADLINE_IRQ_PROBE
	clock_monotonic(now, 0x20000U | ((depth < IRQ_NEST_MAX) ? (uint32_t)irq_nest[depth].irq
								 : 0xffffU));
#endif
	if (depth < IRQ_NEST_MAX) {
		uint32_t elapsed = now - irq_nest[depth].start;
		uint32_t own = elapsed - irq_nest[depth].nested;

		res = irq_nest[depth].res;
		if (depth > 0U) {
			irq_nest[depth - 1U].nested += elapsed;
		} else {
			irq_time += elapsed;
		}
#ifdef CONFIG_SCHED_DEADLINE_IRQ_PROBE
		int16_t irq = irq_nest[depth].irq;

		if (irq >= 0 && irq < CONFIG_NUM_IRQS) {
			z_deadline_irq_probe[irq].max_handler =
				MAX(z_deadline_irq_probe[irq].max_handler, own);
		}
#endif
		report = res != NULL && own > res->isr_budget;
	}
	arch_irq_unlock(key);
	if (report) {
		irq_report(res, now);
	}
}
#endif

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
