/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/kernel/deadline_miss.h>
#include <zephyr/kernel/deferred_release.h>
#include <zephyr/ztest.h>

#define STACK_SIZE (1024 + CONFIG_TEST_EXTRA_STACK_SIZE)

#define US(us) ((uint32_t)k_us_to_cyc_ceil32(us))

/* Two meta-IRQ levels, as a radio's jobs above an upper link layer. */
#define PRIO_HIGH K_HIGHEST_THREAD_PRIO
#define PRIO_LOW  (K_HIGHEST_THREAD_PRIO + 1)

enum job_id {
	JOB_HIGH,
	JOB_HIGH2,
	JOB_LOW,
	JOBS,
};

static struct k_deadline_event events[JOBS];
static struct k_thread threads[JOBS];
K_THREAD_STACK_ARRAY_DEFINE(stacks, JOBS, STACK_SIZE);
static K_SEM_DEFINE(armed, 0, JOBS);
/* Busy time of a run of each job. */
static volatile uint32_t run_us[JOBS];
/* Start of the latest run of each job, and run counts. */
static volatile uint32_t started_at[JOBS];
static volatile int runs[JOBS];

static struct k_deadline_miss last_miss;
static volatile int misses;

/* The first report since the case began: a late start may follow an unsat one. */
void k_deadline_miss_handler(const struct k_deadline_miss *miss)
{
	if (misses == 0) {
		last_miss = *miss;
	}
	misses++;
}

static void job(void *p1, void *p2, void *p3)
{
	int id = POINTER_TO_INT(p1);

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	k_sem_give(&armed);
	for (;;) {
		k_deadline_event_wait(&events[id]);
		started_at[id] = k_cycle_get_32();
		runs[id]++;
		k_busy_wait(run_us[id]);
	}
}

static void *setup(void)
{
	static const int prios[JOBS] = {PRIO_HIGH, PRIO_HIGH, PRIO_LOW};
	static const char *const names[JOBS] = {"high", "high2", "low"};

	for (int i = 0; i < JOBS; i++) {
		k_deadline_event_init(&events[i]);
		events[i].name = names[i];
		(void)k_thread_create(&threads[i], stacks[i], STACK_SIZE, job, INT_TO_POINTER(i),
				      NULL, NULL, prios[i], 0, K_NO_WAIT);
	}
	for (int i = 0; i < JOBS; i++) {
		k_sem_take(&armed, K_FOREVER);
	}
	/* Let every consumer reach its wait. */
	k_sleep(K_MSEC(1));
	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	for (int i = 0; i < JOBS; i++) {
		k_deadline_event_budget_set(&events[i], 0U);
		run_us[i] = 0U;
		runs[i] = 0;
		k_deadline_event_admit_cancel(&events[i]);
	}
	misses = 0;
}

static void after(void *fixture)
{
	ARG_UNUSED(fixture);
	/* Let released jobs finish before the next case. */
	k_sleep(K_MSEC(5));
}

/* Several releases in one interrupt: each is checked with the earlier ones pending. */
struct release {
	enum job_id id;
	uint32_t deadline_us;
};

static struct {
	const struct release *list;
	size_t count;
} releases;

static void post_releases(const void *param)
{
	uint32_t now = k_cycle_get_32();

	ARG_UNUSED(param);
	for (size_t i = 0U; i < releases.count; i++) {
		k_deadline_event_post(&events[releases.list[i].id],
				      (int)(now + US(releases.list[i].deadline_us)));
	}
}

static void release_in_irq(const struct release *list, size_t count)
{
	releases.list = list;
	releases.count = count;
	irq_offload(post_releases, NULL);
}

static void budget(enum job_id id, uint32_t us)
{
	run_us[id] = us;
	k_deadline_event_budget_set(&events[id], US(us));
}

/* A feasible release is admitted: no report, and it runs. */
ZTEST(deadline_admission, test_feasible_release_is_admitted)
{
	static const struct release r[] = {{JOB_HIGH, 1000U}};

	budget(JOB_HIGH, 100U);
	release_in_irq(r, ARRAY_SIZE(r));
	k_sleep(K_MSEC(2));
	zassert_equal(runs[JOB_HIGH], 1);
	zassert_equal(misses, 0, "a feasible release was reported");
}

/*
 * Priority decides: a high-priority release whose work leaves no room for an
 * admitted low-priority one is admitted, and the low-priority job, which can
 * no longer start by its deadline, is reported unsat before either runs.
 */
ZTEST(deadline_admission, test_higher_priority_release_sacrifices_lower)
{
	static const struct release r[] = {{JOB_LOW, 300U}, {JOB_HIGH, 2000U}};

	budget(JOB_LOW, 50U);
	budget(JOB_HIGH, 500U);
	release_in_irq(r, ARRAY_SIZE(r));
	zassert_true(misses >= 1, "infeasibility was not reported at release");
	zassert_equal(last_miss.kind, K_DEADLINE_MISS_UNSAT);
	zassert_equal_ptr(last_miss.object, &events[JOB_LOW], "the lower priority is sacrificed");
	zassert_true(last_miss.demand >= US(500U), "the higher job's work is ahead of it");
	k_sleep(K_MSEC(2));
	zassert_true((int32_t)(started_at[JOB_HIGH] - last_miss.now) >= 0,
		     "reported before the admitted job ran");
}

/*
 * Within one priority the earliest deadline goes first: a release due sooner
 * whose work pushes an admitted job of the same priority past its deadline
 * makes that later-deadline job unsat.
 */
ZTEST(deadline_admission, test_earlier_deadline_sacrifices_later_at_same_priority)
{
	static const struct release r[] = {{JOB_HIGH2, 600U}, {JOB_HIGH, 200U}};

	budget(JOB_HIGH2, 100U);
	budget(JOB_HIGH, 650U);
	release_in_irq(r, ARRAY_SIZE(r));
	zassert_true(misses >= 1);
	zassert_equal(last_miss.kind, K_DEADLINE_MISS_UNSAT);
	zassert_equal_ptr(last_miss.object, &events[JOB_HIGH2]);
}

/* A release that cannot start by its own deadline is unsat itself. */
ZTEST(deadline_admission, test_release_past_its_deadline_is_unsat)
{
	uint32_t now = k_cycle_get_32();

	budget(JOB_HIGH, 50U);
	zassert_equal(k_deadline_event_admit(&events[JOB_HIGH], now + US(500U), now + US(500U),
					     (int)(now + US(400U))),
		      -ETIME);
	zassert_equal(misses, 1);
	zassert_equal_ptr(last_miss.object, &events[JOB_HIGH]);
	k_deadline_event_admit_cancel(&events[JOB_HIGH]);
}

/*
 * A running cooperative job keeps the CPU against its own priority, and only
 * its remaining budget counts: a same-priority release due in 500 us is
 * infeasible behind a fresh 1000 us run, and feasible 800 us into it.
 */
static struct k_thread running_thread;
static K_THREAD_STACK_DEFINE(running_stack, STACK_SIZE);
static volatile bool post_inside;
static volatile int inside_misses[2];

static void post_high(const void *param)
{
	ARG_UNUSED(param);
	k_deadline_event_post(&events[JOB_HIGH], (int)(k_cycle_get_32() + US(500U)));
}

static void running_job(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	k_sem_give(&armed);
	for (;;) {
		k_deadline_event_wait(&events[JOB_HIGH2]);
		inside_misses[0] = misses;
		irq_offload(post_high, NULL);
		inside_misses[0] = misses - inside_misses[0];
		/* Let the released job run, then try again 800 us into a fresh run. */
		k_deadline_event_wait(&events[JOB_HIGH2]);
		k_busy_wait(800U);
		inside_misses[1] = misses;
		irq_offload(post_high, NULL);
		inside_misses[1] = misses - inside_misses[1];
		k_busy_wait(200U);
	}
}

ZTEST(deadline_admission, test_running_job_counts_its_remaining_budget)
{
	struct k_deadline_event saved = events[JOB_HIGH2];

	/* Replace the HIGH2 consumer with one that posts from inside its run. */
	k_thread_abort(&threads[JOB_HIGH2]);
	k_deadline_event_init(&events[JOB_HIGH2]);
	events[JOB_HIGH2].name = saved.name;
	(void)k_thread_create(&running_thread, running_stack, STACK_SIZE, running_job, NULL, NULL,
			      NULL,
			      PRIO_HIGH, 0, K_NO_WAIT);
	k_sem_take(&armed, K_FOREVER);
	k_sleep(K_MSEC(1));

	budget(JOB_HIGH2, 1000U);
	budget(JOB_HIGH, 10U);
	k_deadline_event_post(&events[JOB_HIGH2], (int)(k_cycle_get_32() + US(100U)));
	k_sleep(K_MSEC(3));
	zassert_equal(inside_misses[0], 1, "a full remaining budget ahead was not unsat");
	zassert_equal_ptr(last_miss.object, &events[JOB_HIGH]);
	k_deadline_event_post(&events[JOB_HIGH2], (int)(k_cycle_get_32() + US(100U)));
	k_sleep(K_MSEC(3));
	zassert_equal(inside_misses[1], 0, "the elapsed part of the run was counted");
	k_thread_abort(&running_thread);
}

/*
 * k_deadline_event_post_at(): a consumer released ahead of its instant waits
 * it out at its own priority and starts on time, inside its deadline.
 */
ZTEST(deadline_admission, test_post_at_starts_at_its_instant)
{
	uint32_t now = k_cycle_get_32();
	uint32_t not_before = now + US(300U);

	budget(JOB_HIGH, 10U);
	k_deadline_event_post_at(&events[JOB_HIGH], not_before, (int)(not_before + US(100U)));
	k_sleep(K_MSEC(1));
	zassert_equal(runs[JOB_HIGH], 1);
	zassert_true((int32_t)(started_at[JOB_HIGH] - not_before) >= 0, "started early");
	zassert_true((int32_t)(started_at[JOB_HIGH] - (not_before + US(100U))) <= 0,
		     "started after its deadline");
	zassert_equal(misses, 0);
}

/*
 * Waiting out an instant is demand at the waiter's priority. A wait at the
 * high priority that would make an admitted low-priority job start late is
 * resolved by priority: the low job is unsat.
 */
ZTEST(deadline_admission, test_spin_demand_sacrifices_admitted_lower_job)
{
	uint32_t now = k_cycle_get_32();

	budget(JOB_LOW, 50U);
	zassert_ok(k_deadline_event_admit(&events[JOB_LOW], now + US(100U), now + US(100U),
					  (int)(now + US(300U))));
	zassert_equal(misses, 0);
	zassert_equal(k_deadline_demand_admit(PRIO_HIGH, US(500U)), -ETIME);
	zassert_equal(misses, 1);
	zassert_equal_ptr(last_miss.object, &events[JOB_LOW]);
	k_deadline_event_admit_cancel(&events[JOB_LOW]);
}

/*
 * The wake ladder (k_clock_event_sched_plan() rungs, tested with the service
 * in tests/kernel/clock_event_sched) as admission sees it, for a release armed
 * 200 us ahead that must start within 20 us of its instant.
 */
static int admit_with(const struct k_clock_event_sched_plan *plan_rel, enum job_id id)
{
	uint32_t now = k_cycle_get_32();
	uint32_t arm_at = now + US(200U);
	struct k_clock_event_sched_plan plan = *plan_rel;

	/* The rungs' times are relative to the arm instant here. */
	plan.deliver_by += arm_at;
	return k_deadline_event_admit_plan(&events[id], &plan, arm_at, arm_at,
					   (int)(arm_at + US(20U)));
}

ZTEST(deadline_admission, test_ladder_lf_too_coarse_hf_exact)
{
	/* Deep-only delivery: within two LF cycles (61 us) of the instant. */
	const struct k_clock_event_sched_plan lf = {
		.rung = K_CLOCK_EVENT_SCHED_DEEP, .deliver_by = US(61U),
	};
	const struct k_clock_event_sched_plan hf = {
		.rung = K_CLOCK_EVENT_SCHED_ACTIVE, .deliver_by = 0U,
	};

	budget(JOB_HIGH, 10U);
	zassert_equal(admit_with(&lf, JOB_HIGH), -ETIME, "LF granularity misses the window");
	zassert_equal(misses, 1);
	zassert_ok(admit_with(&hf, JOB_HIGH), "a running HF clock delivers on time");
	zassert_equal(misses, 1);
	k_deadline_event_admit_cancel(&events[JOB_HIGH]);
}

ZTEST(deadline_admission, test_ladder_stopped_hf)
{
	/* An LF wake early enough to restart the HF clock: exact (two-stage). */
	const struct k_clock_event_sched_plan two_stage = {
		.rung = K_CLOCK_EVENT_SCHED_DEEP, .deliver_by = 0U,
	};
	/* Too little time to restart it: waited out on the LF clock. */
	const struct k_clock_event_sched_plan lf_spin = {
		.rung = K_CLOCK_EVENT_SCHED_SPIN, .deliver_by = US(31U), .caller_spin = US(200U),
	};

	budget(JOB_HIGH, 10U);
	zassert_ok(admit_with(&two_stage, JOB_HIGH));
	zassert_equal(admit_with(&lf_spin, JOB_HIGH), -ETIME, "no rung meets the window");
	zassert_equal(misses, 1);
	zassert_equal_ptr(last_miss.object, &events[JOB_HIGH]);
	k_deadline_event_admit_cancel(&events[JOB_HIGH]);
}

ZTEST(deadline_admission, test_ladder_inside_lead_rides_armed_wakeup)
{
	/* An expiry already armed 15 us before the instant waits out the rest. */
	const struct k_clock_event_sched_plan armed_rung = {
		.rung = K_CLOCK_EVENT_SCHED_ARMED, .deliver_by = 0U, .isr_spin = US(15U),
	};
	/* None armed, active clock stopped: an LF wait, too coarse for the window. */
	const struct k_clock_event_sched_plan none = {
		.rung = K_CLOCK_EVENT_SCHED_SPIN, .deliver_by = US(31U), .caller_spin = US(200U),
	};

	budget(JOB_HIGH, 10U);
	zassert_ok(admit_with(&armed_rung, JOB_HIGH), "the armed wakeup is in the window");
	zassert_equal(misses, 0);
	zassert_equal(admit_with(&none, JOB_HIGH), -ETIME);
	zassert_equal(misses, 1);
	k_deadline_event_admit_cancel(&events[JOB_HIGH]);
}

static const struct k_clock_event_sched_plan *spin_plan;
static int spin_ret;

/* In one interrupt: release the high job, then admit the spinning low release. */
static void busy_then_admit(const void *param)
{
	ARG_UNUSED(param);
	k_deadline_event_post(&events[JOB_HIGH], (int)(k_cycle_get_32() + US(2000U)));
	spin_ret = admit_with(spin_plan, JOB_LOW);
}

ZTEST(deadline_admission, test_ladder_spin_only)
{
	/* Inside every lead, nothing armed, HF running: the caller waits, exact to 1 us. */
	const struct k_clock_event_sched_plan spin = {
		.rung = K_CLOCK_EVENT_SCHED_SPIN, .deliver_by = US(1U), .caller_spin = US(200U),
	};

	budget(JOB_LOW, 10U);
	zassert_ok(admit_with(&spin, JOB_LOW), "a spin alone meets the window");
	zassert_equal(misses, 0);
	k_deadline_event_admit_cancel(&events[JOB_LOW]);

	/* A higher-priority job occupies the window: the spinning release is unsat. */
	budget(JOB_HIGH, 400U);
	spin_plan = &spin;
	irq_offload(busy_then_admit, NULL);
	zassert_equal(spin_ret, -ETIME);
	zassert_true(misses >= 1);
	zassert_equal_ptr(last_miss.object, &events[JOB_LOW]);
	k_deadline_event_admit_cancel(&events[JOB_LOW]);
}

/*
 * A publication carries its own budget (k_deadline_event_post_budget()): the same event,
 * registered with a short budget, makes an admitted same-priority job unsat only when its
 * publication declares the long one.
 */
static uint32_t budget_post_us;

static void post_with_budget(const void *param)
{
	uint32_t now = k_cycle_get_32();

	ARG_UNUSED(param);
	k_deadline_event_post(&events[JOB_HIGH2], (int)(now + US(600U)));
	k_deadline_event_post_budget(&events[JOB_HIGH], (int)(now + US(200U)), US(budget_post_us));
}

ZTEST(deadline_admission, test_publication_budget)
{
	budget(JOB_HIGH2, 100U);
	budget(JOB_HIGH, 50U);
	budget_post_us = 100U;
	irq_offload(post_with_budget, NULL);
	k_sleep(K_MSEC(2));
	zassert_equal(misses, 0, "a short publication was reported");

	budget_post_us = 650U;
	irq_offload(post_with_budget, NULL);
	zassert_true(misses >= 1, "the long publication's demand was not counted");
	zassert_equal(last_miss.kind, K_DEADLINE_MISS_UNSAT);
	zassert_equal_ptr(last_miss.object, &events[JOB_HIGH2]);
}

/*
 * An admitted timed release keeps its not-before bound: a job released early that waits to its
 * bound occupies its priority meanwhile, so a later check sees it as demand from the release.
 */
ZTEST(deadline_admission, test_future_release_keeps_not_before)
{
	uint32_t now = k_cycle_get_32();

	budget(JOB_HIGH, 10U);
	budget(JOB_HIGH2, 10U);
	/* Released at +100 us, may not start before +900 us: it holds the priority from +100. */
	zassert_equal(k_deadline_event_admit(&events[JOB_HIGH], now + US(100U), now + US(900U),
					     (int)(now + US(1000U))), 0);
	/* A same-priority job released at +200 us due by +400 us is behind that wait. */
	zassert_equal(k_deadline_event_admit(&events[JOB_HIGH2], now + US(200U), now + US(200U),
					     (int)(now + US(400U))), -ETIME);
	zassert_equal_ptr(last_miss.object, &events[JOB_HIGH2]);
	k_deadline_event_admit_cancel(&events[JOB_HIGH]);
	k_deadline_event_admit_cancel(&events[JOB_HIGH2]);
}

ZTEST_SUITE(deadline_admission, NULL, setup, before, after, NULL);
