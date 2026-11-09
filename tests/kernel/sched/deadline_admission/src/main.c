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

/*
 * Time scale of the cases. Every time below is written for a target on which a
 * release reaches its consumer quickly; the tightest window around a job that
 * really runs is 100 us (a release due 100 us after it is posted). Getting from
 * a release to the consumer's first instruction (the releasing interrupt, the
 * admission checks in it, the context switch) is not counted by admission:
 * CONFIG_SCHED_DEADLINE_ADMISSION_BLOCKING_NS is 0 here, so the cases assume
 * that latency is small against their windows. It is not on every target: on
 * mps2/an385 under QEMU (-icount shift=7, 128 ns per instruction, 25 MHz cycle
 * counter) it is above 100 us, and admitted jobs start late. setup() measures
 * the latency on the target and multiplies every time in the cases by @ref
 * scale, keeping the latency within 1/LATENCY_SHARE of the tightest window.
 */
#define TIGHTEST_WINDOW_US 100U
#define LATENCY_SHARE      4U
#define CALIBRATION_RUNS   8

static uint32_t scale = 1U;

#define US(us) ((uint32_t)k_us_to_cyc_ceil32((uint64_t)(us) * scale))

static void sleep_ms(uint32_t ms)
{
	k_sleep(K_USEC(ms * USEC_PER_MSEC * scale));
}

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

/* What the policy was handed in each of the first two reports of a case. */
static const void *volatile reported_object[2];
static volatile uint8_t reported_ran_count[2];

/* Set by a case: the policy stops the thread it runs on, as a shutdown policy does. */
static volatile bool policy_suspends_reporter;
static struct k_thread *volatile suspended_reporter;
/* Set by a case: the policy's own work leads to another failure, found in an interrupt. */
static volatile bool policy_fails_in_irq;

static void report_unsat_in_irq(const void *param);

/* The first report since the case began: a late start may follow an unsat one. */
void k_deadline_miss_handler(const struct k_deadline_miss *miss)
{
	if (misses == 0) {
		last_miss = *miss;
	}
	if (misses < ARRAY_SIZE(reported_object)) {
		reported_object[misses] = miss->object;
		reported_ran_count[misses] = miss->ran_count;
	}
	misses++;
	if (policy_fails_in_irq) {
		policy_fails_in_irq = false;
		irq_offload(report_unsat_in_irq, NULL);
	}
	if (policy_suspends_reporter && !k_is_in_isr()) {
		policy_suspends_reporter = false;
		suspended_reporter = k_current_get();
		k_thread_suspend(k_current_get());
	}
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

static void budget(enum job_id id, uint32_t us);

/* Release-to-start time of the path most cases use; set by post_calibration(). */
static uint32_t calibration_release;

/* As post_with_budget(): two admitted releases in one interrupt, the second measured. */
static void post_calibration(const void *param)
{
	uint32_t now = k_cycle_get_32();

	ARG_UNUSED(param);
	calibration_release = now;
	k_deadline_event_post(&events[JOB_HIGH2], (int)(now + US(USEC_PER_SEC)));
	k_deadline_event_post(&events[JOB_HIGH], (int)(now + US(USEC_PER_SEC)));
}

/*
 * Measure, at scale 1, the longest time from an interrupt that releases two admitted jobs to
 * the start of the second, and set @ref scale from it.
 */
static void calibrate(void)
{
	uint32_t worst = 0U;
	uint32_t worst_us;

	budget(JOB_HIGH, 10U);
	budget(JOB_HIGH2, 10U);
	for (int i = 0; i < CALIBRATION_RUNS; i++) {
		int before_runs = runs[JOB_HIGH];

		irq_offload(post_calibration, NULL);
		sleep_ms(10U);
		zassert_equal(runs[JOB_HIGH], before_runs + 1, "calibration release did not run");
		worst = MAX(worst, started_at[JOB_HIGH] - calibration_release);
	}
	worst_us = (uint32_t)k_cyc_to_us_ceil32(worst);
	scale = MAX(1U, DIV_ROUND_UP(worst_us * LATENCY_SHARE, TIGHTEST_WINDOW_US));
	TC_PRINT("release to start %u us: times scaled by %u\n", worst_us, scale);
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
	sleep_ms(1U);
	calibrate();
	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	for (int i = 0; i < JOBS; i++) {
		k_deadline_event_disarm(&events[i]);
		k_deadline_event_budget_set(&events[i], 0U);
		run_us[i] = 0U;
		runs[i] = 0;
		k_deadline_event_admit_cancel(&events[i]);
	}
	misses = 0;
	reported_object[0] = NULL;
	reported_object[1] = NULL;
	policy_suspends_reporter = false;
	policy_fails_in_irq = false;
}

static void after(void *fixture)
{
	ARG_UNUSED(fixture);
	/* Let released jobs finish before the next case. */
	sleep_ms(5U);
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
	run_us[id] = us * scale;
	k_deadline_event_budget_set(&events[id], US(us));
}

/* A feasible release is admitted: no report, and it runs. */
ZTEST(deadline_admission, test_feasible_release_is_admitted)
{
	static const struct release r[] = {{JOB_HIGH, 1000U}};

	budget(JOB_HIGH, 100U);
	release_in_irq(r, ARRAY_SIZE(r));
	sleep_ms(2U);
	zassert_equal(runs[JOB_HIGH], 1);
	zassert_equal(misses, 0, "a feasible release was reported");
}

/*
 * An admitted job keeps its guarantee: a high-priority release whose work
 * leaves no room for an admitted low-priority one is the release that fails,
 * reported with the deadline and the demand of the job it would make late.
 */
ZTEST(deadline_admission, test_higher_priority_release_cannot_revoke_lower)
{
	static const struct release r[] = {{JOB_LOW, 300U}, {JOB_HIGH, 2000U}};

	budget(JOB_LOW, 50U);
	budget(JOB_HIGH, 500U);
	release_in_irq(r, ARRAY_SIZE(r));
	zassert_equal(misses, 1, "infeasibility was not reported at release");
	zassert_equal(last_miss.kind, K_DEADLINE_MISS_UNSAT);
	zassert_equal_ptr(last_miss.object, &events[JOB_HIGH], "the request is refused");
	zassert_true(last_miss.demand >= US(500U), "the request's work is ahead of the late job");
	sleep_ms(2U);
	zassert_equal(runs[JOB_HIGH], 0, "a refused release ran");
	zassert_equal(runs[JOB_LOW], 1, "the admitted job did not run");
	zassert_equal(misses, 1, "the admitted job started late");
}

/*
 * Within one priority the earliest deadline goes first: a release due sooner
 * whose work would push an admitted job of the same priority past its
 * deadline fails itself.
 */
ZTEST(deadline_admission, test_earlier_deadline_cannot_revoke_later_at_same_priority)
{
	static const struct release r[] = {{JOB_HIGH2, 600U}, {JOB_HIGH, 200U}};

	budget(JOB_HIGH2, 100U);
	budget(JOB_HIGH, 650U);
	release_in_irq(r, ARRAY_SIZE(r));
	zassert_equal(misses, 1);
	zassert_equal(last_miss.kind, K_DEADLINE_MISS_UNSAT);
	zassert_equal_ptr(last_miss.object, &events[JOB_HIGH]);
}

/*
 * Admitted work that is late without the request is what is reported: the
 * request did not cause it. Here a low-priority release is admitted, then a
 * same-priority job is made late by a budget raised behind the scheduler's
 * back (as an overrun would); the next request finds the set infeasible
 * without itself.
 */
ZTEST(deadline_admission, test_infeasible_admitted_work_is_reported_itself)
{
	uint32_t now = k_cycle_get_32();

	budget(JOB_LOW, 50U);
	budget(JOB_HIGH, 50U);
	zassert_equal(k_deadline_event_admit(&events[JOB_LOW], now + US(100U), now + US(100U),
					     (int)(now + US(300U))),
		      0);
	zassert_equal(k_deadline_event_admit(&events[JOB_HIGH], now + US(50U), now + US(50U),
					     (int)(now + US(5000U))),
		      0);
	/* Its admitted release now brings far more work than was admitted. */
	k_deadline_event_budget_set(&events[JOB_HIGH], US(1000U));
	zassert_equal(k_deadline_demand_admit(PRIO_LOW + 1, US(1U)), -ETIME);
	zassert_equal(misses, 1);
	zassert_equal_ptr(last_miss.object, &events[JOB_LOW], "the late admitted job");
	k_deadline_event_admit_cancel(&events[JOB_LOW]);
	k_deadline_event_admit_cancel(&events[JOB_HIGH]);
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
		k_busy_wait(800U * scale);
		inside_misses[1] = misses;
		irq_offload(post_high, NULL);
		inside_misses[1] = misses - inside_misses[1];
		k_busy_wait(200U * scale);
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
	sleep_ms(1U);

	budget(JOB_HIGH2, 1000U);
	budget(JOB_HIGH, 10U);
	k_deadline_event_post(&events[JOB_HIGH2], (int)(k_cycle_get_32() + US(100U)));
	sleep_ms(3U);
	zassert_equal(inside_misses[0], 1, "a full remaining budget ahead was not unsat");
	zassert_equal_ptr(last_miss.object, &events[JOB_HIGH]);
	k_deadline_event_post(&events[JOB_HIGH2], (int)(k_cycle_get_32() + US(100U)));
	sleep_ms(3U);
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
	sleep_ms(1U);
	zassert_equal(runs[JOB_HIGH], 1);
	zassert_true((int32_t)(started_at[JOB_HIGH] - not_before) >= 0, "started early");
	zassert_true((int32_t)(started_at[JOB_HIGH] - (not_before + US(100U))) <= 0,
		     "started after its deadline");
	zassert_equal(misses, 0);
}

/*
 * Waiting out an instant is demand at the waiter's priority. A wait at the
 * high priority that would make an admitted low-priority job start late is
 * refused: the low job keeps its guarantee.
 */
ZTEST(deadline_admission, test_spin_demand_cannot_revoke_admitted_lower_job)
{
	uint32_t now = k_cycle_get_32();

	budget(JOB_LOW, 50U);
	zassert_ok(k_deadline_event_admit(&events[JOB_LOW], now + US(100U), now + US(100U),
					  (int)(now + US(300U))));
	zassert_equal(misses, 0);
	zassert_equal(k_deadline_demand_admit(PRIO_HIGH, US(500U)), -ETIME);
	zassert_equal(misses, 1);
	zassert_is_null(last_miss.object, "the demand is refused, not the admitted job");
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
 * registered with a short budget, fails against an admitted same-priority job only when its
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
	sleep_ms(2U);
	zassert_equal(misses, 0, "a short publication was reported");

	budget_post_us = 650U;
	irq_offload(post_with_budget, NULL);
	zassert_true(misses >= 1, "the long publication's demand was not counted");
	zassert_equal(last_miss.kind, K_DEADLINE_MISS_UNSAT);
	zassert_equal_ptr(last_miss.object, &events[JOB_HIGH]);
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

/* Report @p id unsat from the calling context: a release that cannot start by its deadline. */
static void report_unsat(enum job_id id)
{
	uint32_t now = k_cycle_get_32();

	(void)k_deadline_event_admit(&events[id], now + US(500U), now + US(500U),
				     (int)(now + US(400U)));
}

static void report_unsat_in_irq(const void *param)
{
	ARG_UNUSED(param);
	report_unsat(JOB_LOW);
}

static struct k_thread reporter_thread;
static K_THREAD_STACK_DEFINE(reporter_stack, STACK_SIZE);
static volatile bool reporter_returned;

static void reporter(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	report_unsat(JOB_HIGH);
	reporter_returned = true;
}

/*
 * A policy that suspends the thread it was called on, inside the report, ends that report:
 * a failure found afterwards in another thread still reaches the policy, with its run history.
 */
ZTEST(deadline_admission, test_policy_suspending_its_reporter_sees_later_failures)
{
	budget(JOB_HIGH, 50U);
	budget(JOB_LOW, 50U);
	policy_suspends_reporter = true;
	reporter_returned = false;
	(void)k_thread_create(&reporter_thread, reporter_stack, STACK_SIZE, reporter, NULL, NULL,
			      NULL, K_PRIO_COOP(0), 0, K_NO_WAIT);
	sleep_ms(1U);
	zassert_equal(misses, 1);
	zassert_equal_ptr(suspended_reporter, &reporter_thread, "the policy ran on the reporter");
	zassert_false(reporter_returned);

	report_unsat(JOB_LOW);
	zassert_equal(misses, 2, "a failure after a suspended report did not reach the policy");
	zassert_equal_ptr(reported_object[1], &events[JOB_LOW]);

	/* Resumed, the first reporter finishes its report without ending a later one. */
	k_thread_resume(&reporter_thread);
	k_thread_join(&reporter_thread, K_FOREVER);
	zassert_true(reporter_returned);
	report_unsat(JOB_HIGH);
	zassert_equal(misses, 3, "a failure after a resumed reporter did not reach the policy");
	k_deadline_event_admit_cancel(&events[JOB_HIGH]);
	k_deadline_event_admit_cancel(&events[JOB_LOW]);
}

/*
 * A failure found while the policy handles another, here in an interrupt the policy's own
 * work raises, is nested: it reaches the policy too, without a run history.
 */
ZTEST(deadline_admission, test_failure_during_a_report_reaches_the_policy)
{
	budget(JOB_HIGH, 50U);
	budget(JOB_LOW, 50U);
	policy_fails_in_irq = true;
	report_unsat(JOB_HIGH);
	zassert_equal(misses, 2, "the nested failure did not reach the policy");
	zassert_equal_ptr(reported_object[0], &events[JOB_HIGH]);
	zassert_equal_ptr(reported_object[1], &events[JOB_LOW]);
	zassert_equal(reported_ran_count[1], 0U, "a nested report has no run history");

	/* The outer report has ended: the next failure owns the history again. */
	report_unsat(JOB_HIGH);
	zassert_equal(misses, 3);
	k_deadline_event_admit_cancel(&events[JOB_HIGH]);
	k_deadline_event_admit_cancel(&events[JOB_LOW]);
}

#define FILL_JOBS CONFIG_SCHED_DEADLINE_ADMISSION_JOBS

static struct k_deadline_event fill_events[FILL_JOBS];
static struct k_thread fill_threads[FILL_JOBS];
K_THREAD_STACK_ARRAY_DEFINE(fill_stacks, FILL_JOBS, STACK_SIZE);
static K_SEM_DEFINE(fill_armed, 0, FILL_JOBS);

static void fill_job(void *p1, void *p2, void *p3)
{
	struct k_deadline_event *event = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	k_sem_give(&fill_armed);
	for (;;) {
		k_deadline_event_wait(event);
	}
}

/*
 * A check holds at most CONFIG_SCHED_DEADLINE_ADMISSION_JOBS jobs. With the
 * table full of feasible timed releases, one more cannot be proved and is
 * refused, although the set would be feasible.
 */
ZTEST(deadline_admission, test_full_job_table_is_unsat)
{
	uint32_t now;

	for (int i = 0; i < FILL_JOBS; i++) {
		k_deadline_event_init(&fill_events[i]);
		(void)k_thread_create(&fill_threads[i], fill_stacks[i], STACK_SIZE, fill_job,
				      &fill_events[i], NULL, NULL, PRIO_LOW, 0, K_NO_WAIT);
	}
	for (int i = 0; i < FILL_JOBS; i++) {
		k_sem_take(&fill_armed, K_FOREVER);
	}
	sleep_ms(1U);
	budget(JOB_HIGH, 1U);
	now = k_cycle_get_32();
	for (int i = 0; i < FILL_JOBS; i++) {
		k_deadline_event_budget_set(&fill_events[i], US(1U));
		zassert_equal(k_deadline_event_admit(&fill_events[i], now + US(USEC_PER_SEC),
						     now + US(USEC_PER_SEC),
						     (int)(now + US(2U * USEC_PER_SEC))),
			      0, "a feasible release in a table with room was refused");
	}
	zassert_equal(misses, 0);
	zassert_equal(k_deadline_event_admit(&events[JOB_HIGH], now + US(USEC_PER_SEC),
					     now + US(USEC_PER_SEC),
					     (int)(now + US(2U * USEC_PER_SEC))),
		      -ETIME, "a release beyond the table was admitted unproved");
	zassert_equal(misses, 1);
	zassert_equal(last_miss.kind, K_DEADLINE_MISS_UNSAT);
	zassert_equal_ptr(last_miss.object, &events[JOB_HIGH], "the request is refused");
	k_deadline_event_admit_cancel(&events[JOB_HIGH]);
	for (int i = 0; i < FILL_JOBS; i++) {
		k_deadline_event_admit_cancel(&fill_events[i]);
		k_deadline_event_budget_set(&fill_events[i], 0U);
		k_thread_abort(&fill_threads[i]);
	}
}

static struct k_deadline_event latch_event;
static struct k_thread latch_thread;
static K_THREAD_STACK_DEFINE(latch_stack, STACK_SIZE);
static K_SEM_DEFINE(latch_armed, 0, 1);
static volatile int latch_misses;
static volatile int latch_admitted;

static void latch_job(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	k_sem_give(&latch_armed);
	for (;;) {
		uint32_t now;
		int before;

		k_deadline_event_wait(&latch_event);
		now = k_cycle_get_32();
		latch_admitted = k_deadline_event_admit(&events[JOB_LOW], now + US(100U),
							now + US(100U), (int)(now + US(300U)));
		before = misses;
		/* Latched behind this run, with work that leaves the low job no room. */
		k_deadline_event_post_budget(&latch_event, (int)(now + US(5000U)), US(1000U));
		latch_misses = misses - before;
		k_deadline_event_reset(&latch_event);
	}
}

static void post_latch(const void *param)
{
	ARG_UNUSED(param);
	k_deadline_event_post(&latch_event, (int)(k_cycle_get_32() + US(USEC_PER_SEC)));
}

/*
 * A publication latched while its consumer runs adds a run behind it. That
 * work is checked: here it would push an admitted lower-priority release past
 * its deadline, so the publication fails when it is made.
 */
ZTEST(deadline_admission, test_latched_publication_is_checked)
{
	k_deadline_event_init(&latch_event);
	latch_event.name = "latch";
	(void)k_thread_create(&latch_thread, latch_stack, STACK_SIZE, latch_job, NULL, NULL, NULL,
			      PRIO_HIGH, 0, K_NO_WAIT);
	k_sem_take(&latch_armed, K_FOREVER);
	sleep_ms(1U);
	budget(JOB_LOW, 50U);
	k_deadline_event_budget_set(&latch_event, US(50U));
	latch_admitted = -1;
	latch_misses = 0;
	irq_offload(post_latch, NULL);
	sleep_ms(2U);
	zassert_equal(latch_admitted, 0, "the low release was feasible before the latched work");
	zassert_equal(latch_misses, 1, "the latched publication was not checked");
	zassert_equal(last_miss.kind, K_DEADLINE_MISS_UNSAT);
	zassert_equal_ptr(last_miss.object, &latch_event, "the latched publication fails");
	k_deadline_event_admit_cancel(&events[JOB_LOW]);
	k_deadline_event_budget_set(&latch_event, 0U);
	k_thread_abort(&latch_thread);
}

/* Recurring jobs: armed (sporadic) and periodic reservations. */

static struct k_deadline_reservation res[JOBS];

static struct k_deadline_reservation *reserve(enum job_id id, uint32_t budget_us, uint32_t t_us,
					      uint32_t d_us)
{
	res[id] = (struct k_deadline_reservation){
		.budget = US(budget_us),
		.min_interarrival = US(t_us),
		.deadline = US(d_us),
	};
	run_us[id] = 0U;
	return &res[id];
}

static void activate(const void *param)
{
	k_deadline_event_post(&events[POINTER_TO_INT(param)], 0);
}

static void activate_twice(const void *param)
{
	k_deadline_event_post(&events[POINTER_TO_INT(param)], 0);
	k_deadline_event_post(&events[POINTER_TO_INT(param)], 0);
}

/* Arming is the point that can fail: a model that cannot hold against what is armed is refused. */
ZTEST(deadline_admission, test_arm_refused_when_infeasible)
{
	zassert_ok(k_deadline_event_arm(&events[JOB_HIGH], reserve(JOB_HIGH, 600U, 1000U, 1000U)));
	/* Released at the worst moment, the low job waits behind 600 us of the high one. */
	zassert_equal(k_deadline_event_arm(&events[JOB_LOW], reserve(JOB_LOW, 100U, 1000U, 300U)),
		      -ETIME);
	zassert_equal(misses, 1);
	zassert_equal(last_miss.kind, K_DEADLINE_MISS_UNSAT);
	zassert_equal_ptr(last_miss.object, &events[JOB_LOW]);
	/* Refused, it is not armed: its release would be checked like any other. */
	zassert_is_null(events[JOB_LOW].reservation);
}

/*
 * Activation of an armed job is never checked and never refused; work that would break its
 * reservation is refused instead, when it asks.
 */
ZTEST(deadline_admission, test_armed_activation_never_refused)
{
	uint32_t now;

	zassert_ok(k_deadline_event_arm(&events[JOB_HIGH], reserve(JOB_HIGH, 100U, 1000U, 500U)));
	budget(JOB_HIGH2, 800U);
	now = k_cycle_get_32();
	zassert_equal(k_deadline_event_admit(&events[JOB_HIGH2], now + US(100U), now + US(100U),
					     (int)(now + US(5000U))),
		      -ETIME, "work that would delay the armed job past its deadline was admitted");
	zassert_equal_ptr(last_miss.object, &events[JOB_HIGH2], "the request is refused");
	k_deadline_event_admit_cancel(&events[JOB_HIGH2]);
	misses = 0;

	irq_offload(activate, INT_TO_POINTER(JOB_HIGH));
	sleep_ms(2U);
	zassert_equal(runs[JOB_HIGH], 1, "the activation did not run");
	zassert_equal(misses, 0, "an activation was checked or refused");
}

/*
 * Finishing a run returns an armed job to armed without a check: under a load that refuses a
 * new arm, the armed job is activated again and again.
 */
ZTEST(deadline_admission, test_rearm_never_refused_under_load)
{
	zassert_ok(k_deadline_event_arm(&events[JOB_HIGH], reserve(JOB_HIGH, 200U, 1000U, 500U)));
	zassert_ok(k_deadline_event_arm(&events[JOB_HIGH2], reserve(JOB_HIGH2, 200U, 1000U, 500U)));
	/* The load: a third reservation no longer fits. */
	zassert_equal(k_deadline_event_arm(&events[JOB_LOW], reserve(JOB_LOW, 200U, 1000U, 350U)),
		      -ETIME);
	misses = 0;
	for (int i = 0; i < 5; i++) {
		irq_offload(activate, INT_TO_POINTER(JOB_HIGH));
		sleep_ms(2U);
	}
	zassert_equal(runs[JOB_HIGH], 5, "an armed job did not run on every activation");
	zassert_equal(misses, 0, "a re-arm was checked or refused");
}

/*
 * A periodic job is released by the kernel every period and goes back to sleep without a check,
 * under the same load.
 */
ZTEST(deadline_admission, test_periodic_resleep_never_refused_under_load)
{
	uint32_t period = k_ticks_to_cyc_floor32(k_us_to_ticks_ceil32(5000U * scale));
	int seen;

	res[JOB_HIGH2] = (struct k_deadline_reservation){
		.budget = US(50U),
		.min_interarrival = period,
		.deadline = period,
	};
	zassert_ok(k_deadline_event_arm(&events[JOB_HIGH], reserve(JOB_HIGH, 200U, 1000U, 500U)));
	zassert_ok(k_deadline_event_arm_periodic(&events[JOB_HIGH2], &res[JOB_HIGH2],
						 k_cycle_get_32() + period));
	/* Behind 200 us of the armed job and 50 us of the periodic one: 250 us, past 240. */
	zassert_equal(k_deadline_event_arm(&events[JOB_LOW], reserve(JOB_LOW, 200U, 1000U, 240U)),
		      -ETIME, "the load does not refuse a new arm");
	misses = 0;
	k_sleep(K_CYC(7U * period));
	seen = runs[JOB_HIGH2];
	zassert_true(seen >= 5, "periodic releases: %d in 7 periods", seen);
	zassert_equal(misses, 0, "a re-sleep was checked or refused");

	k_deadline_event_disarm(&events[JOB_HIGH2]);
	seen = runs[JOB_HIGH2];
	k_sleep(K_CYC(3U * period));
	zassert_equal(runs[JOB_HIGH2], seen, "a disarmed periodic job was released");
}

/* Disarming releases the capacity: an arm refused before it is admitted after. */
ZTEST(deadline_admission, test_disarm_releases_capacity)
{
	zassert_ok(k_deadline_event_arm(&events[JOB_HIGH], reserve(JOB_HIGH, 600U, 1000U, 1000U)));
	zassert_equal(k_deadline_event_arm(&events[JOB_LOW], reserve(JOB_LOW, 100U, 1000U, 300U)),
		      -ETIME);
	k_deadline_event_disarm(&events[JOB_HIGH]);
	misses = 0;
	zassert_ok(k_deadline_event_arm(&events[JOB_LOW], reserve(JOB_LOW, 100U, 1000U, 300U)));
	zassert_equal(misses, 0);
}

/*
 * An activation outside the declared model (here a second one while the first is still
 * outstanding, sooner than the minimum inter-arrival) is refused at its source and reported: it
 * is demand no reservation covers.
 */
ZTEST(deadline_admission, test_activation_outside_model_is_overrun)
{
	zassert_ok(k_deadline_event_arm(&events[JOB_HIGH], reserve(JOB_HIGH, 100U, 5000U, 500U)));
	irq_offload(activate_twice, INT_TO_POINTER(JOB_HIGH));
	sleep_ms(2U);
	zassert_equal(misses, 1);
	zassert_equal(last_miss.kind, K_DEADLINE_MISS_OVERRUN);
	zassert_equal_ptr(last_miss.object, &events[JOB_HIGH]);
	zassert_equal(runs[JOB_HIGH], 1, "the refused activation ran");
}

/*
 * Interference is bounded per busy period, not from the check: a release far ahead, after the CPU
 * has gone idle, waits for one release of the armed job, not for every release between the check
 * and its own (100 us, not 600 us, against a 150 us window). A window shorter than the one
 * release is still refused.
 */
ZTEST(deadline_admission, test_busy_period_bound_for_a_far_release)
{
	uint32_t now;

	zassert_ok(k_deadline_event_arm(&events[JOB_HIGH], reserve(JOB_HIGH, 100U, 1000U, 900U)));
	budget(JOB_LOW, 50U);
	now = k_cycle_get_32();
	zassert_ok(k_deadline_event_admit(&events[JOB_LOW], now + US(5000U), now + US(5000U),
					  (int)(now + US(5150U))),
		   "a far release was charged the interference of the whole wait");
	k_deadline_event_admit_cancel(&events[JOB_LOW]);
	zassert_equal(k_deadline_event_admit(&events[JOB_LOW], now + US(5000U), now + US(5000U),
					     (int)(now + US(5050U))),
		      -ETIME, "a window shorter than one release of the armed job was admitted");
	k_deadline_event_admit_cancel(&events[JOB_LOW]);
}

ZTEST_SUITE(deadline_admission, NULL, setup, before, after, NULL);
