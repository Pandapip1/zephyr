/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel/deadline_miss.h>
#include <zephyr/irq_offload.h>
#include <zephyr/ztest.h>

#define STACK_SIZE (1024 + CONFIG_TEST_EXTRA_STACK_SIZE)

/* Two meta-IRQ levels: the high job preempts the low one. */
#define PRIO_HIGH K_HIGHEST_THREAD_PRIO
#define PRIO_LOW  (K_HIGHEST_THREAD_PRIO + 1)

enum {
	JOB_HIGH,
	JOB_LOW,
	JOBS,
};

static struct k_deadline_event events[JOBS];
static struct k_thread threads[JOBS];
K_THREAD_STACK_ARRAY_DEFINE(stacks, JOBS, STACK_SIZE);
static K_SEM_DEFINE(armed, 0, JOBS);
static volatile uint32_t run_us[JOBS];
static volatile int runs[JOBS];

static struct k_deadline_miss last_miss;
static volatile int misses;

void k_deadline_miss_handler(const struct k_deadline_miss *miss)
{
	last_miss = *miss;
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
		runs[id]++;
		k_busy_wait(run_us[id]);
	}
}

static void *setup(void)
{
	static const int prios[JOBS] = {PRIO_HIGH, PRIO_LOW};

	for (int i = 0; i < JOBS; i++) {
		k_deadline_event_init(&events[i]);
		(void)k_thread_create(&threads[i], stacks[i], STACK_SIZE, job, INT_TO_POINTER(i),
				      NULL, NULL, prios[i], 0, K_NO_WAIT);
	}
	for (int i = 0; i < JOBS; i++) {
		k_sem_take(&armed, K_FOREVER);
	}
	k_msleep(1);
	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	for (int i = 0; i < JOBS; i++) {
		runs[i] = 0;
		run_us[i] = 0U;
	}
	misses = 0;
}

static void release(int id)
{
	k_deadline_event_post(&events[id], (int)(k_cycle_get_32() + k_ms_to_cyc_ceil32(100U)));
}

static void release_in_irq(const void *param)
{
	release(POINTER_TO_INT(param));
}

/* A run past its budget is reported once, with its event. */
ZTEST(deadline_budget, test_run_past_budget_reported)
{
	k_deadline_event_budget_set(&events[JOB_LOW], k_ms_to_cyc_ceil32(2U));
	run_us[JOB_LOW] = 6000U;
	irq_offload(release_in_irq, INT_TO_POINTER(JOB_LOW));
	k_msleep(20);
	zassert_equal(runs[JOB_LOW], 1);
	zassert_equal(misses, 1, "an overrun was not reported exactly once: %d, kind %d", misses,
		      last_miss.kind);
	zassert_equal(last_miss.kind, K_DEADLINE_MISS_BUDGET);
	zassert_equal_ptr(last_miss.object, &events[JOB_LOW]);
	/*
	 * Not preempted, so the run's own time is its wall time: the report comes within the
	 * detection latency of the budget's end (deadline.c, budget enforcement).
	 */
	uint32_t late = last_miss.now - (uint32_t)last_miss.deadline;
#if defined(CONFIG_CLOCK_EVENT_SCHED) && defined(CONFIG_COUNTER_SILABS_TIMER_CLOCK_EVENT_MIN_LEAD_NS)
	/* The active provider's minimum lead, then its delivery tolerance. */
	uint32_t bound = k_ns_to_cyc_ceil32(CONFIG_COUNTER_SILABS_TIMER_CLOCK_EVENT_MIN_LEAD_NS +
					    CONFIG_CLOCK_EVENT_SCHED_LATE_TOLERANCE_NS);
#else
	uint32_t bound = k_ticks_to_cyc_ceil32(2);
#endif
	TC_PRINT("budget overrun detected %u cycles after the budget (bound %u)\n", late, bound);
	zassert_true(late <= bound, "detected %u cycles late, bound %u", late, bound);
	k_deadline_event_budget_set(&events[JOB_LOW], 0U);
}

/* A run within its budget is not reported. */
ZTEST(deadline_budget, test_run_within_budget_not_reported)
{
	k_deadline_event_budget_set(&events[JOB_LOW], k_ms_to_cyc_ceil32(4U));
	run_us[JOB_LOW] = 2000U;
	irq_offload(release_in_irq, INT_TO_POINTER(JOB_LOW));
	k_msleep(20);
	zassert_equal(runs[JOB_LOW], 1);
	zassert_equal(misses, 0);
	k_deadline_event_budget_set(&events[JOB_LOW], 0U);
}

static struct k_timer preempt_timer;

static void preempt(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	release(JOB_HIGH);
}

/*
 * Time switched out is not the run's: a run of 2 ms against a 4 ms budget, preempted 1 ms in by
 * a 6 ms run of a higher priority, takes 8 ms of wall time and is not reported.
 */
ZTEST(deadline_budget, test_preemption_not_charged)
{
	k_deadline_event_budget_set(&events[JOB_LOW], k_ms_to_cyc_ceil32(4U));
	k_deadline_event_budget_set(&events[JOB_HIGH], k_ms_to_cyc_ceil32(8U));
	run_us[JOB_LOW] = 2000U;
	run_us[JOB_HIGH] = 6000U;
	k_timer_init(&preempt_timer, preempt, NULL);
	/* Started first: this thread does not run again until the low job is done. */
	k_timer_start(&preempt_timer, K_MSEC(1), K_NO_WAIT);
	irq_offload(release_in_irq, INT_TO_POINTER(JOB_LOW));
	k_msleep(30);
	zassert_equal(runs[JOB_LOW], 1);
	zassert_equal(runs[JOB_HIGH], 1);
	zassert_equal(misses, 0, "time preempted was charged to the run");
	k_deadline_event_budget_set(&events[JOB_LOW], 0U);
	k_deadline_event_budget_set(&events[JOB_HIGH], 0U);
}

ZTEST_SUITE(deadline_budget, NULL, setup, before, NULL, NULL);
