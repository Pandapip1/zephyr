/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Jobs (CONFIG_SCHED_DEADLINE_JOBS): the running job is tracked, a blocking call inside a run is
 * the job's next release, and a timed call with no job, or one no declaration covers, is
 * reported.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/irq_offload.h>
#include <zephyr/kernel/deadline_miss.h>
#include <deadline_admission.h>

#define STACK_SIZE (1024 + CONFIG_TEST_EXTRA_STACK_SIZE)
#define PRIO_JOB   2
#define PRIO_HIGH  1

#define US(us) ((uint32_t)k_us_to_cyc_ceil32(us))

enum mode {
	MODE_WAIT,
	/* Each run sleeps sleep_us, then runs again from the wake. */
	MODE_SLEEP,
	/* Each run records the current job, in the thread and in an interrupt. */
	MODE_PROBE,
	/* Each run starts a timer. */
	MODE_TIMER,
};

static K_DEADLINE_EVENT_DEFINE(job_event);
static K_DEADLINE_EVENT_DEFINE(plain_event);
static struct k_thread job_thread;
static struct k_thread plain_thread;
static struct k_thread high_thread;
static K_THREAD_STACK_DEFINE(job_stack, STACK_SIZE);
static K_THREAD_STACK_DEFINE(plain_stack, STACK_SIZE);
static K_THREAD_STACK_DEFINE(high_stack, STACK_SIZE);
static struct k_deadline_reservation reservation;

static volatile enum mode mode;
static volatile uint32_t sleep_us;
static volatile uint32_t slept_at;
static volatile uint32_t woke_at;
static volatile int runs;
static volatile int wakes;

static struct k_deadline_event *volatile seen_in_run;
static struct k_deadline_event *volatile seen_in_isr;
static struct k_deadline_event *volatile seen_by_high;
static struct k_deadline_event *volatile seen_after_preemption;
static struct k_deadline_event *volatile seen_in_expiry;
static volatile bool expired;
static K_SEM_DEFINE(high_go, 0, 1);

/* Reports for the objects these cases watch; other threads' are counted apart. */
static struct k_deadline_miss last;
static volatile int reports;
static volatile int unowned_reports;
static struct k_thread *volatile unowned_thread;

void k_deadline_miss_handler(const struct k_deadline_miss *miss)
{
	if (miss->kind == K_DEADLINE_MISS_UNDECLARED && miss->object == NULL) {
		unowned_reports++;
		unowned_thread = miss->thread;
		return;
	}
	last = *miss;
	reports++;
}

static void probe_isr(const void *param)
{
	ARG_UNUSED(param);
	seen_in_isr = z_deadline_current_job();
}

static void timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	seen_in_expiry = z_deadline_current_job();
	expired = true;
}

static K_TIMER_DEFINE(job_timer, timer_expiry, NULL);

static void job_body(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	for (;;) {
		k_deadline_event_wait(&job_event);
		runs++;
		switch (mode) {
		case MODE_SLEEP:
			slept_at = k_cycle_get_32();
			k_sleep(K_USEC(sleep_us));
			woke_at = k_cycle_get_32();
			wakes++;
			break;
		case MODE_PROBE:
			seen_in_run = z_deadline_current_job();
			irq_offload(probe_isr, NULL);
			k_sem_give(&high_go);
			/* The high thread preempts here, sees its own (no) job, and returns. */
			seen_after_preemption = z_deadline_current_job();
			break;
		case MODE_TIMER:
			k_timer_start(&job_timer, K_USEC(500), K_NO_WAIT);
			break;
		default:
			break;
		}
	}
}

static void plain_body(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	for (;;) {
		k_deadline_event_wait(&plain_event);
		k_sleep(K_USEC(1000));
	}
}

static void high_body(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	for (;;) {
		k_sem_take(&high_go, K_FOREVER);
		seen_by_high = z_deadline_current_job();
	}
}

static void arm(uint32_t t_us, uint32_t d_us)
{
	k_deadline_event_disarm(&job_event);
	reservation = (struct k_deadline_reservation){
		.budget = US(200U),
		.min_interarrival = US(t_us),
		.deadline = US(d_us),
	};
	zassert_ok(k_deadline_event_arm(&job_event, &reservation));
}

static void *setup(void)
{
	k_thread_create(&job_thread, job_stack, STACK_SIZE, job_body, NULL, NULL, NULL, PRIO_JOB,
			0, K_NO_WAIT);
	k_thread_create(&plain_thread, plain_stack, STACK_SIZE, plain_body, NULL, NULL, NULL,
			PRIO_JOB, 0, K_NO_WAIT);
	k_thread_create(&high_thread, high_stack, STACK_SIZE, high_body, NULL, NULL, NULL,
			PRIO_HIGH, 0, K_NO_WAIT);
	k_msleep(1);
	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	mode = MODE_WAIT;
	k_msleep(20);
	reports = 0;
	unowned_reports = 0;
	runs = 0;
	wakes = 0;
}

/* A sleep inside an armed job is its next release, at the sleep's end, started in time. */
ZTEST(deadline_jobs, test_sleep_in_job_releases_on_time)
{
	arm(5000U, 1000U);
	mode = MODE_SLEEP;
	sleep_us = 10000U;
	k_deadline_event_post(&job_event, 0);
	k_msleep(30);
	zassert_equal(wakes, 1, "the sleeping run did not wake once");
	zassert_true((int32_t)(woke_at - (slept_at + US(10000U))) >= 0, "woke early");
	zassert_true((int32_t)(woke_at - (slept_at + US(10000U))) <= (int32_t)US(1000U),
		     "woke %d cycles late, past its start-by",
		     (int32_t)(woke_at - (slept_at + US(10000U))));
	zassert_equal(reports, 0, "an on-time wake was reported");
}

/* The wake's lateness is checked against the job's start-by. */
ZTEST(deadline_jobs, test_late_wake_is_reported)
{
	arm(5000U, 500U);
	mode = MODE_SLEEP;
	sleep_us = 10000U;
	k_deadline_event_post(&job_event, 0);
	k_msleep(5);
	/* Hold the CPU across the wake and its start-by. */
	k_busy_wait(8000U);
	k_msleep(10);
	zassert_equal(wakes, 1);
	zassert_true(reports >= 1, "a late wake was not reported");
	zassert_equal(last.kind, K_DEADLINE_MISS_START);
	zassert_equal_ptr(last.object, &job_event);
}

/* A sleep shorter than the job's declared spacing is outside its model. */
ZTEST(deadline_jobs, test_wake_inside_declared_spacing_is_reported)
{
	arm(5000U, 1000U);
	mode = MODE_SLEEP;
	sleep_us = 1000U;
	k_deadline_event_post(&job_event, 0);
	k_msleep(10);
	zassert_equal(wakes, 1);
	zassert_true(reports >= 1, "a wake inside the declared spacing was not reported");
	zassert_equal(last.kind, K_DEADLINE_MISS_OVERRUN);
	zassert_equal_ptr(last.object, &job_event);
}

/* A job with no reservation has no declaration for a wake: its blocking call is reported. */
ZTEST(deadline_jobs, test_undeclared_blocking_call_in_job_is_reported)
{
	k_deadline_event_post(&plain_event, 0);
	k_msleep(5);
	zassert_true(reports >= 1, "an undeclared blocking call was not reported");
	zassert_equal(last.kind, K_DEADLINE_MISS_UNDECLARED);
	zassert_equal_ptr(last.object, &plain_event);
}

/* A timed call with no current job is reported, naming the thread. */
ZTEST(deadline_jobs, test_timed_call_outside_a_job_is_reported)
{
	k_msleep(1);
	k_msleep(1);
	zassert_true(unowned_reports >= 1, "a timed call outside a job was not reported");
	zassert_equal_ptr(unowned_thread, k_current_get());
}

/*
 * The current job is the running thread's: set in its run, none in an interrupt it takes, none
 * in a thread that preempts it, its own again when it resumes.
 */
ZTEST(deadline_jobs, test_current_job_tracks_switches_and_interrupts)
{
	arm(5000U, 5000U);
	seen_in_run = NULL;
	seen_in_isr = &plain_event;
	seen_by_high = &plain_event;
	seen_after_preemption = NULL;
	mode = MODE_PROBE;
	k_deadline_event_post(&job_event, 0);
	k_msleep(5);
	zassert_equal_ptr(seen_in_run, &job_event);
	zassert_is_null(seen_in_isr, "an interrupt inherited the job it interrupted");
	zassert_is_null(seen_by_high, "a preempting thread inherited the job");
	zassert_equal_ptr(seen_after_preemption, &job_event, "the job was lost over a switch");
	k_deadline_event_disarm(&job_event);
}

/* A timer started in a run expires as that job: no undeclared report. */
ZTEST(deadline_jobs, test_timer_expires_as_its_job)
{
	arm(5000U, 5000U);
	expired = false;
	seen_in_expiry = NULL;
	mode = MODE_TIMER;
	k_deadline_event_post(&job_event, 0);
	k_msleep(5);
	zassert_true(expired);
	zassert_equal_ptr(seen_in_expiry, &job_event);
}

ZTEST_SUITE(deadline_jobs, NULL, setup, before, NULL, NULL);
