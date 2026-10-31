/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/random/random.h>
#include <zephyr/kernel/deadline_miss.h>

#define NUM_THREADS 8
/* this should be large enough for us
 * to print a failing assert if necessary
 */
#define STACK_SIZE (512 + CONFIG_TEST_EXTRA_STACK_SIZE)

#define MSEC_TO_CYCLES(msec)  (int)(((uint64_t)(msec) * \
				     (uint64_t)sys_clock_hw_cycles_per_sec()) / \
				    (uint64_t)MSEC_PER_SEC)

struct k_thread worker_threads[NUM_THREADS];
volatile struct k_thread *expected_thread;
k_tid_t worker_tids[NUM_THREADS];

K_THREAD_STACK_ARRAY_DEFINE(worker_stacks, NUM_THREADS, STACK_SIZE);

int thread_deadlines[NUM_THREADS];

/* The number of worker threads that ran, and array of their
 * indices in execution order
 */
int n_exec;
int exec_order[NUM_THREADS];

struct k_thread release_thread;
K_THREAD_STACK_DEFINE(release_stack, STACK_SIZE);
volatile int release_runs;
struct k_thread release_sleep_thread;
K_THREAD_STACK_DEFINE(release_sleep_stack, STACK_SIZE);

#define RELEASE_ORDER_THREADS 2
struct k_thread release_order_threads[RELEASE_ORDER_THREADS];
K_THREAD_STACK_ARRAY_DEFINE(release_order_stacks, RELEASE_ORDER_THREADS,
				    STACK_SIZE);
volatile int release_order_runs;
int release_order[RELEASE_ORDER_THREADS];

struct release_order_args {
	k_tid_t later;
	k_tid_t earlier;
	int later_deadline;
	int earlier_deadline;
};

K_DEADLINE_EVENT_DEFINE(event_latch);
K_SEM_DEFINE(event_latch_armed, 0, 1);
K_SEM_DEFINE(event_latch_done, 0, 1);
struct k_thread event_latch_thread;
K_THREAD_STACK_DEFINE(event_latch_stack, STACK_SIZE);
volatile int event_latch_runs;

#define EVENT_PRIORITY_THREADS 2
struct k_deadline_event event_priority_events[EVENT_PRIORITY_THREADS];
K_SEM_DEFINE(event_priority_armed, 0, EVENT_PRIORITY_THREADS);
struct k_thread event_priority_threads[EVENT_PRIORITY_THREADS];
K_THREAD_STACK_ARRAY_DEFINE(event_priority_stacks, EVENT_PRIORITY_THREADS, STACK_SIZE);
volatile int event_priority_runs;
int event_priority_order[EVENT_PRIORITY_THREADS];

struct event_priority_post_args {
	struct k_deadline_event *lower;
	struct k_deadline_event *higher;
};

#define COALESCED_THREADS 2
struct k_deadline_event coalesced_events[COALESCED_THREADS];
K_SEM_DEFINE(coalesced_armed, 0, COALESCED_THREADS);
struct k_thread coalesced_threads[COALESCED_THREADS];
K_THREAD_STACK_ARRAY_DEFINE(coalesced_stacks, COALESCED_THREADS, STACK_SIZE);
volatile int coalesced_runs;
char coalesced_order[3];

static void coalesced_record(char run)
{
	zassert_true(coalesced_runs < (int)ARRAY_SIZE(coalesced_order),
		     "too many coalesced-deadline runs");
	coalesced_order[coalesced_runs++] = run;
}

static void coalesced_first_worker(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_sem_give(&coalesced_armed);
	k_deadline_event_wait(&coalesced_events[0]);
	coalesced_record('a');
	/* Latch this thread's next run, due after the job it then releases. */
	k_deadline_event_post(&coalesced_events[0], K_DEADLINE(MSEC_TO_CYCLES(30)));
	k_deadline_event_post(&coalesced_events[1], K_DEADLINE(MSEC_TO_CYCLES(20)));
	/* Consumes the latched run: it is due last, so the other job goes first. */
	k_deadline_event_wait(&coalesced_events[0]);
	coalesced_record('A');
	k_thread_suspend(k_current_get());
}

static void coalesced_second_worker(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_sem_give(&coalesced_armed);
	k_deadline_event_wait(&coalesced_events[1]);
	coalesced_record('b');
	k_thread_suspend(k_current_get());
}

#define LEVEL_THREADS 2
struct k_deadline_event level_events[LEVEL_THREADS];
K_SEM_DEFINE(level_armed, 0, LEVEL_THREADS);
struct k_thread level_threads[LEVEL_THREADS];
K_THREAD_STACK_ARRAY_DEFINE(level_stacks, LEVEL_THREADS, STACK_SIZE);
volatile int level_runs;
char level_order[3];
int level_inner_deadline;

static void level_record(char run)
{
	zassert_true(level_runs < (int)ARRAY_SIZE(level_order), "too many level runs");
	level_order[level_runs++] = run;
}

static void level_post_inner(const void *param)
{
	ARG_UNUSED(param);
	k_deadline_event_post(&level_events[1], level_inner_deadline);
}

/* Released first; releases the other job from an IRQ while it runs. */
static void level_outer_worker(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_sem_give(&level_armed);
	k_deadline_event_wait(&level_events[0]);
	level_record('o');
	irq_offload(level_post_inner, NULL);
	level_record('O');
	k_thread_suspend(k_current_get());
}

static void level_inner_worker(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_sem_give(&level_armed);
	k_deadline_event_wait(&level_events[1]);
	level_record('i');
	k_thread_suspend(k_current_get());
}

static void level_run(int outer_prio, int outer_deadline, int inner_prio, int inner_deadline)
{
	level_runs = 0;
	level_inner_deadline = inner_deadline;
	k_sem_reset(&level_armed);
	for (int i = 0; i < LEVEL_THREADS; i++) {
		k_deadline_event_init(&level_events[i]);
	}
	(void)k_thread_create(&level_threads[0], level_stacks[0], STACK_SIZE,
			      level_outer_worker, NULL, NULL, NULL, outer_prio, 0, K_NO_WAIT);
	(void)k_thread_create(&level_threads[1], level_stacks[1], STACK_SIZE,
			      level_inner_worker, NULL, NULL, NULL, inner_prio, 0, K_NO_WAIT);
	for (int i = 0; i < LEVEL_THREADS; i++) {
		k_sem_take(&level_armed, K_FOREVER);
	}
	k_deadline_event_post(&level_events[0], outer_deadline);
	k_sleep(K_MSEC(1));
	zassert_equal(level_runs, 3, "not every run happened");
	for (int i = 0; i < LEVEL_THREADS; i++) {
		k_thread_abort(&level_threads[i]);
	}
}

/* Missed deadlines reported through the custom policy. */
static struct k_deadline_miss last_miss;
static volatile int misses;

void k_deadline_miss_handler(const struct k_deadline_miss *miss)
{
	last_miss = *miss;
	misses++;
}

K_DEADLINE_EVENT_DEFINE(miss_event);
K_SEM_DEFINE(miss_armed, 0, 1);
struct k_thread miss_thread;
K_THREAD_STACK_DEFINE(miss_stack, STACK_SIZE);
volatile int miss_runs;
volatile bool miss_relatch;

/* Consumer below the test thread: a release runs once the test thread blocks. */
static void miss_worker(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_sem_give(&miss_armed);
	for (;;) {
		k_deadline_event_wait(&miss_event);
		miss_runs++;
		if (miss_relatch) {
			miss_relatch = false;
			/* Latched while running: an ordering key only, never checked. */
			k_deadline_event_post(&miss_event, K_DEADLINE(0));
			k_busy_wait(2000);
		}
	}
}

static void miss_start_consumer(void)
{
	misses = 0;
	miss_runs = 0;
	miss_relatch = false;
	k_deadline_event_init(&miss_event);
	k_sem_reset(&miss_armed);
	(void)k_thread_create(&miss_thread, miss_stack, STACK_SIZE, miss_worker, NULL, NULL,
			      NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
	k_sem_take(&miss_armed, K_FOREVER);
	/* Let the consumer reach its wait. */
	k_sleep(K_MSEC(1));
}


static void event_latch_post_twice(const void *param)
{
	struct k_deadline_event *event = (struct k_deadline_event *)param;

	/* Both publications occur before IRQ-exit scheduling and must coalesce. */
	k_deadline_event_post(event, K_DEADLINE(MSEC_TO_CYCLES(20)));
	k_deadline_event_post(event, K_DEADLINE(MSEC_TO_CYCLES(10)));
}

static void event_latch_post_then_reset(const void *param)
{
	struct k_deadline_event *event = (struct k_deadline_event *)param;

	/* One IRQ transaction proves reset wins before IRQ-exit scheduling. */
	k_deadline_event_post(event, K_DEADLINE(MSEC_TO_CYCLES(10)));
	k_deadline_event_reset(event);
}

static void event_priority_post(const void *param)
{
	const struct event_priority_post_args *args = param;

	/* Both releases occur before IRQ-exit scheduling. */
	k_deadline_event_post(args->lower, K_DEADLINE(MSEC_TO_CYCLES(10)));
	k_deadline_event_post(args->higher, K_DEADLINE(MSEC_TO_CYCLES(20)));
}

static void event_latch_worker(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/* Once main can run, this thread is atomically waiting/suspended. */
	k_sem_give(&event_latch_armed);
	k_deadline_event_wait(&event_latch);
	event_latch_runs++;

	/* Publish while the consumer is running, then consume without sleeping. */
	irq_offload(event_latch_post_twice, &event_latch);
	k_deadline_event_wait(&event_latch);
	event_latch_runs++;
	k_sem_give(&event_latch_done);

	/* Keep the persistent consumer alive but no longer runnable. */
	k_deadline_event_wait(&event_latch);
}

static void event_priority_worker(void *p1, void *p2, void *p3)
{
	int index = POINTER_TO_INT(p1);

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/* Tell the test this persistent consumer is purely suspended in its wait. */
	k_sem_give(&event_priority_armed);
	k_deadline_event_wait(&event_priority_events[index]);
	zassert_true(event_priority_runs < EVENT_PRIORITY_THREADS,
		     "too many deadline-event consumers ran");
	event_priority_order[event_priority_runs++] = index;
	k_thread_suspend(k_current_get());
}

static void release_worker(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	release_runs++;
	k_thread_suspend(k_current_get());
}

static void release_order_worker(void *p1, void *p2, void *p3)
{
	int index = POINTER_TO_INT(p1);

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	zassert_true(release_order_runs < RELEASE_ORDER_THREADS,
		     "too many released workers ran");
	release_order[release_order_runs++] = index;
	k_thread_suspend(k_current_get());
}

static void release_order_offload(const void *param)
{
	const struct release_order_args *args = param;

	/* Both transitions occur before interrupt-exit scheduling. */
	zassert_equal(k_thread_release_with_deadline(args->later,
					     args->later_deadline), 0,
		      "later-deadline thread was not released");
	zassert_equal(k_thread_release_with_deadline(args->earlier,
					     args->earlier_deadline), 0,
		      "earlier-deadline thread was not released");
}

/**
 * @brief Verify atomic deadline assignment and release of a persistent thread.
 *
 * A suspended thread is made eligible with a deadline and runs exactly once.
 * A second release fails rather than changing the state of an already-ready
 * thread, which also keeps timeout and object-wait release semantics outside
 * this API.
 */
ZTEST(suite_deadline, test_release_with_deadline)
{
	k_tid_t tid;
	k_tid_t sleeping_tid;

	release_runs = 0;
	tid = k_thread_create(&release_thread, release_stack, STACK_SIZE,
			      release_worker, NULL, NULL, NULL,
			      K_LOWEST_APPLICATION_THREAD_PRIO, 0,
			      K_NO_WAIT);
	k_thread_suspend(tid);

	zassert_equal(k_thread_release_with_deadline(tid,
					     K_DEADLINE(MSEC_TO_CYCLES(10))), 0,
		      "suspended thread was not released");
	zassert_equal(k_thread_release_with_deadline(tid,
					     K_DEADLINE(MSEC_TO_CYCLES(10))), -EALREADY,
		      "already-ready thread was released");

	/* Suspension must not turn a delayed thread into an eligible job. */
	sleeping_tid = k_thread_create(&release_sleep_thread, release_sleep_stack,
				      STACK_SIZE, release_worker, NULL, NULL, NULL,
				      K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_FOREVER);
	k_thread_suspend(sleeping_tid);
	zassert_equal(k_thread_release_with_deadline(sleeping_tid,
					     K_DEADLINE(MSEC_TO_CYCLES(10))), -EALREADY,
		      "sleeping thread was released");
	k_thread_abort(sleeping_tid);

	k_sleep(K_MSEC(10));
	zassert_equal(release_runs, 1, "released thread did not run once");
	k_thread_abort(tid);
}

/**
 * @brief Verify IRQ-released persistent threads retain EDF ordering.
 *
 * Two same-priority suspended threads are released during one IRQ-offload
 * invocation.  The release API only makes them ready; on IRQ exit the normal
 * scheduler must select the thread with the earlier absolute deadline first.
 */
ZTEST(suite_deadline, test_release_with_deadline_order)
{
	struct release_order_args args;

	release_order_runs = 0;
	args.later = k_thread_create(&release_order_threads[0],
				     release_order_stacks[0], STACK_SIZE,
				     release_order_worker, INT_TO_POINTER(0), NULL, NULL,
				     K_LOWEST_APPLICATION_THREAD_PRIO, 0,
				     K_NO_WAIT);
	args.earlier = k_thread_create(&release_order_threads[1],
				       release_order_stacks[1], STACK_SIZE,
				       release_order_worker, INT_TO_POINTER(1), NULL, NULL,
				       K_LOWEST_APPLICATION_THREAD_PRIO, 0,
				       K_NO_WAIT);
	k_thread_suspend(args.later);
	k_thread_suspend(args.earlier);

	/* K_DEADLINE produces absolute, distinct EDF ordering keys. */
	args.later_deadline = K_DEADLINE(MSEC_TO_CYCLES(20));
	args.earlier_deadline = K_DEADLINE(MSEC_TO_CYCLES(10));

	irq_offload(release_order_offload, &args);
	k_sleep(K_MSEC(10));

	zassert_equal(release_order_runs, RELEASE_ORDER_THREADS,
		      "not all released workers ran");
	zassert_equal(release_order[0], 1,
		      "later deadline ran before earlier deadline");
	zassert_equal(release_order[1], 0,
		      "later deadline did not run second");

	for (int i = 0; i < RELEASE_ORDER_THREADS; i++) {
		k_thread_abort(&release_order_threads[i]);
	}
}

/**
 * @brief Verify an IRQ-posted deadline event never loses a running-consumer wake.
 *
 * Two posts while the consumer is suspended coalesce into one consumption.  A
 * subsequent pair posted from an IRQ while that consumer is running remains
 * latched and is consumed immediately at its next wait, rather than being lost
 * because the thread was not suspended at publication time.
 */
ZTEST(suite_deadline, test_deadline_event_irq_latch)
{
	event_latch_runs = 0;
	k_deadline_event_init(&event_latch);
	k_sem_reset(&event_latch_armed);
	k_sem_reset(&event_latch_done);

	(void)k_thread_create(&event_latch_thread, event_latch_stack, STACK_SIZE,
			      event_latch_worker, NULL, NULL, NULL,
			      K_HIGHEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
	k_sem_take(&event_latch_armed, K_FOREVER);

	irq_offload(event_latch_post_twice, &event_latch);
	k_sem_take(&event_latch_done, K_FOREVER);

	zassert_equal(event_latch_runs, 2,
		      "coalesced IRQ publication was lost or consumed more than once");
	k_thread_abort(&event_latch_thread);
}

ZTEST(suite_deadline, test_deadline_event_reset_discards_pending)
{
	event_latch_runs = 0;
	k_deadline_event_init(&event_latch);
	k_sem_reset(&event_latch_armed);
	k_sem_reset(&event_latch_done);

	(void)k_thread_create(&event_latch_thread, event_latch_stack, STACK_SIZE,
			      event_latch_worker, NULL, NULL, NULL,
			      K_HIGHEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
	k_sem_take(&event_latch_armed, K_FOREVER);
	irq_offload(event_latch_post_then_reset, &event_latch);

	/* The job was readied but has no event to consume, so it re-suspends. */
	k_sleep(K_MSEC(1));
	zassert_equal(event_latch_runs, 0, "reset publication ran");
	k_thread_abort(&event_latch_thread);
}

/**
 * @brief Verify deadline-event releases retain static-priority ordering.
 *
 * The lower-priority consumer is released with the earlier start-by deadline,
 * while the higher-priority consumer gets a later one in the same IRQ
 * transaction. Static priority wins across priority levels; EDF orders only
 * consumers with equal static priority. These are run-queue ordering keys,
 * not release times or a guarantee that either deadline is met.
 */
ZTEST(suite_deadline, test_deadline_event_priority_before_deadline)
{
	struct event_priority_post_args args = {
		.lower = &event_priority_events[0],
		.higher = &event_priority_events[1],
	};

	event_priority_runs = 0;
	k_sem_reset(&event_priority_armed);
	for (int i = 0; i < EVENT_PRIORITY_THREADS; i++) {
		k_deadline_event_init(&event_priority_events[i]);
	}

	(void)k_thread_create(&event_priority_threads[0], event_priority_stacks[0],
			      STACK_SIZE, event_priority_worker, INT_TO_POINTER(0),
			      NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
	(void)k_thread_create(&event_priority_threads[1], event_priority_stacks[1],
			      STACK_SIZE, event_priority_worker, INT_TO_POINTER(1),
			      NULL, NULL, K_HIGHEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);

	for (int i = 0; i < EVENT_PRIORITY_THREADS; i++) {
		k_sem_take(&event_priority_armed, K_FOREVER);
	}
	irq_offload(event_priority_post, &args);
	k_sleep(K_MSEC(1));

	zassert_equal(event_priority_runs, EVENT_PRIORITY_THREADS,
		      "not all deadline-event consumers ran");
	zassert_equal(event_priority_order[0], 1,
		      "earlier lower-priority deadline displaced higher priority");
	zassert_equal(event_priority_order[1], 0,
		      "lower-priority consumer did not run second");

	for (int i = 0; i < EVENT_PRIORITY_THREADS; i++) {
		k_thread_abort(&event_priority_threads[i]);
	}
}

/**
 * @brief Verify a publication latched while its consumer runs keeps its deadline.
 *
 * A running consumer posts its own next run, due in 30 ms, then releases an
 * equal-priority job due in 20 ms, which cannot preempt it. Consuming the
 * latched run must apply that run's 30 ms deadline, not keep the key of the
 * run that just finished, so the 20 ms job runs first.
 */
ZTEST(suite_deadline, test_deadline_event_latched_run_keeps_its_deadline)
{
	static const k_thread_entry_t entries[COALESCED_THREADS] = {
		coalesced_first_worker, coalesced_second_worker,
	};

	coalesced_runs = 0;
	k_sem_reset(&coalesced_armed);
	for (int i = 0; i < COALESCED_THREADS; i++) {
		k_deadline_event_init(&coalesced_events[i]);
		(void)k_thread_create(&coalesced_threads[i], coalesced_stacks[i], STACK_SIZE,
				      entries[i], NULL, NULL, NULL,
				      K_HIGHEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
	}
	for (int i = 0; i < COALESCED_THREADS; i++) {
		k_sem_take(&coalesced_armed, K_FOREVER);
	}
	k_deadline_event_post(&coalesced_events[0], K_DEADLINE(MSEC_TO_CYCLES(10)));
	k_sleep(K_MSEC(1));

	zassert_equal(coalesced_runs, 3, "not every run happened");
	zassert_mem_equal(coalesced_order, "abA", 3,
			  "a latched run kept the earlier run's deadline");
	for (int i = 0; i < COALESCED_THREADS; i++) {
		k_thread_abort(&coalesced_threads[i]);
	}
}

/**
 * @brief Verify a higher meta-IRQ level preempts a running lower one.
 *
 * A job at the second meta-IRQ level runs and releases, from an IRQ, a job at
 * the first level whose deadline is later. The higher-priority job runs at
 * IRQ exit, before the lower one finishes: static priority decides across
 * levels, regardless of deadlines.
 */
ZTEST(suite_deadline, test_deadline_event_metairq_level_preempts)
{
	level_run(K_HIGHEST_THREAD_PRIO + 1, K_DEADLINE(MSEC_TO_CYCLES(10)),
		  K_HIGHEST_THREAD_PRIO, K_DEADLINE(MSEC_TO_CYCLES(30)));
	zassert_mem_equal(level_order, "oiO", 3,
			  "a higher meta-IRQ level did not preempt a lower one");
}

/**
 * @brief Verify meta-IRQ threads stay cooperative within their priority.
 *
 * A meta-IRQ thread preempts threads of lower priority only. A job at the
 * same meta-IRQ level released with an earlier deadline waits for the running
 * one to reach a scheduling point, like an interrupt of equal priority; so
 * does one with a later deadline.
 */
ZTEST(suite_deadline, test_deadline_event_metairq_same_level_cooperative)
{
	level_run(K_HIGHEST_THREAD_PRIO, K_DEADLINE(MSEC_TO_CYCLES(30)),
		  K_HIGHEST_THREAD_PRIO, K_DEADLINE(MSEC_TO_CYCLES(10)));
	zassert_mem_equal(level_order, "oOi", 3,
			  "an earlier deadline preempted within a meta-IRQ level");
	level_run(K_HIGHEST_THREAD_PRIO, K_DEADLINE(MSEC_TO_CYCLES(10)),
		  K_HIGHEST_THREAD_PRIO, K_DEADLINE(MSEC_TO_CYCLES(30)));
	zassert_mem_equal(level_order, "oOi", 3,
			  "a later deadline preempted within a meta-IRQ level");
}

/**
 * @brief Verify a job released from idle that starts late is reported.
 *
 * The test thread releases a lower-priority consumer with a 1 ms start-by
 * deadline, then keeps the CPU for 3 ms. The consumer starts after its
 * deadline, and the custom policy receives one late-start report naming the
 * event, its deadline, its release and the thread that ran at the release.
 */
ZTEST(suite_deadline, test_deadline_event_late_start_is_reported)
{
	int deadline;
	uint32_t before;

	miss_start_consumer();
	before = k_cycle_get_32();
	deadline = K_DEADLINE(MSEC_TO_CYCLES(1));
	k_deadline_event_post(&miss_event, deadline);
	k_busy_wait(3000);
	k_sleep(K_MSEC(1));

	zassert_equal(miss_runs, 1, "released consumer did not run once");
	zassert_equal(misses, 1, "late start was not reported once");
	zassert_equal(last_miss.kind, K_DEADLINE_MISS_START);
	zassert_equal_ptr(last_miss.object, &miss_event);
	zassert_str_equal(last_miss.name, "miss_event");
	zassert_equal_ptr(last_miss.thread, &miss_thread);
	zassert_equal(last_miss.deadline, deadline);
	zassert_true((int32_t)(last_miss.release - before) >= 0, "release precedes the post");
	zassert_true((int32_t)(last_miss.now - (uint32_t)deadline) > 0, "not late");
	zassert_equal_ptr(last_miss.running_at_release, k_current_get(),
			  "report does not name the thread that ran instead");
	k_thread_abort(&miss_thread);
}

/**
 * @brief Verify an on-time start and a run latched while busy are not reported.
 *
 * A release from idle that starts within its deadline is not a miss. A
 * publication the consumer posts while it runs is latched: it orders the next
 * run, which starts 2 ms after that publication's deadline, but it belongs to
 * the busy period that may already serve it, so it is not checked.
 */
ZTEST(suite_deadline, test_deadline_event_on_time_and_latched_not_reported)
{
	miss_start_consumer();
	miss_relatch = true;
	k_deadline_event_post(&miss_event, K_DEADLINE(MSEC_TO_CYCLES(10)));
	k_sleep(K_MSEC(10));

	zassert_equal(miss_runs, 2, "latched run did not happen");
	zassert_equal(misses, 0, "an on-time or latched run was reported");
	k_thread_abort(&miss_thread);
}


void worker(void *p1, void *p2, void *p3)
{
	int tidx = POINTER_TO_INT(p1);

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	zassert_true(tidx >= 0 && tidx < NUM_THREADS, "");
	zassert_true(n_exec >= 0 && n_exec < NUM_THREADS, "");

	exec_order[n_exec++] = tidx;

	/* Sleep, don't exit.  It's not implausible that some
	 * platforms implement a thread-based cleanup step for threads
	 * that exit (pthreads does this already) which might muck
	 * with the scheduling.
	 */
	while (1) {
		k_sleep(K_MSEC(1000000));
	}
}

/**
 * @brief Verify EDF threads run in earliest-deadline-first order.
 *
 * @details
 * At equal priority the scheduler must order runnable threads by their deadline
 * (set via k_thread_deadline_set()), running the earliest deadline first.
 * Several equal-priority threads are given random deadlines and must execute in
 * ascending-deadline order.
 *
 * Test steps:
 * - Create NUM_THREADS equal-priority threads, each recording its run order.
 * - Assign each a random deadline, then sleep to let them run.
 * - Verify their execution order matches ascending deadline order.
 *
 * Expected result:
 * - Threads execute earliest-deadline-first.
 *
 * @ingroup tests_kernel_sched
 *
 * @see k_thread_deadline_set()
 */
ZTEST(suite_deadline, test_deadline)
{
	int i;

	n_exec = 0;

	/* Create a bunch of threads at a single lower priority.  Give
	 * them each a random deadline.  Sleep, and check that they
	 * were executed in the right order.
	 */
	for (i = 0; i < NUM_THREADS; i++) {
		worker_tids[i] = k_thread_create(&worker_threads[i],
				worker_stacks[i], STACK_SIZE,
				worker, INT_TO_POINTER(i), NULL, NULL,
				K_LOWEST_APPLICATION_THREAD_PRIO,
				0, K_FOREVER);

		/* Positive-definite number with the bottom 8 bits
		 * masked off to prevent aliasing where "very close"
		 * deadlines end up in the opposite order due to the
		 * changing "now" between calls to
		 * k_thread_deadline_set().
		 *
		 * Use only 30 bits of significant value.  The API
		 * permits 31 (strictly: the deadline time of the
		 * "first" runnable thread in any given priority and
		 * the "last" must be less than 2^31), but because the
		 * time between our generation here and the set of the
		 * deadline below takes non-zero time, it's possible
		 * to see rollovers.  Easier than using a modulus test
		 * or whatnot to restrict the values.
		 */
		thread_deadlines[i] = sys_rand32_get() & 0x3fffff00;
	}

	/*
	 * Assign all worker deadlines before making the workers runnable.
	 * Updating already-queued workers could temporarily mix their initial
	 * zero deadlines with absolute cycle deadlines.
	 */
	for (i = 0; i < NUM_THREADS; i++) {
		k_thread_deadline_set(&worker_threads[i], thread_deadlines[i]);
	}

	for (i = 0; i < NUM_THREADS; i++) {
		k_thread_start(&worker_threads[i]);
	}

	k_sleep(K_MSEC(100));

	zassert_true(n_exec == NUM_THREADS, "not enough threads ran");

	for (i = 1; i < NUM_THREADS; i++) {
		int d0 = thread_deadlines[exec_order[i-1]];
		int d1 = thread_deadlines[exec_order[i]];

		zassert_true(d0 <= d1, "threads ran in wrong order");
	}
	for (i = 0; i < NUM_THREADS; i++) {
		k_thread_abort(worker_tids[i]);
	}
}

void yield_worker(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	zassert_true(n_exec >= 0 && n_exec < NUM_THREADS, "");

	n_exec += 1;

	k_yield();

	/* should not get here until all threads have started */
	zassert_true(n_exec == NUM_THREADS, "");

	k_thread_abort(k_current_get());

	CODE_UNREACHABLE;
}

/**
 * @brief Verify k_yield() rotates among equal deadline/priority threads.
 *
 * @details
 * Threads at the same priority and deadline (0) must each get to run when they
 * cooperatively yield. Each worker increments a counter and yields; all must
 * have run by the time the test wakes.
 *
 * Test steps:
 * - Create NUM_THREADS equal-priority/deadline threads that bump a counter and
 *   k_yield().
 * - Sleep and verify all threads ran.
 *
 * Expected result:
 * - Every thread runs, confirming yield rotates among equal peers.
 *
 * @ingroup tests_kernel_sched
 *
 * @see k_yield()
 */
ZTEST(suite_deadline, test_yield)
{
	/* Test that yield works across threads with the
	 * same deadline and priority. This currently works by
	 * simply not setting a deadline, which results in a
	 * deadline of 0.
	 */

	int i;

	n_exec = 0;

	/* Create a bunch of threads at a single lower priority
	 * and deadline.
	 * Each thread increments its own variable, then yields
	 * to the next. Sleep. Check that all threads ran.
	 */
	for (i = 0; i < NUM_THREADS; i++) {
		k_thread_create(&worker_threads[i],
				worker_stacks[i], STACK_SIZE,
				yield_worker, NULL, NULL, NULL,
				K_LOWEST_APPLICATION_THREAD_PRIO,
				0, K_NO_WAIT);
	}

	zassert_true(n_exec == 0, "threads ran too soon");

	k_sleep(K_MSEC(100));

	zassert_true(n_exec == NUM_THREADS, "not enough threads ran");
}

void unqueue_worker(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	zassert_true(n_exec >= 0 && n_exec < NUM_THREADS, "");

	n_exec += 1;
}

/**
 * @brief Verify k_thread_deadline_set() does not wake a not-yet-started thread.
 *
 * @details
 * Setting a deadline on a thread that is still in its start delay (unqueued)
 * must not make it run before its delay elapses. Threads are created with a
 * start delay, given deadlines while unqueued, and must stay dormant until the
 * delay passes.
 *
 * Test steps:
 * - Create NUM_THREADS threads with a start delay (unqueued).
 * - Set a random deadline on each and sleep less than the delay; none must run.
 * - Sleep past the delay and confirm all threads then run.
 *
 * Expected result:
 * - Deadline-set does not start an unqueued thread early; threads run only after
 *   their start delay.
 *
 * @ingroup tests_kernel_sched
 *
 * @see k_thread_deadline_set()
 */
ZTEST(suite_deadline, test_unqueued)
{
	int i;

	n_exec = 0;

	for (i = 0; i < NUM_THREADS; i++) {
		worker_tids[i] = k_thread_create(&worker_threads[i],
				worker_stacks[i], STACK_SIZE,
				unqueue_worker, NULL, NULL, NULL,
				K_LOWEST_APPLICATION_THREAD_PRIO,
				0, K_MSEC(100));
	}

	zassert_true(n_exec == 0, "threads ran too soon");

	for (i = 0; i < NUM_THREADS; i++) {
		thread_deadlines[i] = sys_rand32_get() & 0x3fffff00;
		k_thread_deadline_set(&worker_threads[i], thread_deadlines[i]);
	}

	k_sleep(K_MSEC(50));

	zassert_true(n_exec == 0, "deadline set make the unqueued thread run");

	k_sleep(K_MSEC(100));

	zassert_true(n_exec == NUM_THREADS, "not enough threads ran");

	for (i = 0; i < NUM_THREADS; i++) {
		k_thread_abort(worker_tids[i]);
	}
}

#if (CONFIG_MP_MAX_NUM_CPUS == 1)
static void reschedule_wrapper(const void *param)
{
	ARG_UNUSED(param);

	k_reschedule();
}

static void test_reschedule_helper0(void *p1, void *p2, void *p3)
{
	/* 4. Reschedule brings us here */

	zassert_true(expected_thread == _current, "");

	expected_thread = &worker_threads[1];
}

static void test_reschedule_helper1(void *p1, void *p2, void *p3)
{
	void (*offload)(void (*f)(const void *p), const void *param) = p1;

	/* 1. First helper expected to execute */

	zassert_true(expected_thread == _current, "");

	offload(reschedule_wrapper, NULL);

	/* 2. Deadlines have not changed. Expected no changes */

	zassert_true(expected_thread == _current, "");

	k_thread_deadline_set(_current, MSEC_TO_CYCLES(1000));

	/* 3. Deadline changed, but there was no reschedule */

	zassert_true(expected_thread == _current, "");

	expected_thread = &worker_threads[0];
	offload(reschedule_wrapper, NULL);

	/* 5. test_thread_reschedule_helper0 executed */

	zassert_true(expected_thread == _current, "");
}

static void thread_offload(void (*f)(const void *p), const void *param)
{
	f(param);
}

/**
 * @brief Verify a deadline change takes effect only on k_reschedule().
 *
 * @details
 * Changing a thread's deadline does not by itself trigger a context switch; an
 * explicit k_reschedule() must re-evaluate the ready queue and switch to the now
 * earliest-deadline thread. The test orchestrates two EDF helpers and checks the
 * expected thread is selected only after k_reschedule() (exercised via a direct
 * call and, on non-SMP, via irq_offload). Built for single-CPU configs only.
 *
 * Test steps:
 * - Create two EDF helper threads with different deadlines.
 * - Confirm changing a deadline alone causes no switch.
 * - Invoke k_reschedule() and confirm the expected thread is then selected.
 *
 * Expected result:
 * - The scheduler switches to the earliest-deadline thread only on reschedule.
 *
 * @ingroup tests_kernel_sched
 *
 * @see k_reschedule()
 * @see k_thread_deadline_set()
 */
ZTEST(suite_deadline, test_thread_reschedule)
{
	k_thread_create(&worker_threads[0], worker_stacks[0], STACK_SIZE,
			test_reschedule_helper0,
			thread_offload, NULL, NULL,
			K_LOWEST_APPLICATION_THREAD_PRIO,
			0, K_NO_WAIT);

	k_thread_create(&worker_threads[1], worker_stacks[1], STACK_SIZE,
			test_reschedule_helper1,
			thread_offload, NULL, NULL,
			K_LOWEST_APPLICATION_THREAD_PRIO,
			0, K_NO_WAIT);

	k_thread_deadline_set(&worker_threads[0], MSEC_TO_CYCLES(500));
	k_thread_deadline_set(&worker_threads[1], MSEC_TO_CYCLES(10));

	expected_thread = &worker_threads[1];

	k_thread_join(&worker_threads[1], K_FOREVER);
	k_thread_join(&worker_threads[0], K_FOREVER);

#ifndef CONFIG_SMP
	/*
	 * When SMP is enabled, there is always a reschedule performed
	 * at the end of the ISR.
	 */
	k_thread_create(&worker_threads[0], worker_stacks[0], STACK_SIZE,
			test_reschedule_helper0,
			irq_offload, NULL, NULL,
			K_LOWEST_APPLICATION_THREAD_PRIO,
			0, K_NO_WAIT);

	k_thread_create(&worker_threads[1], worker_stacks[1], STACK_SIZE,
			test_reschedule_helper1,
			irq_offload, NULL, NULL,
			K_LOWEST_APPLICATION_THREAD_PRIO,
			0, K_NO_WAIT);

	k_thread_deadline_set(&worker_threads[0], MSEC_TO_CYCLES(500));
	k_thread_deadline_set(&worker_threads[1], MSEC_TO_CYCLES(10));

	expected_thread = &worker_threads[1];

	k_thread_join(&worker_threads[1], K_FOREVER);
	k_thread_join(&worker_threads[0], K_FOREVER);

#endif /* !CONFIG_SMP */
}
#endif /* CONFIG_MP_MAX_NUM_CPUS == 1 */

ZTEST_SUITE(suite_deadline, NULL, NULL, NULL, NULL, NULL);
