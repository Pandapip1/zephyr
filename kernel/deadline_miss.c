/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * One policy for every missed or infeasible scheduling deadline. Detection
 * stays where the deadline is known (deadline events, the scheduler clock
 * service, admission control); this file keeps the evidence for a report and
 * hands the failure to the configured policy.
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel/deadline_miss.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

static atomic_t miss_count;
static struct k_spinlock history_lock;
/* The most recent job runs, oldest overwritten first. */
static struct k_deadline_miss_run history[CONFIG_SCHED_DEADLINE_MISS_HISTORY];
static uint8_t history_head;

void z_deadline_miss_job_start(const struct k_deadline_event *event, uint32_t now)
{
	K_SPINLOCK(&history_lock) {
		history[history_head] = (struct k_deadline_miss_run){
			.event = event,
			.start = now,
		};
		history_head = (history_head + 1U) % ARRAY_SIZE(history);
	}
}

/* Snapshot of the runs that overlap a reported failure. */
static struct k_deadline_miss_run snapshot[CONFIG_SCHED_DEADLINE_MISS_HISTORY];
/* A report is being handled; a failure detected meanwhile is only counted. */
static atomic_t reporting;

/* Copy the runs that started in [release, now] into the snapshot, oldest first. */
static void history_fill(struct k_deadline_miss *miss)
{
	miss->ran_count = 0U;
	miss->ran = snapshot;
	K_SPINLOCK(&history_lock) {
		for (size_t i = 0U; i < ARRAY_SIZE(history); i++) {
			const struct k_deadline_miss_run *run =
				&history[(history_head + i) % ARRAY_SIZE(history)];

			if (run->event == NULL || (int32_t)(run->start - miss->release) < 0 ||
			    (int32_t)(miss->now - run->start) < 0) {
				continue;
			}
			snapshot[miss->ran_count++] = *run;
		}
	}
}

static const char *run_name(const struct k_deadline_miss_run *run)
{
#ifdef CONFIG_SCHED_DEADLINE
	return (run->event->name != NULL) ? run->event->name : "-";
#else
	ARG_UNUSED(run);
	return "-";
#endif
}

static const char *kind_name(enum k_deadline_miss_kind kind)
{
	switch (kind) {
	case K_DEADLINE_MISS_START:
		return "late start";
	case K_DEADLINE_MISS_DELIVERY:
		return "late clock event";
	case K_DEADLINE_MISS_UNSAT:
		return "unsat";
	default:
		return "?";
	}
}

static uint32_t cycles_to_ns(int32_t cycles)
{
	return (uint32_t)k_cyc_to_ns_floor64((uint64_t)MAX(cycles, 0));
}

void k_deadline_miss_print(const struct k_deadline_miss *miss)
{
	printk("deadline %s: %s %p thread %p: released %u, deadline %d, now %u: "
	       "%u ns past the deadline, %u ns after release\n",
	       kind_name(miss->kind), (miss->name != NULL) ? miss->name : "-", miss->object,
	       (void *)miss->thread, miss->release, miss->deadline, miss->now,
	       cycles_to_ns((int32_t)(miss->now - (uint32_t)miss->deadline)),
	       cycles_to_ns((int32_t)(miss->now - miss->release)));
	if (miss->kind == K_DEADLINE_MISS_UNSAT) {
		printk("  demand ahead of it: %u ns\n", cycles_to_ns((int32_t)miss->demand));
	}
	printk("  running at release: thread %p\n", (void *)miss->running_at_release);
	for (uint8_t i = 0U; i < miss->ran_count; i++) {
		const struct k_deadline_miss_run *run = &miss->ran[i];

		printk("  started instead: %s %p at %u, %u ns after release\n", run_name(run),
		       (const void *)run->event, run->start,
		       cycles_to_ns((int32_t)(run->start - miss->release)));
	}
}

uint32_t k_deadline_miss_count(void)
{
	return (uint32_t)atomic_get(&miss_count);
}

void z_deadline_miss_report(struct k_deadline_miss *miss)
{
	(void)atomic_inc(&miss_count);
	if (!atomic_cas(&reporting, 0, 1)) {
		return;
	}
	history_fill(miss);
	if (IS_ENABLED(CONFIG_SCHED_DEADLINE_MISS_PANIC)) {
		/* Print synchronously: the report must not depend on deferred logging. */
		LOG_PANIC();
		k_deadline_miss_print(miss);
		k_panic();
	}
#ifdef CONFIG_SCHED_DEADLINE_MISS_CUSTOM
	k_deadline_miss_handler(miss);
#endif
	atomic_set(&reporting, 0);
}
