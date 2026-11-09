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

#include <kthread.h>

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

/*
 * The report that owns the snapshot. It owns it until the policy returns, or
 * until the thread that made it is suspended or aborted: a policy commonly
 * stops the thread whose job missed, from inside the report, and that report
 * is then over although its call never returns. A report made while the owner
 * is live (from an interrupt that preempted it, from the policy itself, or
 * while the owner is preempted or blocked) is nested: it reaches the policy
 * too, without a run history.
 */
static struct k_spinlock report_lock;
static bool reporting;
/* Thread that made the owning report, or NULL for an interrupt. */
static struct k_thread *reporter;
/* Advanced by every owning report, so that an abandoned one cannot end its successor. */
static uint32_t report_generation;

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
	case K_DEADLINE_MISS_OVERRUN:
		return "activation outside its arrival model";
	default:
		return "?";
	}
}

static uint32_t cycles_to_ns(int32_t cycles)
{
	return (uint32_t)k_cyc_to_ns_floor64((uint64_t)MAX(cycles, 0));
}

/* A cycle difference read as signed, in ns. */
static int signed_ns(uint32_t diff)
{
	int32_t cycles = (int32_t)diff;
	uint64_t magnitude = (cycles < 0) ? (uint64_t)(-(int64_t)cycles) : (uint64_t)cycles;
	int64_t ns = (int64_t)k_cyc_to_ns_floor64(magnitude);

	return (int)((cycles < 0) ? -ns : ns);
}

void k_deadline_miss_print(const struct k_deadline_miss *miss)
{
	printk("deadline %s: %s %p thread %p: released %u, deadline %u, now %u: "
	       "%u ns past the deadline, %u ns after release\n",
	       kind_name(miss->kind), (miss->name != NULL) ? miss->name : "-", miss->object,
	       (void *)miss->thread, miss->release, (uint32_t)miss->deadline, miss->now,
	       cycles_to_ns((int32_t)(miss->now - (uint32_t)miss->deadline)),
	       cycles_to_ns((int32_t)(miss->now - miss->release)));
	if (miss->kind == K_DEADLINE_MISS_UNSAT) {
		/* Found before the job runs: its release and deadline may lie ahead. */
		printk("  release %d ns and deadline %d ns from the check, "
		       "demand ahead of it %u ns\n",
		       signed_ns(miss->release - miss->now),
		       signed_ns((uint32_t)miss->deadline - miss->now),
		       cycles_to_ns((int32_t)miss->demand));
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

/* The owning report's thread was stopped inside it and cannot finish it. */
static bool report_abandoned(void)
{
	return (reporter != NULL) && (z_is_thread_suspended(reporter) || z_is_thread_dead(reporter));
}

void z_deadline_miss_report(struct k_deadline_miss *miss)
{
	bool owner = false;
	uint32_t generation = 0U;

	(void)atomic_inc(&miss_count);
	K_SPINLOCK(&report_lock) {
		if (!reporting || report_abandoned()) {
			owner = true;
			reporting = true;
			reporter = k_is_in_isr() ? NULL : _current;
			generation = ++report_generation;
		}
	}
	if (owner) {
		history_fill(miss);
	} else {
		miss->ran_count = 0U;
		miss->ran = snapshot;
	}
	if (IS_ENABLED(CONFIG_SCHED_DEADLINE_MISS_PANIC)) {
		/* Print synchronously: the report must not depend on deferred logging. */
		LOG_PANIC();
		k_deadline_miss_print(miss);
		k_panic();
	}
#ifdef CONFIG_SCHED_DEADLINE_MISS_CUSTOM
	k_deadline_miss_handler(miss);
#endif
	if (owner) {
		K_SPINLOCK(&report_lock) {
			/* A resumed reporter whose report was taken over leaves the new one alone. */
			if (report_generation == generation) {
				reporting = false;
			}
		}
	}
}
