/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Missed scheduling deadlines: report and policy hook.
 *
 * The kernel detects three kinds of deadline failure and reports each through
 * one policy, selected with CONFIG_SCHED_DEADLINE_MISS_POLICY:
 *
 * - A job released from idle through a deadline event started after its
 *   start-by deadline.
 * - A scheduler clock event was delivered later than its deadline plus
 *   CONFIG_CLOCK_EVENT_SCHED_LATE_TOLERANCE_NS.
 * - Admission control found, at release time, that a job cannot meet its
 *   deadline (CONFIG_SCHED_DEADLINE_ADMISSION).
 */

#ifndef ZEPHYR_INCLUDE_KERNEL_DEADLINE_MISS_H_
#define ZEPHYR_INCLUDE_KERNEL_DEADLINE_MISS_H_

#include <stdint.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Kind of deadline failure. */
enum k_deadline_miss_kind {
	/** A released job started after its start-by deadline. */
	K_DEADLINE_MISS_START,
	/** A scheduler clock event was delivered after its deadline and tolerance. */
	K_DEADLINE_MISS_DELIVERY,
	/** A release was found infeasible before it ran (admission control). */
	K_DEADLINE_MISS_UNSAT,
	/**
	 * An armed job was activated outside its declared arrival model: while an
	 * instance was still outstanding, or sooner than its minimum inter-arrival
	 * after the previous activation. The activation is refused.
	 */
	K_DEADLINE_MISS_OVERRUN,
};

/** One job run recorded for a report. */
struct k_deadline_miss_run {
	/** Deadline event whose consumer ran. */
	const struct k_deadline_event *event;
	/** Start of the run, in k_cycle_get_32() units. */
	uint32_t start;
};

/**
 * @brief Description of a missed or infeasible deadline.
 *
 * Times are in k_cycle_get_32() units; for a clock event they are the low
 * 32 bits of the scheduler clock's logical time, which is the same domain
 * when that service is the system timer.
 */
struct k_deadline_miss {
	enum k_deadline_miss_kind kind;
	/** The k_deadline_event, or the k_clock_event for a delivery miss. */
	const void *object;
	/** Name of the deadline event, or NULL. */
	const char *name;
	/** Consumer of the deadline event, or NULL. */
	struct k_thread *thread;
	/** When the job was released, or the clock event fell due. */
	uint32_t release;
	/** Start-by deadline (for a clock event, its deadline). */
	int deadline;
	/** When the failure was detected. */
	uint32_t now;
	/** Thread that was running when the job was released (what ran instead). */
	struct k_thread *running_at_release;
	/** For K_DEADLINE_MISS_UNSAT: work found ahead of the job, cycles. */
	uint32_t demand;
	/** Number of valid entries in @ref ran. */
	uint8_t ran_count;
	/**
	 * Deadline-event jobs that started between the release and @ref now,
	 * oldest first. Points to a kernel snapshot valid during the policy call.
	 */
	const struct k_deadline_miss_run *ran;
};

/**
 * @brief Policy for a missed or infeasible deadline (CONFIG_SCHED_DEADLINE_MISS_CUSTOM).
 *
 * Provided by the application or a subsystem. Called once per failure, from
 * the context that detected it: the job's own thread for a late start, the
 * clock provider's interrupt for a late delivery, and the releasing context
 * (often an interrupt) for an infeasible release. It must therefore be
 * interrupt-safe. It may return, in which case the job proceeds.
 *
 * Every failure reaches the policy, including one detected while an earlier
 * report is still being handled (by an interrupt that preempted the policy, or
 * by the policy itself); such a nested report has no run history
 * (@ref k_deadline_miss.ran_count is 0). The policy may suspend or abort the
 * thread it was called on, also inside the call: that report then ends, and
 * the run history it was given may be reused by the next one.
 *
 * @param miss Description of the failure; valid only during the call.
 */
void k_deadline_miss_handler(const struct k_deadline_miss *miss);

/**
 * @brief Print a report of @p miss with printk().
 *
 * @param miss Description of the failure.
 */
void k_deadline_miss_print(const struct k_deadline_miss *miss);

/**
 * @brief Number of deadline failures detected since boot.
 *
 * @return Count of reported failures of every kind.
 */
uint32_t k_deadline_miss_count(void);

/** @cond INTERNAL_HIDDEN */
/* Fill the run history of @p miss and apply the configured policy. */
void z_deadline_miss_report(struct k_deadline_miss *miss);
/* Record that the consumer of @p event started a run at @p now. */
void z_deadline_miss_job_start(const struct k_deadline_event *event, uint32_t now);
/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_KERNEL_DEADLINE_MISS_H_ */
