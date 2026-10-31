/* SPDX-License-Identifier: Apache-2.0 */

/** @file @brief Deferred release of a deadline-event consumer. */

#ifndef ZEPHYR_INCLUDE_ZEPHYR_KERNEL_DEFERRED_RELEASE_H_
#define ZEPHYR_INCLUDE_ZEPHYR_KERNEL_DEFERRED_RELEASE_H_

#include <zephyr/kernel.h>
#include <zephyr/kernel/clock_event.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief One not-before release source for a deadline-event consumer.
 *
 * @ref not_before is an absolute cycle timestamp in the clock-event
 * provider's domain. @ref deadline is a separate absolute @ref k_cycle_get_32
 * start-by key, used only when the release is published. The two clock domains
 * must be the same for a caller to compare them, but this object never treats
 * @ref deadline as a release time.
 */
struct k_deferred_release {
	struct k_spinlock lock;
	struct k_clock_event *clock_event;
	struct k_deadline_event *event;
	int deadline;
	/* Release time of the armed publication; its run starts no earlier. */
	uint32_t not_before;
	/* The clock event is armed this long before not_before (wake path latency). */
	uint32_t lead;
	bool armed;
};

#define Z_DEFERRED_RELEASE_INITIALIZER \
	{ .event = NULL, .deadline = 0, .lead = 0U, .armed = false }
#define K_DEFERRED_RELEASE_DEFINE(name) \
	struct k_deferred_release name = Z_DEFERRED_RELEASE_INITIALIZER

int k_deferred_release_init(struct k_deferred_release *release,
			    struct k_deadline_event *event,
			    struct k_clock_event *clock_event);

/**
 * @brief Compensate the wake path's latency.
 *
 * Later posts arm their clock event @p lead cycles before @p not_before and
 * publish with k_deadline_event_post_at(): the released consumer waits out the
 * rest by spinning at its own priority and starts at @p not_before, provided
 * @p lead covers the worst-case latency from the clock event's expiry to the
 * consumer's dispatch. 0, the default, arms at @p not_before.
 *
 * @param release Deferred release.
 * @param lead Wake path latency to compensate, in the clock event's cycles.
 */
void k_deferred_release_lead_set(struct k_deferred_release *release, uint32_t lead);

/**
 * @brief Publish @p event no earlier than @p not_before.
 *
 * If @p not_before has passed, publication is immediate. Otherwise it is
 * armed as an absolute clock event. A @p not_before still ahead but inside the
 * provider's minimum arm lead is waited out by spinning on the provider's
 * clock, then published: never early and never dropped. The spin is bounded
 * by that lead. Replaces an older deferred publication that has not yet been
 * released.
 *
 * With CONFIG_SCHED_DEADLINE_ADMISSION and a clock event bound to the scheduler
 * clock service, the release is admitted first (k_deadline_event_admit()): the
 * service's delivery plan (k_clock_event_sched_plan()) bounds when it arrives,
 * and a caller that must wait it out counts that wait as demand at its own
 * priority.
 *
 * @retval 0 Armed or published.
 * @retval -ETIME Admission found the release infeasible and reported it; it is
 *         still armed or published.
 */
int k_deferred_release_post(struct k_deferred_release *release,
			    uint32_t not_before, int deadline);

/**
 * @brief Select a clock-event provider and publish @p event no earlier than
 *        @p not_before.
 *
 * The selected provider must satisfy @p required_power_states and have a
 * minimum arm lead no greater than @p max_min_arm_cycles. @p clock_event is
 * caller-owned storage used for the selected provider; this call replaces any
 * prior unreleased publication on @p release. It returns -ENODEV when no
 * registered provider meets those requirements.
 *
 * @p deadline is a @ref K_DEADLINE start-by key for the released consumer; it
 * is not a release time. @p not_before is the release time and remains in the
 * selected provider's cycle domain. Because @ref K_DEADLINE is expressed in
 * @ref k_cycle_get_32 units, selection is valid only among providers whose
 * cycle domain matches @ref k_cycle_get_32 when their timestamps are compared.
 * This helper does not impose a power-management policy.
 */
int k_deferred_release_post_selected(struct k_deferred_release *release,
				     struct k_clock_event *clock_event,
				     uint32_t not_before, int deadline,
				     uint32_t required_power_states,
				     uint32_t max_min_arm_cycles);

#if (defined(CONFIG_SCHED_DEADLINE_ADMISSION) && defined(CONFIG_CLOCK_EVENT_SCHED)) || \
	defined(__DOXYGEN__)
/**
 * @brief Admit a timed release delivered by the scheduler clock service.
 *
 * Combines a delivery plan (k_clock_event_sched_plan()) for the instant
 * @p arm_at the clock event will be armed for with admission control: the run
 * is released at @p arm_at and starts at @p not_before, later by the plan's
 * granularity (@ref k_clock_event_sched_plan.deliver_by beyond @p arm_at);
 * a wait the plan puts on the caller is demand at the caller's priority, one
 * it puts on an expiry interrupt is demand above every thread. Infeasibility
 * is reported to the missed-deadline policy as K_DEADLINE_MISS_UNSAT.
 *
 * The logical timebase and @ref k_cycle_get_32 must be one domain, as when
 * the service is the system timer.
 *
 * @param event Deadline event the expiry will publish.
 * @param plan Delivery plan for @p arm_at.
 * @param arm_at Instant the clock event is armed for.
 * @param not_before Earliest start of the run.
 * @param deadline Start-by deadline of the run.
 *
 * @retval 0 Feasible.
 * @retval -ETIME Infeasible; reported.
 */
int k_deadline_event_admit_plan(struct k_deadline_event *event,
				const struct k_clock_event_sched_plan *plan, uint32_t arm_at,
				uint32_t not_before, int deadline);
#endif

/** Cancel an unreleased deferred publication. Already-published work remains. */
void k_deferred_release_cancel(struct k_deferred_release *release);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ZEPHYR_KERNEL_DEFERRED_RELEASE_H_ */
