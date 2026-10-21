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
	bool armed;
};

#define Z_DEFERRED_RELEASE_INITIALIZER \
	{ .event = NULL, .deadline = 0, .armed = false }
#define K_DEFERRED_RELEASE_DEFINE(name) \
	struct k_deferred_release name = Z_DEFERRED_RELEASE_INITIALIZER

int k_deferred_release_init(struct k_deferred_release *release,
			    struct k_deadline_event *event,
			    struct k_clock_event *clock_event);

/**
 * @brief Publish @p event no earlier than @p not_before.
 *
 * If @p not_before has passed, publication is immediate. Otherwise it is
 * armed as an absolute clock event. An arm that the provider cannot complete
 * reliably returns -ETIME and does not publish early. Replaces an older
 * deferred publication that has not yet been released.
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

/** Cancel an unreleased deferred publication. Already-published work remains. */
void k_deferred_release_cancel(struct k_deferred_release *release);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ZEPHYR_KERNEL_DEFERRED_RELEASE_H_ */
