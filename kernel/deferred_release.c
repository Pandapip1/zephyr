/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/kernel/deferred_release.h>

static void deferred_release_fire(struct k_clock_event *clock_event, uint32_t generation,
				  void *user_data)
{
	struct k_deferred_release *release = user_data;
	struct k_deadline_event *event = NULL;
	int deadline = 0;

	ARG_UNUSED(clock_event);
	ARG_UNUSED(generation);
	K_SPINLOCK(&release->lock) {
		if (release->armed) {
			release->armed = false;
			event = release->event;
			deadline = release->deadline;
		}
	}
	if (event != NULL) {
		k_deadline_event_post(event, deadline);
	}
}

/*
 * A release time inside the provider's minimum arm lead cannot be armed, but
 * it is still a release time: wait it out on the provider's own clock rather
 * than drop the release or publish it early. The wait is bounded by that lead.
 */
static void deferred_release_wait_until(struct k_clock_event *clock_event, uint32_t not_before)
{
	while ((int32_t)(not_before - k_clock_event_now(clock_event)) > 0) {
		arch_nop();
	}
}

int k_deferred_release_init(struct k_deferred_release *release,
			    struct k_deadline_event *event,
			    struct k_clock_event *clock_event)
{
	if (release == NULL || event == NULL || clock_event == NULL) {
		return -EINVAL;
	}
	release->event = event;
	release->clock_event = clock_event;
	release->deadline = 0;
	release->armed = false;
	return k_clock_event_set_handler(clock_event, deferred_release_fire, release);
}

int k_deferred_release_post(struct k_deferred_release *release,
			    uint32_t not_before, int deadline)
{
	struct k_clock_event *clock_event;
	struct k_deadline_event *event = NULL;
	uint32_t now;
	bool wait = false;
	int ret = 0;

	if (release == NULL || release->event == NULL) {
		return -EINVAL;
	}
	clock_event = release->clock_event;

	K_SPINLOCK(&release->lock) {
		/* Replace any prior deferred release before publishing a new one. */
		k_clock_event_cancel(clock_event);
		now = k_clock_event_now(clock_event);
		if ((int32_t)(not_before - now) <= 0) {
			release->armed = false;
			event = release->event;
		} else {
			release->deadline = deadline;
			release->armed = true;
			ret = k_clock_event_arm_abs(clock_event, not_before);
			if (ret == -ETIME) {
				/* Inside the arm lead: wait it out after unlocking. */
				release->armed = false;
				wait = true;
				ret = 0;
			} else if (ret != 0) {
				release->armed = false;
			}
		}
	}
	if (wait) {
		deferred_release_wait_until(clock_event, not_before);
		k_deadline_event_post(release->event, deadline);
	}
	if (event != NULL) {
		k_deadline_event_post(event, deadline);
	}
	return ret;
}

int k_deferred_release_post_selected(struct k_deferred_release *release,
				     struct k_clock_event *clock_event,
				     uint32_t not_before, int deadline,
				     uint32_t required_power_states,
				     uint32_t max_min_arm_cycles)
{
	const struct k_clock_event_provider *provider;
	struct k_deadline_event *event = NULL;
	uint32_t now;
	bool wait = false;
	int ret = 0;

	if (release == NULL || release->event == NULL || clock_event == NULL) {
		return -EINVAL;
	}

	provider = k_clock_event_provider_select(required_power_states,
					  max_min_arm_cycles);
	if (provider == NULL) {
		return -ENODEV;
	}

	K_SPINLOCK(&release->lock) {
		/* A selected post replaces any pending deferred publication. */
		if (release->clock_event != NULL) {
			k_clock_event_cancel(release->clock_event);
		}
		release->armed = false;
		if (provider->backend->event_init != NULL) {
			ret = provider->backend->event_init(provider->context, clock_event,
						    deferred_release_fire, release);
		} else {
			ret = k_clock_event_init(clock_event, provider->backend, provider->context,
						 deferred_release_fire, release);
		}
		if (ret != 0) {
			break;
		}
		release->clock_event = clock_event;
		now = k_clock_event_now(clock_event);
		if ((int32_t)(not_before - now) <= 0) {
			event = release->event;
		} else {
			release->deadline = deadline;
			release->armed = true;
			ret = k_clock_event_arm_abs(clock_event, not_before);
			if (ret == -ETIME) {
				/* Inside the arm lead: wait it out after unlocking. */
				release->armed = false;
				wait = true;
				ret = 0;
			} else if (ret != 0) {
				release->armed = false;
			}
		}
	}
	if (wait) {
		deferred_release_wait_until(clock_event, not_before);
		k_deadline_event_post(release->event, deadline);
	}
	if (event != NULL) {
		k_deadline_event_post(event, deadline);
	}
	return ret;
}

void k_deferred_release_cancel(struct k_deferred_release *release)
{
	__ASSERT(release != NULL, "deferred release is NULL");
	K_SPINLOCK(&release->lock) {
		release->armed = false;
		k_clock_event_cancel(release->clock_event);
	}
}
