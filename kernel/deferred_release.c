/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <zephyr/kernel/deferred_release.h>

static void deferred_release_fire(struct k_clock_event *clock_event, void *user_data)
{
	struct k_deferred_release *release = user_data;
	struct k_deadline_event *event = NULL;
	int deadline = 0;

	ARG_UNUSED(clock_event);
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
			if (ret != 0) {
				release->armed = false;
			}
		}
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
