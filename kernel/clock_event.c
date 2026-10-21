/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel/clock_event.h>

int k_clock_event_init(struct k_clock_event *event,
		       const struct k_clock_event_backend *backend, void *context,
		       k_clock_event_handler_t handler, void *user_data)
{
	if ((event == NULL) || (backend == NULL) || (backend->now == NULL) ||
	    (backend->min_arm_cycles == NULL) || (backend->arm_abs == NULL) ||
	    (backend->cancel == NULL) || (handler == NULL)) {
		return -EINVAL;
	}

	K_SPINLOCK(&event->lock) {
		event->backend = backend;
		event->context = context;
		event->handler = handler;
		event->user_data = user_data;
		event->generation = 0U;
		event->armed = false;
	}

	return 0;
}

uint32_t k_clock_event_min_arm_cycles(const struct k_clock_event *event)
{
	__ASSERT(event != NULL, "clock event is NULL");
	__ASSERT(event->backend != NULL, "clock event is not initialized");

	return event->backend->min_arm_cycles(event->context);
}

int k_clock_event_arm_abs(struct k_clock_event *event, uint32_t deadline)
{
	int ret;

	if ((event == NULL) || (event->backend == NULL)) {
		return -EINVAL;
	}

	K_SPINLOCK(&event->lock) {
		uint32_t now = event->backend->now(event->context);
		uint32_t lead = event->backend->min_arm_cycles(event->context);
		int32_t remaining = (int32_t)(deadline - now);

		/* The API's half-range rule also makes this signed comparison safe. */
		if ((lead > INT32_MAX) || (remaining < (int32_t)lead)) {
			ret = -ETIME;
			break;
		}

		/* Replacement invalidates an old callback before changing hardware. */
		if (event->armed) {
			event->backend->cancel(event->context);
		}
		event->armed = false;
		event->generation++;
		ret = event->backend->arm_abs(event->context, deadline, event->generation);
		if (ret == 0) {
			event->armed = true;
		}
	}

	return ret;
}

void k_clock_event_cancel(struct k_clock_event *event)
{
	__ASSERT(event != NULL, "clock event is NULL");
	__ASSERT(event->backend != NULL, "clock event is not initialized");

	K_SPINLOCK(&event->lock) {
		event->generation++;
		if (event->armed) {
			event->backend->cancel(event->context);
			event->armed = false;
		}
	}
}

void k_clock_event_fire(struct k_clock_event *event, uint32_t generation)
{
	k_clock_event_handler_t handler = NULL;
	void *user_data = NULL;

	__ASSERT(event != NULL, "clock event is NULL");
	__ASSERT(event->backend != NULL, "clock event is not initialized");

	K_SPINLOCK(&event->lock) {
		if (event->armed && (event->generation == generation)) {
			event->armed = false;
			handler = event->handler;
			user_data = event->user_data;
		}
	}

	if (handler != NULL) {
		handler(event, user_data);
	}
}
