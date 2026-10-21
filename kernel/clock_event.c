/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel/clock_event.h>

static struct k_spinlock provider_lock;
static struct k_clock_event_provider providers[CONFIG_CLOCK_EVENT_MAX_PROVIDERS];
static size_t provider_count;

int k_clock_event_provider_register(const struct k_clock_event_backend *backend,
				    void *context, uint32_t power_states)
{
	int ret = 0;

	if ((backend == NULL) || (backend->now == NULL) ||
	    (backend->min_arm_cycles == NULL) || (backend->arm_abs == NULL) ||
	    (backend->cancel == NULL) || (power_states == 0U)) {
		return -EINVAL;
	}
	K_SPINLOCK(&provider_lock) {
		if (provider_count == ARRAY_SIZE(providers)) {
			ret = -ENOMEM;
		} else {
			providers[provider_count++] = (struct k_clock_event_provider) {
				.backend = backend, .context = context, .power_states = power_states,
			};
		}
	}
	return ret;
}

const struct k_clock_event_provider *
k_clock_event_provider_select(uint32_t required_power_states, uint32_t max_min_arm_cycles)
{
	const struct k_clock_event_provider *best = NULL;
	uint32_t best_lead = UINT32_MAX;

	K_SPINLOCK(&provider_lock) {
		for (size_t i = 0U; i < provider_count; i++) {
			const struct k_clock_event_provider *candidate = &providers[i];
			uint32_t lead;

			if ((candidate->power_states & required_power_states) !=
			    required_power_states) {
				continue;
			}
			lead = candidate->backend->min_arm_cycles(candidate->context);
			if ((lead <= max_min_arm_cycles) && (lead < best_lead)) {
				best = candidate;
				best_lead = lead;
			}
		}
	}
	return best;
}

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

int k_clock_event_set_handler(struct k_clock_event *event,
			    k_clock_event_handler_t handler, void *user_data)
{
	int ret = 0;

	if ((event == NULL) || (event->backend == NULL) || (handler == NULL)) {
		return -EINVAL;
	}
	K_SPINLOCK(&event->lock) {
		if (event->armed) {
			ret = -EBUSY;
		} else {
			event->handler = handler;
			event->user_data = user_data;
		}
	}
	return ret;
}

uint32_t k_clock_event_min_arm_cycles(const struct k_clock_event *event)
{
	__ASSERT(event != NULL, "clock event is NULL");
	__ASSERT(event->backend != NULL, "clock event is not initialized");

	return event->backend->min_arm_cycles(event->context);
}

uint32_t k_clock_event_now(const struct k_clock_event *event)
{
	__ASSERT(event != NULL, "clock event is NULL");
	__ASSERT(event->backend != NULL, "clock event is not initialized");

	return event->backend->now(event->context);
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
