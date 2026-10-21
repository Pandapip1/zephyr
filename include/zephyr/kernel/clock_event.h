/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Optional absolute clock-event provider interface.
 *
 * This interface is deliberately separate from the system-clock timeout API.
 * A system-clock timeout requests a tick announcement; a clock event requests
 * one absolute-cycle callback. In particular, it does not change the
 * not-before semantics of @ref k_timeout_t.
 */

#ifndef ZEPHYR_INCLUDE_ZEPHYR_KERNEL_CLOCK_EVENT_H_
#define ZEPHYR_INCLUDE_ZEPHYR_KERNEL_CLOCK_EVENT_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

struct k_clock_event;

/** Provider remains usable while the CPU is active. */
#define K_CLOCK_EVENT_POWER_ACTIVE BIT(0)
/** Provider remains usable in the platform's deepest retained sleep state. */
#define K_CLOCK_EVENT_POWER_DEEP_SLEEP BIT(1)

/**
 * @brief Callback invoked when an armed clock event expires.
 *
 * @p generation identifies the particular successful arm that expired.  It is
 * supplied so clients that keep their own arm payload can reject a callback
 * captured before a replacement/cancel race without consulting mutable
 * "latest arm" state.
 */
typedef void (*k_clock_event_handler_t)(struct k_clock_event *event,
					uint32_t generation, void *user_data);

/**
 * @brief Optional hardware clock-event provider operations.
 *
 * All cycle values share one wrapping 32-bit cycle domain. A deadline may be
 * at most INT32_MAX cycles ahead of @ref now. The generic layer serializes
 * @ref arm_abs and @ref cancel for one event, and never calls them from its
 * expiry callback. Providers must not invoke k_clock_event_fire() inline from
 * arm_abs().
 */
struct k_clock_event_backend {
	/**
	 * Optionally bind an event to provider-specific delivery state.
	 *
	 * Providers whose interrupt mux needs to retain the event pointer use this
	 * hook. Providers without such state leave it NULL and the generic layer
	 * initializes the event directly.
	 */
	int (*event_init)(void *context, struct k_clock_event *event,
			  k_clock_event_handler_t handler, void *user_data);
	/** Read the current cycle count in the provider's cycle domain. */
	uint32_t (*now)(void *context);
	/** Return the minimum reliable lead time for an absolute arm. */
	uint32_t (*min_arm_cycles)(void *context);
	/**
	 * Arm @p generation for an absolute deadline.
	 *
	 * Return -ETIME when the provider cannot arm reliably because the target is
	 * too close or already past. The provider's eventual IRQ must call
	 * k_clock_event_fire() with this generation.
	 */
	int (*arm_abs)(void *context, uint32_t deadline, uint32_t generation);
	/** Cancel the currently armed provider alarm, if any. */
	void (*cancel)(void *context);
};

/** A registered provider and the power states in which it can deliver. */
struct k_clock_event_provider {
	const struct k_clock_event_backend *backend;
	void *context;
	uint32_t power_states;
};

/**
 * @brief A single logical absolute clock event.
 *
 * The fields are internal. Initialize with @ref k_clock_event_init before
 * use. One event maps to one provider alarm channel at a time.
 */
struct k_clock_event {
	struct k_spinlock lock;
	const struct k_clock_event_backend *backend;
	void *context;
	k_clock_event_handler_t handler;
	void *user_data;
	uint32_t generation;
	bool armed;
};

#define Z_CLOCK_EVENT_INITIALIZER \
	{ .backend = NULL, .context = NULL, .handler = NULL, .user_data = NULL, \
	  .generation = 0U, .armed = false }

/** @brief Statically define a clock event; call k_clock_event_init() before use. */
#define K_CLOCK_EVENT_DEFINE(name) struct k_clock_event name = Z_CLOCK_EVENT_INITIALIZER

/** Initialize an event with a provider and its expiry handler. */
int k_clock_event_init(struct k_clock_event *event,
		       const struct k_clock_event_backend *backend, void *context,
		       k_clock_event_handler_t handler, void *user_data);

/** Replace an unarmed event's handler. Returns -EBUSY while armed. */
int k_clock_event_set_handler(struct k_clock_event *event,
			    k_clock_event_handler_t handler, void *user_data);

/**
 * @brief Register an optional provider for selector clients.
 *
 * Direct @ref k_clock_event_init users need not register. Providers selected
 * together must use the same cycle domain; @p max_min_arm_cycles is measured
 * in that domain. Registration is intended for device initialization.
 */
int k_clock_event_provider_register(const struct k_clock_event_backend *backend,
				    void *context, uint32_t power_states);

/** Select the lowest-lead provider satisfying power and arm-lead requirements. */
const struct k_clock_event_provider *
k_clock_event_provider_select(uint32_t required_power_states, uint32_t max_min_arm_cycles);

/** Return the provider's minimum reliable absolute-arm lead, in cycles. */
uint32_t k_clock_event_min_arm_cycles(const struct k_clock_event *event);

/** Return the provider's current cycle count in its absolute-arm domain. */
uint32_t k_clock_event_now(const struct k_clock_event *event);

/**
 * @brief Arm an absolute event.
 *
 * Replaces any old arm. Returns -ETIME without calling the provider when the
 * deadline is already inside its declared minimum lead; a provider may also
 * return -ETIME after its own final hardware check. This routine never makes
 * work eligible early—the caller decides how to handle a late arm.
 */
int k_clock_event_arm_abs(struct k_clock_event *event, uint32_t deadline);

/**
 * @brief Arm an absolute event and return the generation assigned to this arm.
 *
 * This is otherwise identical to @ref k_clock_event_arm_abs.  Callers that
 * associate private payload with an arm should keep the returned generation
 * with that payload and compare it in their expiry callback.
 */
int k_clock_event_arm_abs_with_generation(struct k_clock_event *event,
					    uint32_t deadline, uint32_t *generation);

/** Cancel an event and invalidate any stale provider callback. */
void k_clock_event_cancel(struct k_clock_event *event);

/**
 * @brief Publish an expiry from a provider IRQ.
 *
 * Stale generations and cancelled events are discarded. The user handler runs
 * after the event lock is released and may arm or cancel the event again.
 */
void k_clock_event_fire(struct k_clock_event *event, uint32_t generation);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ZEPHYR_KERNEL_CLOCK_EVENT_H_ */
