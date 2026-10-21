/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file @brief Silicon Labs BURTC clock-event provider. */

#ifndef ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_COUNTER_SILABS_BURTC_H_
#define ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_COUNTER_SILABS_BURTC_H_

#include <zephyr/device.h>
#include <zephyr/kernel/clock_event.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bind one generic clock event to a combined BURTC system-clock device.
 *
 * The combined BURTC driver owns its sole COMP channel and multiplexes this
 * event with the system-clock deadline and the Counter API's one alarm. The
 * implementation has a small fixed pool of bound events; each event must
 * remain valid for the lifetime of its binding.
 */
int counter_silabs_burtc_clock_event_init(const struct device *dev,
					  struct k_clock_event *event,
					  k_clock_event_handler_t handler,
					  void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_COUNTER_SILABS_BURTC_H_ */
