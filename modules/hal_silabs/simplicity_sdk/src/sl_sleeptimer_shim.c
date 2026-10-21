/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Sleeptimer tick queries on Zephyr's uptime, for HAL drivers that only measure elapsed time
 * (for example sl_i2c.c timeouts) in builds whose system timer is not the sleeptimer.
 */

#include <zephyr/kernel.h>

#include <sl_sleeptimer.h>

uint32_t sl_sleeptimer_get_tick_count(void)
{
	return (uint32_t)k_uptime_ticks();
}

uint64_t sl_sleeptimer_get_tick_count64(void)
{
	return (uint64_t)k_uptime_ticks();
}

uint32_t sl_sleeptimer_tick_to_ms(uint32_t tick)
{
	return k_ticks_to_ms_floor32(tick);
}
