/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Timing functions backed by the system clock's 64-bit hardware cycle count.
 *
 * For system timers whose cycle count is fine-grained and keeps advancing in every sleep state
 * the system enters, such as the scheduler clock (CONFIG_CLOCK_EVENT_SYSTEM_TIMER). Time always
 * advances; timing_start() and timing_stop() change nothing.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/clock.h>
#include <zephyr/timing/timing.h>

void soc_timing_init(void)
{
}

void soc_timing_start(void)
{
}

void soc_timing_stop(void)
{
}

timing_t soc_timing_counter_get(void)
{
	return k_cycle_get_64();
}

uint64_t soc_timing_cycles_get(volatile timing_t *const start, volatile timing_t *const end)
{
	return *end - *start;
}

uint64_t soc_timing_freq_get(void)
{
	return sys_clock_hw_cycles_per_sec();
}

uint64_t soc_timing_cycles_to_ns(uint64_t cycles)
{
	return k_cyc_to_ns_floor64(cycles);
}

uint64_t soc_timing_cycles_to_ns_avg(uint64_t cycles, uint32_t count)
{
	return soc_timing_cycles_to_ns(cycles) / count;
}

uint32_t soc_timing_freq_get_mhz(void)
{
	return (uint32_t)(sys_clock_hw_cycles_per_sec() / 1000000U);
}
