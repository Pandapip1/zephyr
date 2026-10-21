/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * System timer on the scheduler clock service.
 *
 * Hardware cycles are the service's logical timebase, so every registered
 * clock-event provider feeds system time. Kernel timeouts are one scheduler
 * clock-event client: the service, not this driver, decides which provider
 * delivers each one.
 */

#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/clock_event.h>
#include <zephyr/sys_clock.h>

BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == CONFIG_CLOCK_EVENT_SCHED_FREQUENCY,
	     "hardware cycles must be the scheduler clock service's timebase");
BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC % CONFIG_SYS_CLOCK_TICKS_PER_SEC == 0,
	     "a kernel tick must be a whole number of cycles");

#define CYC_PER_TICK ((uint64_t)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_TICKS                                                                                  \
	((uint32_t)((uint64_t)CONFIG_CLOCK_EVENT_SYSTEM_TIMER_MAX_TIMEOUT_MS *                     \
		    CONFIG_SYS_CLOCK_TICKS_PER_SEC / MSEC_PER_SEC))

BUILD_ASSERT(MAX_TICKS > 0, "maximum timeout is shorter than a tick");
BUILD_ASSERT((uint64_t)MAX_TICKS * CYC_PER_TICK < INT32_MAX,
	     "maximum timeout must fit a scheduler clock-event arm");

static struct k_clock_event timeout_event;
/* Cycle count of the last announced tick boundary; under the system clock lock. */
static uint64_t announced;

static void arm(uint64_t now, uint32_t ticks)
{
	uint64_t elapsed = (now - announced) / CYC_PER_TICK;
	uint64_t deadline = announced + (elapsed + ticks) * CYC_PER_TICK;

	/*
	 * A deadline inside the service's minimum lead cannot be armed; deliver
	 * it as soon as the service can rather than drop it.
	 */
	if (k_clock_event_arm_abs(&timeout_event, (uint32_t)deadline) != 0) {
		(void)k_clock_event_arm_abs(&timeout_event,
					    (uint32_t)k_clock_event_sched_now() +
					    k_clock_event_min_arm_cycles(&timeout_event) + 1U);
	}
}

static void timeout_expired(struct k_clock_event *event, uint32_t generation, void *user_data)
{
	k_spinlock_key_t key = sys_clock_lock();
	uint64_t now = k_clock_event_sched_now();
	uint32_t ticks = (uint32_t)((now - announced) / CYC_PER_TICK);

	ARG_UNUSED(event);
	ARG_UNUSED(generation);
	ARG_UNUSED(user_data);
	announced += ticks * CYC_PER_TICK;
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL) || ticks == 0U) {
		/* Periodic tick, or an early wake: never leave nothing armed. */
		arm(now, 1U);
	}
	sys_clock_announce_locked(ticks, key);
}

void sys_clock_set_timeout(uint32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}
	arm(k_clock_event_sched_now(), CLAMP(ticks, 1U, MAX_TICKS));
}

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0U;
	}
	return (uint32_t)((k_clock_event_sched_now() - announced) / CYC_PER_TICK);
}

uint32_t sys_clock_cycle_get_32(void)
{
	return (uint32_t)k_clock_event_sched_now();
}

uint64_t sys_clock_cycle_get_64(void)
{
	return k_clock_event_sched_now();
}

static int sys_clock_driver_init(void)
{
	k_spinlock_key_t key;
	int ret = k_clock_event_sched_init(&timeout_event, timeout_expired, NULL);

	if (ret != 0) {
		return ret;
	}
	key = sys_clock_lock();
	announced = k_clock_event_sched_now() / CYC_PER_TICK * CYC_PER_TICK;
	arm(k_clock_event_sched_now(), 1U);
	sys_clock_unlock(key);
	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
