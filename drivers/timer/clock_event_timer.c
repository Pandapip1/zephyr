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
 *
 * With CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB the expiry only releases a deadline
 * job, which announces. From the release until that job has announced and
 * reprogrammed, only the job arms: one release is outstanding at a time, and
 * no two are closer than the job's minimum inter-arrival.
 */

#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/clock_event.h>
#include <zephyr/sys/clock.h>

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

#ifdef CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB
BUILD_ASSERT(IS_ENABLED(CONFIG_TICKLESS_KERNEL), "the announcing job reprograms on announce");
BUILD_ASSERT(CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB_DEADLINE_NS <=
	     CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB_MIN_INTERARRIVAL_NS,
	     "an announcement is due before the next can be released");

#define JOB_MIN_GAP k_ns_to_cyc_ceil64(CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB_MIN_INTERARRIVAL_NS)

static void announce_job(void *a, void *b, void *c);

K_THREAD_DEFINE(sys_clock_announce_thread, CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB_STACK_SIZE,
		announce_job, NULL, NULL, NULL, CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB_PRIORITY, 0, 0);
/* Bound to its consumer from the start, so that the job can arm before its first wait. */
static struct k_deadline_event announce_event = {
	.consumer = &_k_thread_obj_sys_clock_announce_thread,
	.name = "sys_clock_announce",
};
/* Released and not yet reprogrammed by the job; under the system clock lock. */
static bool released;
/* When the last release was. */
static uint64_t released_at;

static bool arm_allowed(void)
{
	return !released || (!k_is_in_isr() && k_current_get() == sys_clock_announce_thread);
}
#else
static bool arm_allowed(void)
{
	return true;
}
#endif

static void arm(uint64_t now, uint32_t ticks)
{
	uint64_t elapsed = (now - announced) / CYC_PER_TICK;
	uint64_t deadline = announced + (elapsed + ticks) * CYC_PER_TICK;

	if (!arm_allowed()) {
		return;
	}
#ifdef CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB
	if (deadline < released_at + JOB_MIN_GAP) {
		deadline = released_at + JOB_MIN_GAP;
	}
#endif
	/*
	 * A tick boundary inside the service's minimum lead is refused, and so is
	 * one the clock passes while the arm runs: the service reads the clock
	 * again after the lead check, and a read costs time. Either way nothing
	 * would be armed and the kernel would never be announced to again. Arm
	 * the first later boundary the service accepts instead: announcements
	 * fall on tick boundaries and a timeout may expire late, never early.
	 * Each refusal costs less than a tick, so this ends.
	 */
	while (k_clock_event_arm_abs(&timeout_event, (uint32_t)deadline) != 0) {
		deadline += CYC_PER_TICK;
	}
}

#ifdef CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB
static void timeout_expired(struct k_clock_event *event, uint32_t generation, void *user_data)
{
	k_spinlock_key_t key = sys_clock_lock();
	uint64_t now = k_clock_event_sched_now();

	ARG_UNUSED(event);
	ARG_UNUSED(generation);
	ARG_UNUSED(user_data);
	released = true;
	released_at = now;
	sys_clock_unlock(key);
	k_deadline_event_post(&announce_event,
			      (int)((uint32_t)now +
				    k_ns_to_cyc_ceil32(CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB_DEADLINE_NS)));
}

static void announce_job(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

#if CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB_BUDGET_NS > 0
	static struct k_deadline_reservation reservation = {
		.budget = k_ns_to_cyc_ceil32(CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB_BUDGET_NS),
		.min_interarrival =
			k_ns_to_cyc_floor32(CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB_MIN_INTERARRIVAL_NS),
		.deadline = k_ns_to_cyc_floor32(CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB_DEADLINE_NS),
		.isr_budget = k_ns_to_cyc_ceil32(CONFIG_CLOCK_EVENT_SYSTEM_TIMER_JOB_ISR_NS),
	};

	/* A refusal is reported; the job runs either way. */
	(void)k_deadline_event_arm(&announce_event, &reservation);
#endif
	for (;;) {
		k_spinlock_key_t key;
		uint64_t now;
		uint32_t ticks;

		k_deadline_event_wait(&announce_event);
		key = sys_clock_lock();
		now = k_clock_event_sched_now();
		ticks = (uint32_t)((now - announced) / CYC_PER_TICK);
		announced += ticks * CYC_PER_TICK;
		if (ticks == 0U) {
			arm(now, 1U);
		}
		/* Announcing reprograms the next expiry. */
		sys_clock_announce_locked(ticks, key);
		key = sys_clock_lock();
		released = false;
		sys_clock_unlock(key);
	}
}
#else
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
#endif

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
