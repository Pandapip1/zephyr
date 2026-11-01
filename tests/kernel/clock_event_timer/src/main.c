/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The system timer on the scheduler clock service (drivers/timer/clock_event_timer.c), driven
 * by fake providers whose clock reads cost time, as a peripheral read does. The kernel must be
 * announced to after every sys_clock_set_timeout(), whatever the clock does during the arm.
 */

#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/clock_event.h>
#include <zephyr/ztest.h>

#define LOGICAL_HZ  CONFIG_CLOCK_EVENT_SCHED_FREQUENCY
#define CYC_PER_TICK (LOGICAL_HZ / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define DEEP_HZ     32768U
#define DEEP_MIN    6U
#define ACTIVE_HZ   1000000U
#define ACTIVE_MIN  2U
/* The service's minimum lead in logical cycles: the active provider's, rounded up. */
#define LEAD        DIV_ROUND_UP((uint64_t)ACTIVE_MIN * LOGICAL_HZ, ACTIVE_HZ)

/* Both fake providers count one shared time base; every read advances it by read_ns. */
static uint64_t fake_ns;
#define READ_NS_DEFAULT 200U
/* A deep edge is waited for by reading the clock: reads must always advance it. */
static uint32_t read_ns = READ_NS_DEFAULT;

struct fake_provider;

struct fake_slot {
	struct fake_provider *provider;
	struct k_clock_event *event;
	uint32_t deadline;
	uint32_t generation;
	bool armed;
};

struct fake_provider {
	struct fake_provider *provider; /* context layout shared with fake_slot */
	uint32_t hz;
	uint32_t min_lead;
	struct fake_slot slots[1];
	size_t used;
};

static struct fake_provider deep = {.provider = &deep, .hz = DEEP_HZ, .min_lead = DEEP_MIN};
static struct fake_provider active = {.provider = &active, .hz = ACTIVE_HZ,
				      .min_lead = ACTIVE_MIN};

static struct fake_provider *owner(void *context)
{
	return ((struct fake_slot *)context)->provider;
}

static uint32_t cycles_at(const struct fake_provider *p, uint64_t ns)
{
	return (uint32_t)((ns * p->hz) / NSEC_PER_SEC);
}

static uint64_t ns_at(const struct fake_provider *p, uint32_t cycles)
{
	return ((uint64_t)cycles * NSEC_PER_SEC) / p->hz;
}

static uint32_t fake_now(void *context)
{
	fake_ns += read_ns;
	return cycles_at(owner(context), fake_ns);
}

static uint32_t fake_min_arm_cycles(void *context)
{
	return owner(context)->min_lead;
}

static uint32_t fake_frequency_hz(void *context)
{
	return owner(context)->hz;
}

static int fake_arm_abs(void *context, uint32_t deadline, uint32_t generation)
{
	struct fake_slot *slot = context;

	/* Like real providers, refuse a deadline inside the declared lead. */
	if ((int32_t)(deadline - cycles_at(slot->provider, fake_ns)) <
	    (int32_t)slot->provider->min_lead) {
		return -ETIME;
	}
	slot->deadline = deadline;
	slot->generation = generation;
	slot->armed = true;
	return 0;
}

static void fake_cancel(void *context)
{
	((struct fake_slot *)context)->armed = false;
}

static int fake_event_init(void *context, struct k_clock_event *event,
			   k_clock_event_handler_t handler, void *user_data);

static const struct k_clock_event_backend fake_backend = {
	.event_init = fake_event_init,
	.now = fake_now,
	.min_arm_cycles = fake_min_arm_cycles,
	.arm_abs = fake_arm_abs,
	.cancel = fake_cancel,
	.frequency_hz = fake_frequency_hz,
};

static int fake_event_init(void *context, struct k_clock_event *event,
			   k_clock_event_handler_t handler, void *user_data)
{
	struct fake_provider *p = context;
	struct fake_slot *slot = &p->slots[p->used++];

	slot->provider = p;
	slot->event = event;
	return k_clock_event_init(event, &fake_backend, slot, handler, user_data);
}

/* The providers exist before the system timer starts, as a SoC's do. */
static int fake_providers_register(void)
{
	int ret = k_clock_event_provider_register(&fake_backend, &deep,
						  K_CLOCK_EVENT_POWER_ACTIVE |
						  K_CLOCK_EVENT_POWER_DEEP_SLEEP);

	if (ret == 0) {
		ret = k_clock_event_provider_register(&fake_backend, &active,
						      K_CLOCK_EVENT_POWER_ACTIVE);
	}
	return ret;
}

SYS_INIT(fake_providers_register, PRE_KERNEL_1, 0);

/* The driver's entry points, renamed by CMakeLists.txt. */
void cet_set_timeout(int32_t ticks, bool idle);

static uint32_t announced_ticks;

/* The kernel's announcement: count it, then ask for nothing, as an idle tickless kernel does. */
void cet_announce_locked(uint32_t ticks, k_spinlock_key_t key)
{
	announced_ticks += ticks;
	cet_set_timeout(K_TICKS_FOREVER, false);
	sys_clock_unlock(key);
}

/* The armed provider compare due first, or NULL. */
static struct fake_slot *earliest_armed(void)
{
	struct fake_slot *d = &deep.slots[0];
	struct fake_slot *a = &active.slots[0];

	if (d->armed && a->armed) {
		return (ns_at(&deep, d->deadline) <= ns_at(&active, a->deadline)) ? d : a;
	}
	return d->armed ? d : (a->armed ? a : NULL);
}

/* Deliver a provider expiry the way a provider IRQ would. */
static void fake_expire(struct fake_slot *slot)
{
	unsigned int key = irq_lock();

	slot->armed = false;
	fake_ns = MAX(fake_ns, ns_at(slot->provider, slot->deadline) + 1U);
	k_clock_event_fire(slot->event, slot->generation);
	irq_unlock(key);
}

/* Free reads: move time to the first instant @p before logical cycles ahead of a boundary. */
static uint64_t place_before_boundary(uint32_t before)
{
	uint64_t now = k_clock_event_sched_now();
	uint64_t boundary = (now / CYC_PER_TICK + 2U) * CYC_PER_TICK;

	while (k_clock_event_sched_now() < boundary - before) {
		fake_ns += NSEC_PER_USEC / 10U;
	}
	return boundary;
}

/*
 * The kernel asks for an announcement at the next tick boundary while time stands within
 * @p before of it; every clock read costs @p cost ns. Returns the boundary.
 */
static uint64_t ask_one_tick(uint32_t before, uint32_t cost)
{
	uint64_t boundary;
	unsigned int key;

	read_ns = 0U;
	boundary = place_before_boundary(before);
	read_ns = cost;
	/* The kernel calls it with its timeout lock held. */
	key = irq_lock();
	cet_set_timeout(1, false);
	irq_unlock(key);
	read_ns = READ_NS_DEFAULT;
	return boundary;
}

/* Deliver the announcement that was asked for; it must come within two ticks of @p boundary. */
static void expect_announcement(uint64_t boundary)
{
	struct fake_slot *slot = earliest_armed();
	uint32_t ticks = announced_ticks;

	zassert_not_null(slot, "nothing armed: the kernel is never announced to again");
	zassert_true(ns_at(slot->provider, slot->deadline) <=
		     (boundary + 2U * CYC_PER_TICK) * NSEC_PER_SEC / LOGICAL_HZ,
		     "the request was dropped: the next arm is the older, later one");
	/* A deep wake hands off to the active provider, which then delivers. */
	for (int i = 0; i < 2 && announced_ticks == ticks; i++) {
		slot = earliest_armed();
		zassert_not_null(slot, "the hand-off armed nothing");
		fake_expire(slot);
	}
	zassert_true(announced_ticks > ticks, "no announcement");
	zassert_true(k_clock_event_sched_now() <= boundary + 2U * CYC_PER_TICK,
		     "announced later than the next boundary the service can arm");
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	/* Start from an idle kernel timeout: at most one long arm pending. */
	expect_announcement(ask_one_tick(CYC_PER_TICK / 2U, READ_NS_DEFAULT));
}

ZTEST(clock_event_timer, test_boundary_clear_of_the_lead)
{
	expect_announcement(ask_one_tick(CYC_PER_TICK / 2U, NSEC_PER_USEC / 5U));
}

/*
 * Every phase of the request against the next boundary, with clock reads from much cheaper
 * than the lead to longer than it. Close to the boundary the arm is refused at the lead check;
 * a little further out the check passes and the clock passes the boundary before the service
 * reads it again. Either way the announcement must still be armed.
 */
ZTEST(clock_event_timer, test_every_phase_and_read_cost)
{
	static const uint32_t costs_ns[] = {
		NSEC_PER_USEC / 5U,
		NSEC_PER_USEC,
		ACTIVE_MIN * NSEC_PER_USEC,
		3U * ACTIVE_MIN * NSEC_PER_USEC,
	};

	ARRAY_FOR_EACH(costs_ns, i) {
		for (uint32_t before = 1U; before < 12U * LEAD; before += LEAD / 4U) {
			expect_announcement(ask_one_tick(before, costs_ns[i]));
		}
	}
}

ZTEST_SUITE(clock_event_timer, NULL, NULL, before, NULL, NULL);
