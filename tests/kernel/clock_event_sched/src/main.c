/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/kernel/clock_event.h>
#include <zephyr/ztest.h>

#define LOGICAL_HZ CONFIG_CLOCK_EVENT_SCHED_FREQUENCY
#define DEEP_HZ    32768U
#define DEEP_MIN   6U
#define ACTIVE_HZ  1000000U
#define ACTIVE_MIN 2U
/* The configured budget plus two deep cycles of anchor uncertainty. */
#define WAKE_NS    (CONFIG_CLOCK_EVENT_SCHED_DEEP_WAKE_NS + 2U * NSEC_PER_SEC / DEEP_HZ)

/* Both fake providers count one shared time base; every read advances it. */
static uint64_t fake_ns;

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
	struct fake_slot slots[CONFIG_CLOCK_EVENT_SCHED_CLIENTS];
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

static uint64_t logical_ns(uint64_t logical)
{
	return (logical * NSEC_PER_SEC) / LOGICAL_HZ;
}

static uint32_t fake_now(void *context)
{
	fake_ns += 200U;
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

/* Deliver a provider expiry the way a provider IRQ would. */
static void fake_expire(struct fake_slot *slot)
{
	zassert_true(slot->armed);
	slot->armed = false;
	fake_ns = MAX(fake_ns, ns_at(slot->provider, slot->deadline) + 1U);
	k_clock_event_fire(slot->event, slot->generation);
}

K_CLOCK_EVENT_DEFINE(event);
K_CLOCK_EVENT_DEFINE(second);
static struct k_clock_event *last_published;
static unsigned int publications;
static uint32_t published_generation;
static uint64_t published_at;

static void publish(struct k_clock_event *clock_event, uint32_t generation, void *user_data)
{
	ARG_UNUSED(clock_event);
	ARG_UNUSED(user_data);
	publications++;
	published_generation = generation;
	published_at = k_clock_event_sched_now();
	last_published = clock_event;
}

static void *setup(void)
{
	zassert_ok(k_clock_event_provider_register(&fake_backend, &deep,
						   K_CLOCK_EVENT_POWER_ACTIVE |
						   K_CLOCK_EVENT_POWER_DEEP_SLEEP));
	zassert_ok(k_clock_event_provider_register(&fake_backend, &active,
						   K_CLOCK_EVENT_POWER_ACTIVE));
	zassert_ok(k_clock_event_sched_init(&event, publish, NULL));
	zassert_ok(k_clock_event_sched_init(&second, publish, NULL));
	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	k_clock_event_cancel(&event);
	k_clock_event_cancel(&second);
	publications = 0U;
}

ZTEST(clock_event_sched, test_time_follows_the_fast_provider)
{
	uint64_t t0 = k_clock_event_sched_now();
	uint64_t t1;

	/* 10 us is a third of a deep cycle: only the active provider resolves it. */
	fake_ns += 10U * NSEC_PER_USEC;
	t1 = k_clock_event_sched_now();
	zassert_true(t1 > t0);
	zassert_within(logical_ns(t1 - t0), 10U * NSEC_PER_USEC, 2U * NSEC_PER_USEC);
	zassert_equal(k_clock_event_frequency_hz(&event), LOGICAL_HZ);
}

ZTEST(clock_event_sched, test_min_lead_is_the_active_providers)
{
	zassert_equal(k_clock_event_min_arm_cycles(&event),
		      DIV_ROUND_UP((uint64_t)ACTIVE_MIN * LOGICAL_HZ, ACTIVE_HZ));
	zassert_equal(k_clock_event_arm_abs(&event, k_clock_event_now(&event)), -ETIME);
}

ZTEST(clock_event_sched, test_long_lead_wakes_on_deep_then_hands_off)
{
	struct fake_slot *d = &deep.slots[0];
	struct fake_slot *a = &active.slots[0];
	uint64_t deadline = k_clock_event_sched_now() + 100U * (LOGICAL_HZ / DEEP_HZ);
	uint32_t generation;

	zassert_ok(k_clock_event_arm_abs_with_generation(&event, (uint32_t)deadline,
							  &generation));
	zassert_true(d->armed);
	zassert_false(a->armed);
	zassert_within(ns_at(&deep, d->deadline), logical_ns(deadline) - WAKE_NS,
		       NSEC_PER_SEC / DEEP_HZ + NSEC_PER_USEC,
		       "deep arm must lead by the wake budget");

	fake_expire(d);
	zassert_equal(publications, 0U, "the wake budget is not the deadline");
	zassert_true(a->armed, "the remainder moves to the active provider");

	fake_expire(a);
	zassert_equal(publications, 1U);
	zassert_equal(published_generation, generation);
	zassert_true(published_at >= deadline, "never delivered early");
	zassert_true(logical_ns(published_at - deadline) < 2U * NSEC_PER_USEC);
}

ZTEST(clock_event_sched, test_short_lead_uses_active_provider)
{
	struct fake_slot *d = &deep.slots[0];
	struct fake_slot *a = &active.slots[0];
	uint64_t deadline = k_clock_event_sched_now() + 5U * (LOGICAL_HZ / DEEP_HZ);

	zassert_ok(k_clock_event_arm_abs(&event, (uint32_t)deadline));
	zassert_false(d->armed);
	zassert_true(a->armed);

	fake_expire(a);
	zassert_equal(publications, 1U);
	zassert_true(published_at >= deadline, "never delivered early");
	zassert_true(logical_ns(published_at - deadline) < 2U * NSEC_PER_USEC);
}

ZTEST(clock_event_sched, test_cancel_and_replace_discard_stale_expiry)
{
	struct fake_slot *a = &active.slots[0];
	struct fake_slot stale;

	zassert_ok(k_clock_event_arm_abs(&event, k_clock_event_now(&event) + 200U));
	stale = *a;
	k_clock_event_cancel(&event);
	zassert_false(a->armed);
	stale.armed = true;
	fake_expire(&stale);
	zassert_equal(publications, 0U);

	zassert_ok(k_clock_event_arm_abs(&event, k_clock_event_now(&event) + 200U));
	stale = *a;
	zassert_ok(k_clock_event_arm_abs(&event,
					 k_clock_event_now(&event) + 200U * (LOGICAL_HZ / DEEP_HZ)));
	stale.armed = true;
	fake_expire(&stale);
	zassert_equal(publications, 0U, "a replaced active arm must not publish");
}

ZTEST(clock_event_sched, test_one_provider_compare_serves_every_client)
{
	struct fake_slot *a = &active.slots[0];
	uint64_t now = k_clock_event_sched_now();
	uint64_t late = now + 40U * LOGICAL_HZ / USEC_PER_SEC;
	uint64_t early = now + 20U * LOGICAL_HZ / USEC_PER_SEC;

	/* Two short-lead clients share the active provider's single compare. */
	zassert_ok(k_clock_event_arm_abs(&event, (uint32_t)late));
	zassert_ok(k_clock_event_arm_abs(&second, (uint32_t)early));
	zassert_equal(active.used, 1U, "the service binds one hardware event per provider");

	fake_expire(a);
	zassert_equal(publications, 1U);
	zassert_equal_ptr(last_published, &second, "earliest deadline first");
	zassert_true(published_at >= early);
	zassert_true(a->armed, "the compare moves on to the next client");

	fake_expire(a);
	zassert_equal(publications, 2U);
	zassert_equal_ptr(last_published, &event);
	zassert_true(published_at >= late);
}

/*
 * An arm right at the service's minimum lead can fall inside the active provider's own lead
 * once the service has run. It must still be delivered exactly once, from a provider expiry,
 * never synchronously inside the arm call (whose event is not armed yet).
 */
ZTEST(clock_event_sched, test_arm_inside_provider_lead_is_delivered_once)
{
	struct fake_slot *a = &active.slots[0];
	uint32_t deadline = k_clock_event_now(&event) + k_clock_event_min_arm_cycles(&event);

	zassert_ok(k_clock_event_arm_abs(&event, deadline));
	zassert_equal(publications, 0U, "no delivery from inside the arm call");
	zassert_true(a->armed, "a provider must be armed to deliver it");

	fake_expire(a);
	zassert_equal(publications, 1U);
	zassert_true((int32_t)((uint32_t)published_at - deadline) >= 0, "never delivered early");
}

ZTEST_SUITE(clock_event_sched, NULL, setup, before, NULL, NULL);
