/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/kernel/clock_event.h>
#include <zephyr/ztest.h>

struct fake_clock_event_provider {
	uint32_t now;
	uint32_t min_lead;
	uint32_t deadline;
	uint32_t generation;
	unsigned int arm_calls;
	unsigned int cancel_calls;
	bool reject_arm;
};

static uint32_t fake_now(void *context)
{
	return ((struct fake_clock_event_provider *)context)->now;
}

static uint32_t fake_min_arm_cycles(void *context)
{
	return ((struct fake_clock_event_provider *)context)->min_lead;
}

static int fake_arm_abs(void *context, uint32_t deadline, uint32_t generation)
{
	struct fake_clock_event_provider *provider = context;

	provider->deadline = deadline;
	provider->generation = generation;
	provider->arm_calls++;
	return provider->reject_arm ? -ETIME : 0;
}

static void fake_cancel(void *context)
{
	((struct fake_clock_event_provider *)context)->cancel_calls++;
}

static const struct k_clock_event_backend fake_backend = {
	.now = fake_now,
	.min_arm_cycles = fake_min_arm_cycles,
	.arm_abs = fake_arm_abs,
	.cancel = fake_cancel,
};

K_CLOCK_EVENT_DEFINE(event);
static unsigned int publications;

static void publish_event(struct k_clock_event *clock_event, void *user_data)
{
	ARG_UNUSED(clock_event);
	ARG_UNUSED(user_data);
	publications++;
}

static void reset_event(struct fake_clock_event_provider *provider)
{
	*provider = (struct fake_clock_event_provider) {
		.now = 100U,
		.min_lead = 5U,
	};
	publications = 0U;
	zassert_equal(k_clock_event_init(&event, &fake_backend, provider,
				   publish_event, NULL), 0, "event init failed");
}

ZTEST(clock_event, test_absolute_arm_and_callback_publication)
{
	struct fake_clock_event_provider provider;

	reset_event(&provider);
	zassert_equal(k_clock_event_min_arm_cycles(&event), 5U,
		      "provider lead was not reported");
	zassert_equal(k_clock_event_arm_abs(&event, 110U), 0,
		      "absolute arm failed");
	zassert_equal(provider.arm_calls, 1U, "provider did not arm");
	zassert_equal(provider.deadline, 110U, "wrong absolute deadline");

	/* This models a provider IRQ; the generic layer publishes once. */
	k_clock_event_fire(&event, provider.generation);
	zassert_equal(publications, 1U, "expiry callback was not published");
	k_clock_event_fire(&event, provider.generation);
	zassert_equal(publications, 1U, "expired generation published twice");
}

ZTEST(clock_event, test_too_late_is_reported_without_provider_arm)
{
	struct fake_clock_event_provider provider;

	reset_event(&provider);
	zassert_equal(k_clock_event_arm_abs(&event, 104U), -ETIME,
		      "inside-minimum-lead arm was accepted");
	zassert_equal(provider.arm_calls, 0U,
		      "too-late arm reached provider");
}

ZTEST(clock_event, test_provider_final_too_late_is_reported)
{
	struct fake_clock_event_provider provider;

	reset_event(&provider);
	provider.reject_arm = true;
	zassert_equal(k_clock_event_arm_abs(&event, 110U), -ETIME,
		      "provider final late result was not propagated");
	zassert_equal(provider.arm_calls, 1U,
		      "provider did not perform final arm check");
	k_clock_event_fire(&event, provider.generation);
	zassert_equal(publications, 0U, "failed arm published an event");
}

ZTEST(clock_event, test_cancel_and_replacement_discard_stale_callback)
{
	struct fake_clock_event_provider provider;
	uint32_t old_generation;

	reset_event(&provider);
	zassert_equal(k_clock_event_arm_abs(&event, 110U), 0, "first arm failed");
	old_generation = provider.generation;
	zassert_equal(k_clock_event_arm_abs(&event, 120U), 0, "replacement arm failed");
	zassert_equal(provider.cancel_calls, 1U, "replacement did not cancel old arm");
	k_clock_event_fire(&event, old_generation);
	zassert_equal(publications, 0U, "stale replacement callback published");

	k_clock_event_cancel(&event);
	zassert_equal(provider.cancel_calls, 2U, "cancel did not reach provider");
	k_clock_event_fire(&event, provider.generation);
	zassert_equal(publications, 0U, "cancelled callback published");
}

ZTEST_SUITE(clock_event, NULL, NULL, NULL, NULL, NULL);
