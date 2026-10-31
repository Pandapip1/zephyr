/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <zephyr/kernel/deferred_release.h>
#include <zephyr/ztest.h>

struct fake_provider {
	uint32_t now;
	/* When non-zero, each clock read advances time by this many cycles. */
	uint32_t tick_per_read;
	uint32_t min_lead;
	uint32_t generation;
	unsigned int arms;
	unsigned int cancels;
	uint32_t armed_at;
};

static uint32_t fake_now(void *context)
{
	struct fake_provider *provider = context;
	uint32_t now = provider->now;

	provider->now += provider->tick_per_read;
	return now;
}

static uint32_t fake_min_lead(void *context)
{
	return ((struct fake_provider *)context)->min_lead;
}

static int fake_arm(void *context, uint32_t deadline, uint32_t generation)
{
	struct fake_provider *provider = context;

	provider->armed_at = deadline;
	provider->generation = generation;
	provider->arms++;
	return 0;
}

static void fake_cancel(void *context)
{
	((struct fake_provider *)context)->cancels++;
}

static const struct k_clock_event_backend fake_backend = {
	.now = fake_now,
	.min_arm_cycles = fake_min_lead,
	.arm_abs = fake_arm,
	.cancel = fake_cancel,
};

K_DEADLINE_EVENT_DEFINE(event);
K_DEFERRED_RELEASE_DEFINE(release);
K_CLOCK_EVENT_DEFINE(clock_event);
K_CLOCK_EVENT_DEFINE(selected_clock_event);

static struct fake_provider selected_provider = {
	.now = 100U,
	.min_lead = 3U,
};

static void unused_clock_event_handler(struct k_clock_event *event, uint32_t generation,
				       void *user_data)
{
	ARG_UNUSED(event);
	ARG_UNUSED(generation);
	ARG_UNUSED(user_data);
}

static void reset(struct fake_provider *provider)
{
	*provider = (struct fake_provider) { .now = 100U, .min_lead = 5U };
	k_deadline_event_init(&event);
	zassert_equal(k_clock_event_init(&clock_event, &fake_backend, provider,
					 unused_clock_event_handler, NULL), 0, "clock init failed");
	zassert_equal(k_deferred_release_init(&release, &event, &clock_event), 0,
		      "init failed");
	k_deferred_release_lead_set(&release, 0U);
}

ZTEST(deferred_release, test_future_release_waits_for_provider_expiry)
{
	struct fake_provider provider;

	reset(&provider);
	zassert_equal(k_deferred_release_post(&release, 110U, 222), 0,
		      "future release arm failed");
	zassert_equal(provider.arms, 1U, "provider was not armed");
	zassert_false(event.pending, "released before not-before");
	k_clock_event_fire(&clock_event, provider.generation);
	zassert_true(event.pending, "provider expiry did not release event");
}

ZTEST(deferred_release, test_due_release_posts_immediately)
{
	struct fake_provider provider;

	reset(&provider);
	zassert_equal(k_deferred_release_post(&release, 100U, 222), 0,
		      "due release failed");
	zassert_equal(provider.arms, 0U, "due release unnecessarily armed");
	zassert_true(event.pending, "due release was not posted");
}

ZTEST(deferred_release, test_inside_lead_release_waits_then_posts)
{
	struct fake_provider provider;

	reset(&provider);
	provider.tick_per_read = 1U;
	zassert_equal(k_deferred_release_post(&release, 104U, 222), 0,
		      "inside-lead release was refused");
	zassert_equal(provider.arms, 0U, "inside-lead release reached provider");
	zassert_true(event.pending, "inside-lead release was dropped");
	zassert_true((int32_t)(provider.now - 104U) >= 0, "inside-lead release posted early");
}

ZTEST(deferred_release, test_selected_provider_arms_deferred_release)
{
	struct fake_provider provider;

	reset(&provider);
	selected_provider = (struct fake_provider) { .now = 100U, .min_lead = 3U };
	zassert_equal(k_clock_event_provider_register(&fake_backend, &selected_provider,
					      K_CLOCK_EVENT_POWER_DEEP_SLEEP), 0,
		      "provider registration failed");
	zassert_equal(k_deferred_release_post_selected(&release, &selected_clock_event,
					       110U, K_DEADLINE(222U),
					       K_CLOCK_EVENT_POWER_DEEP_SLEEP, 3U), 0,
		      "selected release arm failed");
	zassert_equal(selected_provider.arms, 1U, "selected provider was not armed");
	zassert_false(event.pending, "selected release posted early");
	k_clock_event_fire(&selected_clock_event, selected_provider.generation);
	zassert_true(event.pending, "selected provider expiry did not release event");
}

/*
 * A lead taken off the arm compensates the wake path: the clock event is armed
 * that much before the release time, and the publication it makes carries the
 * release time as the run's not-before bound.
 */
ZTEST(deferred_release, test_lead_arms_early_and_bounds_the_start)
{
	struct fake_provider provider;

	reset(&provider);
	k_deferred_release_lead_set(&release, 4U);
	zassert_equal(k_deferred_release_post(&release, 120U, 222), 0, "arm failed");
	zassert_equal(provider.arms, 1U, "provider was not armed");
	zassert_equal(provider.armed_at, 116U, "not armed a lead early");
	zassert_false(event.pending, "released before the early arm expired");
	k_clock_event_fire(&clock_event, provider.generation);
	zassert_true(event.pending, "expiry did not release event");
	zassert_true(event.has_not_before, "release time not carried");
	zassert_equal(event.not_before, 120U, "wrong not-before bound");
}

ZTEST_SUITE(deferred_release, NULL, NULL, NULL, NULL, NULL);
