/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <zephyr/kernel/deferred_release.h>
#include <zephyr/ztest.h>

struct fake_provider {
	uint32_t now;
	uint32_t min_lead;
	uint32_t generation;
	unsigned int arms;
	unsigned int cancels;
};

static uint32_t fake_now(void *context)
{
	return ((struct fake_provider *)context)->now;
}

static uint32_t fake_min_lead(void *context)
{
	return ((struct fake_provider *)context)->min_lead;
}

static int fake_arm(void *context, uint32_t deadline, uint32_t generation)
{
	struct fake_provider *provider = context;

	ARG_UNUSED(deadline);
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

static void unused_clock_event_handler(struct k_clock_event *event, void *user_data)
{
	ARG_UNUSED(event);
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

ZTEST(deferred_release, test_too_late_future_release_is_not_posted_early)
{
	struct fake_provider provider;

	reset(&provider);
	zassert_equal(k_deferred_release_post(&release, 104U, 222), -ETIME,
		      "inside-lead release was accepted");
	zassert_equal(provider.arms, 0U, "too-late release reached provider");
	zassert_false(event.pending, "too-late release posted early");
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

ZTEST_SUITE(deferred_release, NULL, NULL, NULL, NULL, NULL);
