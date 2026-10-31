/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <limits.h>

#include <zephyr/kernel.h>
#include <zephyr/kernel/deferred_release.h>

#ifdef CONFIG_CLOCK_EVENT_SCHED
bool z_clock_event_is_sched(const struct k_clock_event *event);
#endif

/* Publish one release: through the not-before bound when a lead was taken off its arm. */
static void deferred_release_publish(struct k_deferred_release *release, uint32_t not_before,
				     int deadline)
{
	if (release->lead != 0U) {
		k_deadline_event_post_at(release->event, not_before, deadline);
	} else {
		k_deadline_event_post(release->event, deadline);
	}
}

static void deferred_release_fire(struct k_clock_event *clock_event, uint32_t generation,
				  void *user_data)
{
	struct k_deferred_release *release = user_data;
	bool fire = false;
	uint32_t not_before = 0U;
	int deadline = 0;

	ARG_UNUSED(clock_event);
	ARG_UNUSED(generation);
	K_SPINLOCK(&release->lock) {
		if (release->armed) {
			release->armed = false;
			fire = true;
			deadline = release->deadline;
			not_before = release->not_before;
		}
	}
	if (fire) {
		deferred_release_publish(release, not_before, deadline);
	}
}

/*
 * A release time inside the provider's minimum arm lead cannot be armed, but
 * it is still a release time: wait it out on the provider's own clock rather
 * than drop the release or publish it early. The wait is bounded by that lead.
 */
static void deferred_release_wait_until(struct k_clock_event *clock_event, uint32_t not_before)
{
	while ((int32_t)(not_before - k_clock_event_now(clock_event)) > 0) {
		arch_nop();
	}
}

/*
 * Admit a release armed at @p arm_at whose run starts at @p not_before. With the
 * scheduler clock service, its delivery plan bounds the arrival and names any wait
 * this caller or an expiry interrupt must spend; that wait is demand too.
 */
static int deferred_release_admit(struct k_deferred_release *release,
				  struct k_clock_event *clock_event, uint32_t arm_at,
				  uint32_t not_before, int deadline)
{
#ifdef CONFIG_SCHED_DEADLINE_ADMISSION
#ifdef CONFIG_CLOCK_EVENT_SCHED
	if (z_clock_event_is_sched(clock_event)) {
		struct k_clock_event_sched_plan plan;
		uint64_t now = k_clock_event_sched_now();
		uint64_t target = now + (int64_t)(int32_t)(arm_at - (uint32_t)now);

		if (k_clock_event_sched_plan(target, &plan) == 0) {
			return k_deadline_event_admit_plan(release->event, &plan, arm_at,
							   not_before, deadline);
		}
	}
#endif
	return k_deadline_event_admit(release->event, arm_at, not_before, deadline);
#else
	ARG_UNUSED(release);
	ARG_UNUSED(clock_event);
	ARG_UNUSED(arm_at);
	ARG_UNUSED(not_before);
	ARG_UNUSED(deadline);
	return 0;
#endif
}

/* Arm @p release on its clock event, or publish it now. Caller holds release->lock. */
static int deferred_release_arm_locked(struct k_deferred_release *release, uint32_t arm_at,
				       uint32_t not_before, int deadline, bool *publish, bool *wait)
{
	int ret;

	if ((int32_t)(arm_at - k_clock_event_now(release->clock_event)) <= 0) {
		release->armed = false;
		*publish = true;
		return 0;
	}
	release->deadline = deadline;
	release->not_before = not_before;
	release->armed = true;
	ret = k_clock_event_arm_abs(release->clock_event, arm_at);
	if (ret == -ETIME) {
		/* Inside the arm lead: wait it out after unlocking. */
		release->armed = false;
		*wait = true;
		ret = 0;
	} else if (ret != 0) {
		release->armed = false;
	}
	return ret;
}

int k_deferred_release_init(struct k_deferred_release *release,
			    struct k_deadline_event *event,
			    struct k_clock_event *clock_event)
{
	if (release == NULL || event == NULL || clock_event == NULL) {
		return -EINVAL;
	}
	release->event = event;
	release->clock_event = clock_event;
	release->deadline = 0;
	release->armed = false;
	return k_clock_event_set_handler(clock_event, deferred_release_fire, release);
}

void k_deferred_release_lead_set(struct k_deferred_release *release, uint32_t lead)
{
	K_SPINLOCK(&release->lock) {
		release->lead = lead;
	}
}

int k_deferred_release_post(struct k_deferred_release *release,
			    uint32_t not_before, int deadline)
{
	uint32_t arm_at;
	bool publish = false;
	bool wait = false;
	int admitted;
	int ret = 0;

	if (release == NULL || release->event == NULL) {
		return -EINVAL;
	}
	arm_at = not_before - release->lead;
	admitted = deferred_release_admit(release, release->clock_event, arm_at, not_before,
					  deadline);
	K_SPINLOCK(&release->lock) {
		/* Replace any prior deferred release before publishing a new one. */
		k_clock_event_cancel(release->clock_event);
		ret = deferred_release_arm_locked(release, arm_at, not_before, deadline, &publish,
						  &wait);
	}
	if (wait) {
		deferred_release_wait_until(release->clock_event, arm_at);
		publish = true;
	}
	if (publish) {
		deferred_release_publish(release, not_before, deadline);
	}
	return (ret != 0) ? ret : admitted;
}

int k_deferred_release_post_selected(struct k_deferred_release *release,
				     struct k_clock_event *clock_event,
				     uint32_t not_before, int deadline,
				     uint32_t required_power_states,
				     uint32_t max_min_arm_cycles)
{
	const struct k_clock_event_provider *provider;
	uint32_t arm_at;
	bool publish = false;
	bool wait = false;
	int admitted = 0;
	int ret = 0;

	if (release == NULL || release->event == NULL || clock_event == NULL) {
		return -EINVAL;
	}

	provider = k_clock_event_provider_select(required_power_states,
					  max_min_arm_cycles);
	if (provider == NULL) {
		return -ENODEV;
	}

	arm_at = not_before - release->lead;
	K_SPINLOCK(&release->lock) {
		/* A selected post replaces any pending deferred publication. */
		if (release->clock_event != NULL) {
			k_clock_event_cancel(release->clock_event);
		}
		release->armed = false;
		if (provider->backend->event_init != NULL) {
			ret = provider->backend->event_init(provider->context, clock_event,
						    deferred_release_fire, release);
		} else {
			ret = k_clock_event_init(clock_event, provider->backend, provider->context,
						 deferred_release_fire, release);
		}
		if (ret != 0) {
			break;
		}
		release->clock_event = clock_event;
		ret = deferred_release_arm_locked(release, arm_at, not_before, deadline, &publish,
						  &wait);
	}
	if (ret == 0) {
		admitted = deferred_release_admit(release, clock_event, arm_at, not_before,
						  deadline);
	}
	if (wait) {
		deferred_release_wait_until(clock_event, arm_at);
		publish = true;
	}
	if (publish) {
		deferred_release_publish(release, not_before, deadline);
	}
	return (ret != 0) ? ret : admitted;
}

void k_deferred_release_cancel(struct k_deferred_release *release)
{
	__ASSERT(release != NULL, "deferred release is NULL");
	K_SPINLOCK(&release->lock) {
		release->armed = false;
		k_clock_event_cancel(release->clock_event);
	}
#ifdef CONFIG_SCHED_DEADLINE_ADMISSION
	if (release->event != NULL) {
		k_deadline_event_admit_cancel(release->event);
	}
#endif
}
