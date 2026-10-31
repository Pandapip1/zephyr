/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Scheduler clock service.
 *
 * Every registered clock-event provider with a known frequency feeds one
 * logical, monotonic 64-bit timebase counting CONFIG_CLOCK_EVENT_SCHED_FREQUENCY:
 *
 * - While the CPU is in a state where the fastest active-only provider runs,
 *   logical time advances from that provider.
 * - Across any PM state deeper than PM_STATE_RUNTIME_IDLE it advances from
 *   the deep-sleep provider.
 *
 * The two are anchored together at a deep-provider cycle edge when the CPU
 * enters and leaves such a state, and re-anchored whenever a deep-provider
 * expiry is delivered (which happens on an edge).
 *
 * Clients arm absolute logical deadlines; the service keeps them in one
 * queue and programs each provider with a single compare, its earliest
 * pending target. A client whose lead covers the deep provider's arm floor
 * plus CONFIG_CLOCK_EVENT_SCHED_DEEP_WAKE_NS waits on the deep provider,
 * waking early by that budget and then finishing on the active provider.
 * Any other client waits on the active provider, and while one does, every
 * PM state deeper than PM_STATE_RUNTIME_IDLE is locked. Clients never name a
 * provider.
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/kernel/clock_event.h>
#include <zephyr/kernel/deadline_miss.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/util.h>

#define LOGICAL_HZ CONFIG_CLOCK_EVENT_SCHED_FREQUENCY
#define CLIENTS    CONFIG_CLOCK_EVENT_SCHED_CLIENTS

enum sched_stage {
	STAGE_IDLE,
	/* Waiting on the deep provider for wake_at. */
	STAGE_DEEP,
	/* Waiting on the active provider for deadline. */
	STAGE_ACTIVE,
};

struct sched_client {
	struct k_clock_event *event;
	uint64_t deadline;
	uint64_t wake_at;
	uint32_t generation;
	enum sched_stage stage;
};

/* A logical time and the provider count at the same instant. */
struct anchor {
	uint64_t logical;
	uint32_t count;
};

struct fired {
	struct k_clock_event *event;
	uint32_t generation;
};

static struct k_spinlock sched_lock;
static struct sched_client clients[CLIENTS];
static const struct k_clock_event_provider *deep_provider;
static const struct k_clock_event_provider *active_provider;
static struct k_clock_event deep_hw;
static struct k_clock_event active_hw;
static bool active_bound;
static uint32_t deep_hz;
static uint32_t active_hz;
/* Provider minimum leads (provider counts), fixed once a provider is adopted. */
static uint32_t deep_min;
static uint32_t active_min;
/* Derived on adoption, in logical cycles: client minimum lead, deep floor, wake budget. */
static uint32_t client_min;
static uint64_t deep_floor;
static uint64_t wake_budget;

/*
 * Provider count to logical cycles as integer part plus 32-bit fraction, so that the
 * time read on every k_cycle_get() is two 32x32 multiplies rather than a 64-bit division.
 */
struct scale {
	uint32_t ipart;
	uint32_t frac;
};
static struct scale deep_scale;
static struct scale active_scale;
/* Logical cycles to provider counts. */
static struct scale deep_inverse;
static struct scale active_inverse;

static struct scale inverse_for(uint32_t hz)
{
	return (struct scale){
		.ipart = (uint32_t)(hz / LOGICAL_HZ),
		.frac = (uint32_t)((((uint64_t)hz % LOGICAL_HZ) << 32) / LOGICAL_HZ),
	};
}

static struct scale scale_for(uint32_t hz)
{
	return (struct scale){
		.ipart = (uint32_t)(LOGICAL_HZ / hz),
		.frac = (uint32_t)((((uint64_t)LOGICAL_HZ % hz) << 32) / hz),
	};
}

static uint64_t scale_apply(struct scale s, uint32_t delta)
{
	return (uint64_t)delta * s.ipart + (((uint64_t)delta * s.frac) >> 32);
}
static struct anchor deep_anchor;
static struct anchor active_anchor;
/* Logical time currently advances from the active provider. */
static bool on_active;
static uint64_t last_now;
static bool pm_locked;
/* Provider count each hardware event is armed for, to skip identical re-arms. */
static struct {
	uint32_t target;
	bool armed;
} deep_armed, active_armed;

static const enum pm_state locked_states[] = {
	PM_STATE_SUSPEND_TO_IDLE, PM_STATE_STANDBY, PM_STATE_SUSPEND_TO_RAM,
	PM_STATE_SUSPEND_TO_DISK, PM_STATE_SOFT_OFF,
};

static uint32_t provider_now(const struct k_clock_event_provider *provider)
{
	return provider->backend->now(provider->context);
}

static uint32_t provider_min_arm(const struct k_clock_event_provider *provider)
{
	return provider->backend->min_arm_cycles(provider->context);
}

static uint32_t provider_hz(const struct k_clock_event_provider *provider)
{
	return provider->backend->frequency_hz(provider->context);
}

/* Convert @p cycles at @p from_hz to @p to_hz, rounding up. */
static uint64_t convert_ceil(uint64_t cycles, uint32_t from_hz, uint32_t to_hz)
{
	return DIV_ROUND_UP(cycles * to_hz, from_hz);
}

static uint64_t provider_lead_ns(const struct k_clock_event_provider *provider)
{
	return convert_ceil(provider_min_arm(provider), provider_hz(provider), NSEC_PER_SEC);
}

/* Logical time; caller holds sched_lock. */
static uint64_t now_locked(void)
{
	uint64_t now;

	if (on_active) {
		uint32_t count = provider_now(active_provider);
		uint32_t delta = count - active_anchor.count;

		/* Re-base well before the active counter's 32-bit wrap. */
		if (delta >= BIT(30)) {
			active_anchor.logical += scale_apply(active_scale, delta);
			active_anchor.count = count;
			delta = 0U;
		}
		now = active_anchor.logical + scale_apply(active_scale, delta);
	} else {
		uint32_t delta = provider_now(deep_provider) - deep_anchor.count;

		now = deep_anchor.logical + scale_apply(deep_scale, delta);
	}
	/* Hand-offs between sources must never move time backwards. */
	if (now < last_now) {
		now = last_now;
	}
	last_now = now;
	return now;
}

/* Wait for the next deep-provider edge; returns the count after it. */
static uint32_t deep_edge(void)
{
	uint32_t d0 = provider_now(deep_provider);
	uint32_t d1;

	do {
		d1 = provider_now(deep_provider);
	} while (d1 == d0);
	return d1;
}

/*
 * Anchor both providers to the current logical time at a deep edge. With
 * @p wait false the caller is already on an edge (a deep expiry).
 */
static void anchor_locked(bool wait)
{
	uint32_t deep = wait ? deep_edge() : provider_now(deep_provider);
	uint64_t logical = now_locked();

	deep_anchor = (struct anchor){.logical = logical, .count = deep};
	if (active_provider != NULL) {
		active_anchor = (struct anchor){.logical = logical,
						.count = provider_now(active_provider)};
	}
}

/*
 * Logical cycles to provider counts, rounded up. Deadlines are at most half a 32-bit span
 * ahead of the anchors, so the delta fits 32 bits; this runs on every arm, in whichever
 * thread schedules a timeout, so it multiplies instead of calling a 64-bit division.
 */
static uint32_t scale_apply_ceil(struct scale s, uint32_t delta)
{
	uint64_t fraction = (uint64_t)delta * s.frac;

	return delta * s.ipart + (uint32_t)(fraction >> 32) + (((uint32_t)fraction != 0U) ? 1U : 0U);
}

static uint32_t to_deep(uint64_t logical)
{
	/* Long without a deep expiry: re-anchor so the span fits (within a deep cycle). */
	if (logical - deep_anchor.logical > UINT32_MAX) {
		anchor_locked(false);
	}
	return deep_anchor.count +
	       scale_apply_ceil(deep_inverse, (uint32_t)(logical - deep_anchor.logical));
}

static uint32_t to_active(uint64_t logical)
{
	return active_anchor.count +
	       scale_apply_ceil(active_inverse, (uint32_t)(logical - active_anchor.logical));
}

static void pm_lock_set(bool lock)
{
	if (!IS_ENABLED(CONFIG_PM) || pm_locked == lock) {
		return;
	}
	pm_locked = lock;
	ARRAY_FOR_EACH(locked_states, i) {
		if (lock) {
			pm_policy_state_lock_get(locked_states[i], PM_ALL_SUBSTATES);
		} else {
			pm_policy_state_lock_put(locked_states[i], PM_ALL_SUBSTATES);
		}
	}
}

#ifdef CONFIG_CLOCK_EVENT_SCHED_TRACE
/* Recent service decisions, printed with a miss report. */
struct trace_entry {
	char op;
	int8_t client;
	uint8_t stage;
	uint64_t now;
	uint64_t value;
};
static struct trace_entry trace_ring[CONFIG_CLOCK_EVENT_SCHED_TRACE_ENTRIES];
static uint8_t trace_head;

static void trace(char op, const struct sched_client *c, uint64_t now, uint64_t value)
{
	trace_ring[trace_head] = (struct trace_entry){
		.op = op,
		.client = (c != NULL) ? (int8_t)(c - clients) : -1,
		.stage = (c != NULL) ? (uint8_t)c->stage : 0U,
		.now = now,
		.value = value,
	};
	trace_head = (trace_head + 1U) % ARRAY_SIZE(trace_ring);
}

static void trace_dump(void)
{
	for (size_t i = 0U; i < ARRAY_SIZE(trace_ring); i++) {
		const struct trace_entry *t =
			&trace_ring[(trace_head + i) % ARRAY_SIZE(trace_ring)];

		if (t->op != 0) {
			printk("  %c c%d s%u now %llu value %llu\n", t->op, t->client, t->stage,
			       (unsigned long long)t->now, (unsigned long long)t->value);
		}
	}
}
#else
#define trace(op, c, now, value) do { } while (false)
#define trace_dump() do { } while (false)
#endif

static int hw_arm(struct k_clock_event *hw, uint32_t target, uint32_t *armed_target, bool *armed)
{
	int ret;

	if (*armed && *armed_target == target) {
		trace('=', NULL, 0U, target);
		return 0;
	}
	ret = k_clock_event_arm_abs(hw, target);
	*armed = ret == 0;
	*armed_target = target;
	trace(ret != 0 ? 'E' : ((hw == &deep_hw) ? 'd' : 'a'), NULL, 0U, target);
	return ret;
}

static void hw_cancel(struct k_clock_event *hw, bool *armed)
{
	if (*armed) {
		k_clock_event_cancel(hw);
		*armed = false;
	}
}

/*
 * Advance every client whose target has passed, then program each provider
 * with its earliest remaining target. Fired clients are appended to @p out.
 * Caller holds sched_lock.
 */
/*
 * A clock event delivered later than its deadline plus the configured tolerance is a missed
 * deadline: count it, and panic when the build asks for that.
 */
static uint32_t late_count;
/* Which provider's expiry is being serviced ('D', 'A'), for miss reports. */
static char servicing;
/* First late delivery found while servicing, reported once sched_lock is released. */
static struct {
	const struct k_clock_event *event;
	uint64_t deadline;
	uint64_t now;
	enum sched_stage stage;
	bool valid;
} late;

static void check_lateness(const struct sched_client *c, uint64_t now, enum sched_stage stage)
{
	static const uint64_t tolerance =
		DIV_ROUND_UP((uint64_t)CONFIG_CLOCK_EVENT_SCHED_LATE_TOLERANCE_NS * LOGICAL_HZ,
			     NSEC_PER_SEC);

	if (now - c->deadline <= tolerance) {
		return;
	}
	late_count++;
	if (!late.valid) {
		late.event = c->event;
		late.deadline = c->deadline;
		late.now = now;
		late.stage = stage;
		late.valid = true;
	}
}

/* Hand a late delivery to the missed-deadline policy. Caller does not hold sched_lock. */
static void report_lateness(void)
{
	struct k_deadline_miss miss;
	bool valid = false;

	/* Racy peek: a late delivery is recorded under the lock this ISR just released. */
	if (!late.valid) {
		return;
	}
	miss = (struct k_deadline_miss){
		.kind = K_DEADLINE_MISS_DELIVERY,
	};
	K_SPINLOCK(&sched_lock) {
		if (late.valid) {
			miss.object = late.event;
			miss.release = (uint32_t)late.deadline;
			miss.deadline = (int)(uint32_t)late.deadline;
			miss.now = (uint32_t)late.now;
			late.valid = false;
			valid = true;
		}
	}
	if (!valid) {
		return;
	}
	miss.running_at_release = k_current_get();
	if (IS_ENABLED(CONFIG_SCHED_DEADLINE_MISS_PANIC)) {
		/* The service's own evidence, printed before the policy panics. */
		LOG_PANIC();
		printk("  deep anchor %llu@%u, active anchor %llu@%u, deep armed %d@%u, "
		       "active armed %d@%u\n",
		       (unsigned long long)deep_anchor.logical, deep_anchor.count,
		       (unsigned long long)active_anchor.logical, active_anchor.count,
		       deep_armed.armed, deep_armed.target, active_armed.armed,
		       active_armed.target);
		trace_dump();
	}
	z_deadline_miss_report(&miss);
}

uint32_t k_clock_event_sched_late_count(void)
{
	return late_count;
}

/*
 * Make the fastest provider expire as soon as it can, so that a client already
 * due is delivered from a provider IRQ. Caller holds sched_lock.
 */
static void arm_asap_locked(void)
{
	trace('x', NULL, 0U, 0U);
	if (active_bound) {
		/* Start well clear of the lead: each failed attempt costs an arm path. */
		uint32_t lead = 4U * active_min;

		pm_lock_set(true);
		/*
		 * Keep a compare that already expires within the widest margin, or has
		 * passed and is pending. Re-arming clears it and moves it out again: an
		 * arm path repeated faster than the lead, such as the idle thread's,
		 * would then postpone the expiry indefinitely.
		 */
		if (active_armed.armed &&
		    (int32_t)(active_armed.target - provider_now(active_provider)) <=
			    (int32_t)(8U * lead + 1U)) {
			return;
		}
		/*
		 * The provider re-reads its own counter, which a fast clock moves on
		 * while this code runs: widen the margin until the arm sticks.
		 */
		for (uint8_t attempt = 0U; attempt < 4U; attempt++, lead *= 2U) {
			if (hw_arm(&active_hw, provider_now(active_provider) + lead + 1U,
				   &active_armed.target, &active_armed.armed) == 0) {
				return;
			}
		}
	}
	if (deep_armed.armed &&
	    (int32_t)(deep_armed.target - provider_now(deep_provider)) <= (int32_t)(deep_min + 1U)) {
		return;
	}
	/* -EBUSY: the deep provider accepts an arm again within its lead. */
	while (hw_arm(&deep_hw, provider_now(deep_provider) + deep_min + 1U, &deep_armed.target,
		      &deep_armed.armed) == -EBUSY) {
	}
}

/*
 * Advance every client whose target has passed, then program each provider
 * with its earliest remaining target. With @p out, due clients are appended to
 * it for delivery once sched_lock is released. Without it (an arm or cancel,
 * which runs inside the generic layer's own event lock), nothing is delivered
 * here: a due client is left pending and a provider is made to expire as soon
 * as possible so that the provider IRQ delivers it. Caller holds sched_lock.
 */
static size_t service_locked(struct fired *out)
{
	size_t n = 0U;

	for (;;) {
		uint64_t now = now_locked();
		struct sched_client *deep_next = NULL;
		struct sched_client *active_next = NULL;
		bool due_pending = false;

		ARRAY_FOR_EACH_PTR(clients, c) {
			enum sched_stage was = c->stage;

			if (c->stage == STAGE_DEEP && c->wake_at <= now) {
				/* The remainder after the wake budget belongs to the active provider. */
				c->stage = active_bound ? STAGE_ACTIVE : STAGE_DEEP;
				c->wake_at = c->deadline;
				trace('T', c, now, c->deadline);
			}
			if (c->stage != STAGE_IDLE && c->deadline <= now) {
				if (out == NULL) {
					due_pending = true;
					trace('P', c, now, c->deadline);
					continue;
				}
				c->stage = STAGE_IDLE;
				check_lateness(c, now, was);
				out[n++] = (struct fired){c->event, c->generation};
				trace('F', c, now, c->deadline);
			}
			if (c->stage == STAGE_DEEP &&
			    (deep_next == NULL || c->wake_at < deep_next->wake_at)) {
				deep_next = c;
			} else if (c->stage == STAGE_ACTIVE &&
				   (active_next == NULL || c->deadline < active_next->deadline)) {
				active_next = c;
			}
		}
		if (due_pending) {
			arm_asap_locked();
			return n;
		}

		/* The active provider's target: its earliest client, or a deep stand-in. */
		struct sched_client *active_for = active_next;
		uint64_t active_target = (active_next != NULL) ? active_next->deadline : 0U;
		bool deep_deferred = false;

		if (deep_next == NULL) {
			hw_cancel(&deep_hw, &deep_armed.armed);
		} else {
			int ret = hw_arm(&deep_hw, to_deep(deep_next->wake_at), &deep_armed.target,
					 &deep_armed.armed);

			if (ret == -EBUSY && active_bound) {
				/*
				 * The deep provider is still applying an earlier arm and
				 * accepts a new one within its minimum lead. Until then the
				 * active provider stands in, for the wake or for the retry,
				 * whichever is first, and deep sleep stays locked: no deep
				 * compare would end it. The client remains a deep one.
				 */
				uint64_t retry = MIN(deep_next->wake_at, now + deep_floor);

				deep_deferred = true;
				if (active_for == NULL || retry < active_target) {
					active_for = deep_next;
					active_target = retry;
				}
			} else if (ret == -EBUSY) {
				/* Nothing else can wake it: retry, bounded by the deep lead. */
				continue;
			} else if (ret != 0) {
				/* Inside the deep floor: hand it to the active provider. */
				if (active_bound) {
					deep_next->stage = STAGE_ACTIVE;
					continue;
				}
				if (out == NULL) {
					arm_asap_locked();
					return n;
				}
				/* Without one, the deadline is closer than any provider can arm. */
				while (now_locked() < deep_next->deadline) {
				}
				continue;
			}
		}

		pm_lock_set(active_next != NULL || deep_deferred);
		if (active_for == NULL) {
			if (active_bound) {
				hw_cancel(&active_hw, &active_armed.armed);
			}
		} else if (hw_arm(&active_hw, to_active(active_target), &active_armed.target,
				  &active_armed.armed) != 0) {
			if (out == NULL) {
				arm_asap_locked();
				return n;
			}
			/* Closer than the active provider's lead: wait it out, never early. */
			while (now_locked() < active_target) {
			}
			continue;
		}
		return n;
	}
}

static void deliver(struct fired *fired, size_t n)
{
	for (size_t i = 0U; i < n; i++) {
		k_clock_event_fire(fired[i].event, fired[i].generation);
	}
}

/* Logical deadline nearest to now for a 32-bit client deadline. */
static uint64_t extend(uint32_t deadline, uint64_t now)
{
	return now + (int64_t)(int32_t)(deadline - (uint32_t)now);
}

static uint32_t sched_now(void *context)
{
	ARG_UNUSED(context);

	return (uint32_t)k_clock_event_sched_now();
}

static uint32_t sched_min_arm_cycles(void *context)
{
	ARG_UNUSED(context);
	return client_min;
}

static uint32_t sched_frequency_hz(void *context)
{
	ARG_UNUSED(context);

	return LOGICAL_HZ;
}

static int sched_arm_abs(void *context, uint32_t deadline, uint32_t generation)
{
	struct sched_client *c = context;
	int ret = 0;

	K_SPINLOCK(&sched_lock) {
		uint64_t now = now_locked();
		/* Without an active provider there is nothing to hand off to. */
		uint64_t wake = active_bound ? wake_budget : 0U;

		c->deadline = extend(deadline, now);
		c->generation = generation;
		if (c->deadline <= now) {
			c->stage = STAGE_IDLE;
			(void)service_locked(NULL);
			ret = -ETIME;
			K_SPINLOCK_BREAK;
		}
		if (!active_bound || c->deadline - now >= deep_floor + wake) {
			c->stage = STAGE_DEEP;
			c->wake_at = c->deadline - wake;
		} else {
			c->stage = STAGE_ACTIVE;
		}
		trace('R', c, now, c->deadline);
		(void)service_locked(NULL);
	}
	return ret;
}

static void sched_cancel(void *context)
{
	struct sched_client *c = context;

	K_SPINLOCK(&sched_lock) {
		c->stage = STAGE_IDLE;
		trace('C', c, 0U, 0U);
		(void)service_locked(NULL);
	}
}

static const struct k_clock_event_backend sched_backend = {
	.now = sched_now,
	.min_arm_cycles = sched_min_arm_cycles,
	.arm_abs = sched_arm_abs,
	.cancel = sched_cancel,
	.frequency_hz = sched_frequency_hz,
};

static void hw_expired(struct k_clock_event *event, uint32_t generation, void *user_data)
{
	struct fired fired[CLIENTS];
	size_t n;

	ARG_UNUSED(generation);
	ARG_UNUSED(user_data);
	K_SPINLOCK(&sched_lock) {
		if (event == &deep_hw) {
			/* The provider disarmed itself delivering this expiry. */
			deep_armed.armed = false;
			/* A deep expiry is delivered on a deep edge: re-anchor for free. */
			anchor_locked(false);
		} else {
			active_armed.armed = false;
		}
		servicing = (event == &deep_hw) ? 'D' : 'A';
		trace(servicing, NULL, now_locked(), 0U);
		n = service_locked(fired);
		servicing = 0;
	}
	deliver(fired, n);
	report_lateness();
}

/*
 * Adopt newly registered providers: the first deep-sleep one, and the
 * active-only one with the shortest arm lead in time if it beats the deep
 * provider. Caller holds sched_lock.
 */
static void select_locked(void)
{
	const struct k_clock_event_provider *provider;

	for (size_t i = 0U; (provider = k_clock_event_provider_get(i)) != NULL; i++) {
		if (provider->backend->frequency_hz == NULL) {
			continue;
		}
		if ((provider->power_states & K_CLOCK_EVENT_POWER_DEEP_SLEEP) != 0U) {
			if (deep_provider == NULL &&
			    k_clock_event_provider_bind(provider, &deep_hw, hw_expired, NULL) == 0) {
				deep_provider = provider;
				deep_hz = provider_hz(provider);
				deep_min = provider_min_arm(provider);
				deep_scale = scale_for(deep_hz);
				deep_inverse = inverse_for(deep_hz);
				/* One extra deep cycle covers rounding a target up to a deep edge. */
				deep_floor = convert_ceil(deep_min + 1U, deep_hz, LOGICAL_HZ);
				wake_budget = convert_ceil(CONFIG_CLOCK_EVENT_SCHED_DEEP_WAKE_NS,
							   NSEC_PER_SEC, LOGICAL_HZ);
				client_min = MAX(1U, (uint32_t)convert_ceil(deep_min, deep_hz,
									    LOGICAL_HZ));
				deep_anchor = (struct anchor){.logical = last_now,
							      .count = provider_now(provider)};
			}
		} else if (deep_provider != NULL && !active_bound &&
			   (provider->power_states & K_CLOCK_EVENT_POWER_ACTIVE) != 0U &&
			   provider_lead_ns(provider) < provider_lead_ns(deep_provider) &&
			   k_clock_event_provider_bind(provider, &active_hw, hw_expired, NULL) == 0) {
			active_provider = provider;
			active_hz = provider_hz(provider);
			active_min = provider_min_arm(provider);
			active_scale = scale_for(active_hz);
			active_inverse = inverse_for(active_hz);
			client_min = MAX(1U, (uint32_t)convert_ceil(active_min, active_hz, LOGICAL_HZ));
			/*
			 * A deep expiry re-anchors on the count it reads, some way into
			 * that deep cycle, and a target rounds up to a whole cycle: a deep
			 * compare may be two deep cycles late. With an active provider the
			 * deep provider only wakes the CPU, that much early, and the
			 * active provider delivers the deadline.
			 */
			wake_budget += convert_ceil(2U, deep_hz, LOGICAL_HZ);
			anchor_locked(true);
			on_active = true;
			active_bound = true;
		}
	}
}

void z_clock_event_sched_provider_added(void)
{
	K_SPINLOCK(&sched_lock) {
		select_locked();
	}
}

uint64_t k_clock_event_sched_now(void)
{
	uint64_t now = 0U;

	K_SPINLOCK(&sched_lock) {
		if (deep_provider != NULL) {
			now = now_locked();
		}
	}
	return now;
}

int k_clock_event_sched_init(struct k_clock_event *event, k_clock_event_handler_t handler,
			     void *user_data)
{
	struct sched_client *c = NULL;
	int ret = 0;

	if (event == NULL || handler == NULL) {
		return -EINVAL;
	}
	K_SPINLOCK(&sched_lock) {
		if (deep_provider == NULL) {
			ret = -ENODEV;
			K_SPINLOCK_BREAK;
		}
		ARRAY_FOR_EACH_PTR(clients, slot) {
			if (slot->event == event) {
				c = slot;
				break;
			}
			if (c == NULL && slot->event == NULL) {
				c = slot;
			}
		}
		if (c == NULL) {
			ret = -ENOMEM;
			K_SPINLOCK_BREAK;
		}
		c->event = event;
		c->stage = STAGE_IDLE;
	}
	if (ret != 0) {
		return ret;
	}
	return k_clock_event_init(event, &sched_backend, c, handler, user_data);
}

#ifdef CONFIG_PM
/*
 * The active provider may stop in any state deeper than runtime idle: carry
 * logical time on the deep provider across it.
 */
static void sched_pm_entry(enum pm_state state)
{
	if (state == PM_STATE_RUNTIME_IDLE || !active_bound) {
		return;
	}
	K_SPINLOCK(&sched_lock) {
		anchor_locked(true);
		on_active = false;
	}
}

static void sched_pm_exit(enum pm_state state)
{
	if (state == PM_STATE_RUNTIME_IDLE || !active_bound) {
		return;
	}
	K_SPINLOCK(&sched_lock) {
		anchor_locked(true);
		on_active = true;
	}
}

static struct pm_notifier sched_pm_notifier = {
	.state_entry = sched_pm_entry,
	.state_exit = sched_pm_exit,
};

static int clock_event_sched_pm_init(void)
{
	pm_notifier_register(&sched_pm_notifier);
	return 0;
}

SYS_INIT(clock_event_sched_pm_init, PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif
