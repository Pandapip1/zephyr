/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * BURTC as a deep-sleep clock-event provider.
 *
 * BURTC has a single COMP register. This driver owns it and arbitrates a
 * small fixed set of clock events by earliest deadline. Because COMP is
 * shared, a latched COMP flag only says that some event matched: each event
 * is delivered only once CNT has reached its own deadline. An event already
 * due without its own match pends the IRQ instead of having a past value
 * written to COMP, and only the programmed COMP value is clamped to the
 * LFSYNC write floor.
 */

#define DT_DRV_COMPAT silabs_burtc_counter

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_silabs.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/clock_event.h>

#include <sl_hal_burtc.h>

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "exactly one BURTC instance must be enabled");

/* BURTC LFSYNC writes need several LF-clock cycles to take effect. */
#define MIN_DELAY_CYC 6U
#define SLOTS         CONFIG_SILABS_BURTC_CLOCK_EVENT_SLOTS

struct burtc_slot {
	struct k_clock_event *event;
	uint32_t deadline;
	uint32_t generation;
	bool armed;
};

static struct k_spinlock lock;
static struct burtc_slot slots[SLOTS];
static uint32_t frequency;
/* Last COMP value and the CNT at which it was written. */
static uint32_t compare_value;
static uint32_t compare_written_at;

static bool deadline_due(uint32_t deadline, uint32_t now)
{
	return (int32_t)(deadline - now) <= 0;
}

static void rearm_locked(uint32_t now)
{
	bool armed = false;
	bool due = false;
	uint32_t deadline = 0U;

	ARRAY_FOR_EACH_PTR(slots, slot) {
		if (!slot->armed) {
			continue;
		}
		if (deadline_due(slot->deadline, now)) {
			due = true;
		} else if (!armed || (slot->deadline - now) < (deadline - now)) {
			deadline = slot->deadline;
			armed = true;
		}
	}

	if (due) {
		/* Writing a past COMP would wait for a wrap: pend the IRQ instead. */
		sl_hal_burtc_set_interrupts(BURTC_IF_COMP);
		sl_hal_burtc_enable_interrupts(BURTC_IF_COMP);
		return;
	}
	if (!armed) {
		sl_hal_burtc_disable_interrupts(BURTC_IF_COMP);
		return;
	}
	if ((deadline - now) < MIN_DELAY_CYC) {
		/*
		 * Keep an earlier write that already lands in
		 * [deadline, now + MIN_DELAY_CYC]; rewriting it on every re-mux
		 * would keep pushing a near deadline back.
		 */
		if ((compare_value - compare_written_at) >= MIN_DELAY_CYC &&
		    (compare_value - now) <= MIN_DELAY_CYC && !deadline_due(compare_value, now) &&
		    (deadline - now) <= (compare_value - now)) {
			sl_hal_burtc_enable_interrupts(BURTC_IF_COMP);
			return;
		}
		deadline = now + MIN_DELAY_CYC;
	}
	compare_value = deadline;
	compare_written_at = now;
	sl_hal_burtc_set_compare(deadline);
	sl_hal_burtc_enable_interrupts(BURTC_IF_COMP);
}

static uint32_t burtc_now(void *context)
{
	ARG_UNUSED(context);

	return sl_hal_burtc_get_counter();
}

static uint32_t burtc_min_arm_cycles(void *context)
{
	ARG_UNUSED(context);

	return MIN_DELAY_CYC;
}

static uint32_t burtc_frequency_hz(void *context)
{
	ARG_UNUSED(context);

	return frequency;
}

static int burtc_arm_abs(void *context, uint32_t deadline, uint32_t generation)
{
	struct burtc_slot *slot = context;
	int ret = 0;

	K_SPINLOCK(&lock) {
		uint32_t now = sl_hal_burtc_get_counter();

		if ((int32_t)(deadline - now) < (int32_t)MIN_DELAY_CYC) {
			ret = -ETIME;
			K_SPINLOCK_BREAK;
		}
		slot->deadline = deadline;
		slot->generation = generation;
		slot->armed = true;
		rearm_locked(now);
	}
	return ret;
}

static void burtc_cancel(void *context)
{
	struct burtc_slot *slot = context;

	K_SPINLOCK(&lock) {
		slot->armed = false;
		rearm_locked(sl_hal_burtc_get_counter());
	}
}

static int burtc_event_init(void *context, struct k_clock_event *event,
			    k_clock_event_handler_t handler, void *user_data);

static const struct k_clock_event_backend burtc_backend = {
	.event_init = burtc_event_init,
	.now = burtc_now,
	.min_arm_cycles = burtc_min_arm_cycles,
	.arm_abs = burtc_arm_abs,
	.cancel = burtc_cancel,
	.frequency_hz = burtc_frequency_hz,
};

static int burtc_event_init(void *context, struct k_clock_event *event,
			    k_clock_event_handler_t handler, void *user_data)
{
	struct burtc_slot *slot = NULL;

	ARG_UNUSED(context);
	K_SPINLOCK(&lock) {
		ARRAY_FOR_EACH_PTR(slots, candidate) {
			if (candidate->event == event) {
				slot = candidate;
				break;
			}
			if (slot == NULL && candidate->event == NULL) {
				slot = candidate;
			}
		}
		if (slot != NULL) {
			slot->event = event;
		}
	}
	if (slot == NULL) {
		return -ENOMEM;
	}
	return k_clock_event_init(event, &burtc_backend, slot, handler, user_data);
}

static void burtc_isr(const void *arg)
{
	struct k_clock_event *events[SLOTS] = {0};
	uint32_t generations[SLOTS] = {0};

	ARG_UNUSED(arg);
	sl_hal_burtc_clear_interrupts(BURTC_IF_COMP);
	K_SPINLOCK(&lock) {
		uint32_t now = sl_hal_burtc_get_counter();

		ARRAY_FOR_EACH(slots, i) {
			if (slots[i].armed && deadline_due(slots[i].deadline, now)) {
				slots[i].armed = false;
				events[i] = slots[i].event;
				generations[i] = slots[i].generation;
			}
		}
		rearm_locked(now);
	}
	/* Handlers may arm or cancel again, so run them outside the mux lock. */
	ARRAY_FOR_EACH(events, i) {
		if (events[i] != NULL) {
			k_clock_event_fire(events[i], generations[i]);
		}
	}
}

static int burtc_clock_event_init(void)
{
	const struct device *clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0));
	static const struct silabs_clock_control_cmu_config clock_cfg =
		SILABS_DT_INST_CLOCK_CFG(0);
	const uint32_t prescaler = DT_INST_PROP(0, clock_div);
	sl_hal_burtc_init_t config = {
		prescaler, /* clkdiv */
		false,     /* stop during debug halt */
		false,     /* COMP is not the top value */
		true,      /* EM4 wakeup on compare match */
		false,     /* no EM4 wakeup on overflow */
	};
	uint32_t rate = 0U;
	int err;

	err = clock_control_on(clock_dev, (clock_control_subsys_t)&clock_cfg);
	if (err < 0 && err != -EALREADY) {
		return err;
	}
	err = clock_control_get_rate(clock_dev, (clock_control_subsys_t)&clock_cfg, &rate);
	if (err != 0 || rate == 0U) {
		return -EINVAL;
	}
	frequency = rate / prescaler;

	sl_hal_burtc_init(&config);
	sl_hal_burtc_enable();
	sl_hal_burtc_disable_interrupts(BURTC_IF_COMP | BURTC_IF_OF);
	sl_hal_burtc_clear_interrupts(BURTC_IF_COMP | BURTC_IF_OF);
	sl_hal_burtc_reset_counter();
	sl_hal_burtc_start();

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), burtc_isr, NULL, 0);
	irq_enable(DT_INST_IRQN(0));

	return k_clock_event_provider_register(&burtc_backend, NULL,
					       K_CLOCK_EVENT_POWER_ACTIVE |
					       K_CLOCK_EVENT_POWER_DEEP_SLEEP);
}

SYS_INIT(burtc_clock_event_init, PRE_KERNEL_1, CONFIG_SILABS_BURTC_CLOCK_EVENT_INIT_PRIORITY);
