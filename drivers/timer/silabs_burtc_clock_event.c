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
 * written to COMP.
 *
 * COMP is an LF-synchronized register (EFR32xG22 reference manual, BURTC
 * register access; timings measured on an EFR32BG22 with BURTC on LFXO):
 *
 * - A COMP write takes effect at the third LF edge after the bus write.
 *   SYNCBUSY.COMP is set until then, 2.0 to 3.0 LF periods (61 to 92 us),
 *   and COMP reads the old value meanwhile.
 * - IF.COMP sets on the edge where CNT advances from COMP to COMP + 1.
 * - COMP = CNT + d (CNT read just before the write) fires for d >= 3; for
 *   d <= 2 it waits for the 32-bit wrap.
 * - Rewriting COMP while CNT reads COMP - k still delivers the old match for
 *   k <= 2.
 *
 * So COMP is programmed one count before the deadline, and nothing here ever
 * waits for SYNCBUSY: sl_hal_burtc_set_compare() would spin up to two sync
 * periods (183 us) with interrupts locked, which is longer than the
 * scheduler's lateness tolerance. While a write is still synchronizing, a
 * target no earlier than the pending match keeps it (that match fires first
 * and the IRQ re-arms); an earlier target is refused with -EBUSY, to be
 * retried by the caller once the write has landed, within MIN_DELAY_CYC.
 * Old or early matches are harmless: the IRQ re-checks every deadline.
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

/* IF.COMP sets as CNT leaves COMP: program COMP this many counts before the deadline. */
#define COMP_MATCH_OFFSET 1U
/* A COMP write takes effect at this LF edge after the write. */
#define COMP_SYNC_EDGES   3U
/* CNT may advance once between the read that decides a target and the COMP write. */
#define COMP_READ_SKEW    1U
/*
 * Minimum lead from the CNT read to a deadline: the written COMP must still
 * be at least COMP_SYNC_EDGES ahead of CNT when the write is issued. It is
 * also the longest a refused (-EBUSY) caller waits before the synchronizing
 * write has landed, since every write is issued with at least this lead.
 */
#define MIN_DELAY_CYC     (COMP_MATCH_OFFSET + COMP_SYNC_EDGES + COMP_READ_SKEW)
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
/* Deadline whose match the last COMP write programs. */
static uint32_t compare_deadline;
static bool compare_written;

static bool deadline_due(uint32_t deadline, uint32_t now)
{
	return (int32_t)(deadline - now) <= 0;
}

static bool compare_syncing(void)
{
	return (BURTC->SYNCBUSY & BURTC_SYNCBUSY_COMP) != 0U;
}

/*
 * Program COMP for the earliest armed deadline. Returns -EBUSY when that
 * needs a COMP write while an earlier one is still synchronizing and the
 * pending match would come after it; the hardware is then left as it was.
 */
static int rearm_locked(uint32_t now)
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
		return 0;
	}
	if (!armed) {
		/* A synchronizing or stale match may still latch IF; the IRQ ignores it. */
		sl_hal_burtc_disable_interrupts(BURTC_IF_COMP);
		return 0;
	}
	/*
	 * Keep a programmed match for a deadline still ahead and no later than
	 * this one when it is this deadline's, when a write cannot replace it
	 * yet, or when a rewrite would no longer cancel it: it fires first and
	 * the IRQ re-arms. Otherwise write the exact deadline, so that a far
	 * early match does not cost a spurious wake.
	 */
	if (compare_written && !deadline_due(compare_deadline, now) &&
	    (compare_deadline - now) <= (deadline - now) &&
	    (compare_deadline == deadline || compare_syncing() ||
	     (compare_deadline - now) < MIN_DELAY_CYC)) {
		sl_hal_burtc_enable_interrupts(BURTC_IF_COMP);
		return 0;
	}
	if (compare_syncing()) {
		return -EBUSY;
	}
	/* A nearer remaining deadline (after an IRQ or cancel) is delayed to the lead, never early. */
	if ((deadline - now) < MIN_DELAY_CYC) {
		deadline = now + MIN_DELAY_CYC;
	}
	compare_deadline = deadline;
	compare_written = true;
	/* Not sl_hal_burtc_set_compare(): it spins on SYNCBUSY before and after the write. */
	BURTC->COMP = deadline - COMP_MATCH_OFFSET;
	sl_hal_burtc_enable_interrupts(BURTC_IF_COMP);
	return 0;
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
		ret = rearm_locked(now);
		if (ret != 0) {
			slot->armed = false;
		}
	}
	return ret;
}

static void burtc_cancel(void *context)
{
	struct burtc_slot *slot = context;

	K_SPINLOCK(&lock) {
		slot->armed = false;
		/* Only later deadlines remain: a pending match covers them (-EBUSY is moot). */
		(void)rearm_locked(sl_hal_burtc_get_counter());
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
		/*
		 * -EBUSY: a write still synchronizing holds a pending match, which
		 * fires and re-arms here; every armed deadline was accepted no
		 * earlier than the match programmed for it.
		 */
		(void)rearm_locked(now);
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
