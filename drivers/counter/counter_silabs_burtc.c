/*
 * Copyright (c) 2026, Shontal Biton
 *
 * Based on:
 *  counter_gecko_rtcc.c Copyright (c) 2019, Piotr Mienkowski
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_burtc_counter

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/counter/silabs_burtc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_silabs.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/clock.h>

#include <sl_hal_burtc.h>

LOG_MODULE_REGISTER(counter_silabs_burtc, CONFIG_COUNTER_LOG_LEVEL);

#define BURTC_MAX_VALUE (_BURTC_CNT_MASK)
#define BURTC_ALARM_NUM 1

#if defined(CONFIG_GECKO_BURTC_TIMER)
#define MAX_TIMEOUT_CYC (UINT32_MAX >> 1)
/* BURTC LFSYNC writes need several LF-clock cycles to take effect. */
#define MIN_DELAY_CYC 6U

extern unsigned int z_clock_hw_cycles_per_sec;
#endif

struct counter_silabs_config {
	struct counter_config_info info;
	const struct device *clock_dev;
	const struct silabs_clock_control_cmu_config clock_cfg;
	void (*irq_config)(void);
	uint32_t prescaler;
};

struct counter_silabs_data {
#if defined(CONFIG_COUNTER_SILABS_BURTC_ALARM)
	counter_alarm_callback_t callback;
	void *user_data;
	bool alarm_armed;
	uint32_t alarm_ticks;
#else
	counter_top_callback_t top_callback;
	void *top_user_data;
#endif
	struct k_spinlock lock;
#if defined(CONFIG_GECKO_BURTC_TIMER)
	atomic_t last_count;
	uint32_t cyc_per_tick;
	uint32_t max_timeout_ticks;
	uint32_t system_deadline;
	bool system_deadline_armed;
	/*
	 * A relative Counter alarm can be more than half a 32-bit wrap away.
	 * Keep track of whether COMP was explicitly programmed for that alarm:
	 * after the match, CNT can already be numerically past the target, so a
	 * signed subtraction alone cannot tell it apart from an alarm another
	 * wrap away.
	 */
	bool compare_is_alarm;
	#if defined(CONFIG_COUNTER_SILABS_BURTC_CLOCK_EVENT)
	/* One generic client shares the single COMP mux with Counter and sysclock. */
	struct k_clock_event *clock_event;
	uint32_t clock_event_deadline;
	uint32_t clock_event_generation;
	bool clock_event_armed;
	#endif
	bool system_clock_initialized;
#endif
};

#if defined(CONFIG_GECKO_BURTC_TIMER)
static const struct device *system_clock_dev;

static bool deadline_due(uint32_t deadline, uint32_t now)
{
	return (int32_t)(deadline - now) <= 0;
}

static bool deadline_first(uint32_t candidate, uint32_t current, uint32_t now)
{
	return (candidate - now) <= (current - now);
}

/* The one hardware compare is shared by the Counter API and the kernel. */
static void counter_silabs_rearm_compare(struct counter_silabs_data *data, uint32_t now)
{
	bool armed = false;
	bool alarm_selected = false;
	#if defined(CONFIG_COUNTER_SILABS_BURTC_CLOCK_EVENT)
	bool clock_event_selected = false;
	#endif
	uint32_t deadline = 0;

	if (data->system_deadline_armed) {
		deadline = data->system_deadline;
		armed = true;
	}
#if defined(CONFIG_COUNTER_SILABS_BURTC_ALARM)
	if (data->alarm_armed) {
		/*
		 * The system-clock maximum timeout is below half a wrap.  A
		 * relative Counter alarm farther away is deliberately left behind
		 * that bounded wake-up until it enters the unambiguous half-wrap
		 * window.  It must not be treated as an already-expired deadline.
		 */
		uint32_t alarm_delta = data->alarm_ticks - now;

		if (alarm_delta < MAX_TIMEOUT_CYC &&
		    (!armed || deadline_first(data->alarm_ticks, deadline, now))) {
			deadline = data->alarm_ticks;
			armed = true;
			alarm_selected = true;
		}
	}
#endif
	#if defined(CONFIG_COUNTER_SILABS_BURTC_CLOCK_EVENT)
	if (data->clock_event_armed) {
		uint32_t event_delta = data->clock_event_deadline - now;

		if (event_delta < MAX_TIMEOUT_CYC &&
		    (!armed || deadline_first(data->clock_event_deadline, deadline, now))) {
			deadline = data->clock_event_deadline;
			armed = true;
			clock_event_selected = true;
			alarm_selected = false;
		}
	}
	#endif

	if (armed) {
		/*
		 * A BURTC LFSYNC write needs several LF-clock cycles.  In
		 * particular, Counter's EXPIRE_WHEN_LATE path must not set now+1
		 * and accidentally turn a late callback into a full-wrap wait.
		 */
		/*
		 * An alarm deadline has already been normalized at submission time.
		 * Do not keep moving it to now + MIN_DELAY_CYC every time the system
		 * clock re-arms COMP: that can starve a short client alarm forever.
		 */
		if (!alarm_selected
		#if defined(CONFIG_COUNTER_SILABS_BURTC_CLOCK_EVENT)
		    && !clock_event_selected
		#endif
		    &&
		    (deadline_due(deadline, now) ||
		     (deadline - now) < MIN_DELAY_CYC)) {
			deadline = now + MIN_DELAY_CYC;
		}
		data->compare_is_alarm = alarm_selected;
		sl_hal_burtc_set_compare(deadline);
		sl_hal_burtc_enable_interrupts(BURTC_IF_COMP);
	} else {
		data->compare_is_alarm = false;
		sl_hal_burtc_disable_interrupts(BURTC_IF_COMP);
	}
}

#if defined(CONFIG_COUNTER_SILABS_BURTC_CLOCK_EVENT)
static uint32_t counter_silabs_clock_event_now(void *context)
{
	ARG_UNUSED(context);
	return sl_hal_burtc_get_counter();
}

static uint32_t counter_silabs_clock_event_min_arm_cycles(void *context)
{
	ARG_UNUSED(context);
	return MIN_DELAY_CYC;
}

static int counter_silabs_clock_event_arm_abs(void *context, uint32_t deadline,
					      uint32_t generation)
{
	const struct device *dev = context;
	struct counter_silabs_data *data = dev->data;
	uint32_t now;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->lock);
	now = sl_hal_burtc_get_counter();
	if ((int32_t)(deadline - now) < MIN_DELAY_CYC) {
		k_spin_unlock(&data->lock, key);
		return -ETIME;
	}

	data->clock_event_deadline = deadline;
	data->clock_event_generation = generation;
	data->clock_event_armed = true;
	counter_silabs_rearm_compare(data, now);
	k_spin_unlock(&data->lock, key);
	return 0;
}

static void counter_silabs_clock_event_cancel(void *context)
{
	const struct device *dev = context;
	struct counter_silabs_data *data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->lock);
	data->clock_event_armed = false;
	counter_silabs_rearm_compare(data, sl_hal_burtc_get_counter());
	k_spin_unlock(&data->lock, key);
}

static const struct k_clock_event_backend counter_silabs_clock_event_backend = {
	.now = counter_silabs_clock_event_now,
	.min_arm_cycles = counter_silabs_clock_event_min_arm_cycles,
	.arm_abs = counter_silabs_clock_event_arm_abs,
	.cancel = counter_silabs_clock_event_cancel,
};

int counter_silabs_burtc_clock_event_init(const struct device *dev,
					  struct k_clock_event *event,
					  k_clock_event_handler_t handler,
					  void *user_data)
{
	struct counter_silabs_data *data;
	k_spinlock_key_t key;
	int ret;

	if (!device_is_ready(dev) || event == NULL) {
		return -EINVAL;
	}

	data = dev->data;
	key = k_spin_lock(&data->lock);
	if (data->clock_event != NULL && data->clock_event != event) {
		k_spin_unlock(&data->lock, key);
		return -EBUSY;
	}
	data->clock_event = event;
	k_spin_unlock(&data->lock, key);

	ret = k_clock_event_init(event, &counter_silabs_clock_event_backend,
				 (void *)dev, handler, user_data);
	if (ret != 0) {
		key = k_spin_lock(&data->lock);
		if (data->clock_event == event) {
			data->clock_event = NULL;
		}
		k_spin_unlock(&data->lock, key);
	}
	return ret;
}
#endif

void sys_clock_set_timeout(uint32_t ticks, bool idle)
{
	const struct device *dev = system_clock_dev;
	struct counter_silabs_data *data;
	uint32_t curr;
	uint32_t prev;
	uint32_t unannounced;
	uint32_t to_announce;
	k_spinlock_key_t key;

	ARG_UNUSED(idle);
	if (dev == NULL || !IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	data = dev->data;
	/* Zero means the nearest upcoming tick. */
	ticks = CLAMP(ticks, 1U, data->max_timeout_ticks) - 1U;
	key = k_spin_lock(&data->lock);
	curr = sl_hal_burtc_get_counter();
	prev = atomic_get(&data->last_count);
	unannounced = (curr - prev) / data->cyc_per_tick;
	to_announce = MIN(unannounced + ticks + 1U, data->max_timeout_ticks);
	data->system_deadline = prev + to_announce * data->cyc_per_tick;
	if ((data->system_deadline - curr) < MIN_DELAY_CYC) {
		data->system_deadline += data->cyc_per_tick;
	}
	data->system_deadline_armed = true;
	counter_silabs_rearm_compare(data, curr);
	k_spin_unlock(&data->lock, key);
}

uint32_t sys_clock_elapsed(void)
{
	const struct device *dev = system_clock_dev;
	const struct counter_silabs_data *data;

	if (dev == NULL) {
		return 0;
	}
	data = dev->data;
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL) || !data->system_clock_initialized) {
		return 0;
	}
	return (sl_hal_burtc_get_counter() - atomic_get(&data->last_count)) / data->cyc_per_tick;
}

uint32_t sys_clock_cycle_get_32(void)
{
	if (system_clock_dev == NULL ||
	    !((const struct counter_silabs_data *)system_clock_dev->data)->system_clock_initialized) {
		return 0;
	}
	return sl_hal_burtc_get_counter();
}
#endif

static int counter_silabs_start(const struct device *dev)
{
	ARG_UNUSED(dev);

	sl_hal_burtc_start();
	return 0;
}

static int counter_silabs_stop(const struct device *dev)
{
#if defined(CONFIG_GECKO_BURTC_TIMER)
	/* The public Counter is also Zephyr's monotonic system-clock source. */
	ARG_UNUSED(dev);
	return -ENOTSUP;
#else
	ARG_UNUSED(dev);

	sl_hal_burtc_stop();
	return 0;
#endif
}

static int counter_silabs_get_value(const struct device *dev, uint32_t *ticks)
{
	ARG_UNUSED(dev);

	*ticks = sl_hal_burtc_get_counter();
	return 0;
}

static uint32_t counter_silabs_get_top_value(const struct device *dev)
{
	ARG_UNUSED(dev);
#if defined(CONFIG_GECKO_BURTC_TIMER)
	/* COMP is a transient muxed deadline, not the free-running counter top. */
	return BURTC_MAX_VALUE;
#else
	return sl_hal_burtc_get_compare();
#endif
}

static uint32_t counter_silabs_get_freq(const struct device *dev)
{
	const struct counter_silabs_config *const config = dev->config;
	const struct device *clock_dev = config->clock_dev;
	uint32_t freq_hz = 0;

	if (clock_control_get_rate(clock_dev, (clock_control_subsys_t)&config->clock_cfg,
				   &freq_hz)) {
		return 0;
	}
	return freq_hz / config->prescaler;
}

#if defined(CONFIG_COUNTER_SILABS_BURTC_ALARM)
static int counter_silabs_set_alarm(const struct device *dev, uint8_t chan_id,
				    const struct counter_alarm_cfg *alarm_cfg)
{
	struct counter_silabs_data *const dev_data = dev->data;
	uint32_t alarm_ticks = alarm_cfg->ticks;
	uint32_t now;
	k_spinlock_key_t key;

	/* BURTC has only one channel */
	if (chan_id != 0) {
		return -EINVAL;
	}

	key = k_spin_lock(&dev_data->lock);
	/* No need to validate top value because burtc doesn't support it. */
	if (dev_data->alarm_armed) {
		k_spin_unlock(&dev_data->lock, key);
		return -EINVAL;
	}

	now = sl_hal_burtc_get_counter();
	if (!(alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE)) {
		alarm_ticks += now;
	} else {
		/*
		 * BURTC compares are one-shot absolute matches.  A compare value
		 * already behind CNT would otherwise wait for the 32-bit wrap,
		 * even though the counter API explicitly requests late expiry.
		 */
		if ((int32_t)(alarm_ticks - now) <= 0) {
			if (!(alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE)) {
				k_spin_unlock(&dev_data->lock, key);
				return -ETIME;
			}
			/*
			 * Program the earliest deadline the LFSYNC path can honor. The
			 * shared system-clock re-arm must retain this normalized value;
			 * otherwise repeated now + MIN_DELAY_CYC rewrites can starve it.
			 */
			alarm_ticks = now + MIN_DELAY_CYC;
		}
	}
	if ((alarm_ticks - now) < MIN_DELAY_CYC) {
		/* A future deadline can also be too close for a synchronized write. */
		alarm_ticks = now + MIN_DELAY_CYC;
	}

	dev_data->callback = alarm_cfg->callback;
	dev_data->user_data = alarm_cfg->user_data;
	dev_data->alarm_ticks = alarm_ticks;
	dev_data->alarm_armed = true;

#if defined(CONFIG_GECKO_BURTC_TIMER)
	counter_silabs_rearm_compare(dev_data, now);
#else
	sl_hal_burtc_clear_interrupts(BURTC_IF_COMP);
	sl_hal_burtc_set_compare(alarm_ticks);
	sl_hal_burtc_enable_interrupts(BURTC_IF_COMP);
#endif
	k_spin_unlock(&dev_data->lock, key);
	LOG_DBG("set alarm: %u", alarm_ticks);
	return 0;
}

static int counter_silabs_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	struct counter_silabs_data *const dev_data = dev->data;
	k_spinlock_key_t key;

	/* BURTC has only one channel */
	if (chan_id != 0) {
		return -EINVAL;
	}

	key = k_spin_lock(&dev_data->lock);
	dev_data->callback = NULL;
	dev_data->user_data = NULL;
	dev_data->alarm_armed = false;

#if defined(CONFIG_GECKO_BURTC_TIMER)
	counter_silabs_rearm_compare(dev_data, sl_hal_burtc_get_counter());
#else
	/* Disable the compare interrupt */
	sl_hal_burtc_disable_interrupts(BURTC_IF_COMP);
	sl_hal_burtc_clear_interrupts(BURTC_IF_COMP);
	sl_hal_burtc_set_compare(0);
#endif
	k_spin_unlock(&dev_data->lock, key);
	LOG_DBG("cancel alarm");
	return 0;
}

static int counter_silabs_set_top_value(const struct device *dev, const struct counter_top_cfg *cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cfg);
	return -ENOTSUP;
}

#else
static int counter_silabs_set_top_value(const struct device *dev, const struct counter_top_cfg *cfg)
{
#if defined(CONFIG_GECKO_BURTC_TIMER)
	ARG_UNUSED(dev);
	ARG_UNUSED(cfg);
	return -ENOTSUP;
#else
	struct counter_silabs_data *const dev_data = dev->data;
	uint32_t ticks;
	uint32_t flags;
	int err = 0;

	sl_hal_burtc_clear_interrupts(BURTC_IF_COMP);

	dev_data->top_callback = cfg->callback;
	dev_data->top_user_data = cfg->user_data;
	ticks = cfg->ticks;
	flags = cfg->flags;

	if (!(flags & COUNTER_TOP_CFG_DONT_RESET)) {
		sl_hal_burtc_reset_counter();
	} else if (sl_hal_burtc_get_counter() > ticks) {
		if (flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) {
			sl_hal_burtc_reset_counter();
		} else {
			err = -ETIME;
		}
	}

	sl_hal_burtc_set_compare(ticks);
	LOG_DBG("set top value: %u", ticks);

	/* Enable the compare interrupt */
	sl_hal_burtc_enable_interrupts(BURTC_IF_COMP);
	return err;
#endif
}

static int counter_silabs_set_alarm(const struct device *dev, uint8_t chan_id,
				    const struct counter_alarm_cfg *alarm_cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan_id);
	ARG_UNUSED(alarm_cfg);
	return -ENOTSUP;
}

static int counter_silabs_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan_id);
	return -ENOTSUP;
}
#endif

static uint32_t counter_silabs_get_pending_int(const struct device *dev)
{
	ARG_UNUSED(dev);
	return sl_hal_burtc_get_enabled_pending_interrupts() != 0;
}

static int counter_silabs_init(const struct device *dev)
{
	int err = 0;
	const struct counter_silabs_config *const burtc_cfg = dev->config;
	struct counter_silabs_data *const data = dev->data;

	sl_hal_burtc_init_t burtc_config = {
		burtc_cfg->prescaler, /* Set clkdiv. */
		false,                /* Disable RTC during debug halt. */
#if defined(CONFIG_COUNTER_SILABS_BURTC_ALARM)
		false, /* Don't set compare value as the top value. */
		true,  /* Enable EM4 wakeup on compare match. */
		false, /* Disable EM4 wakeup on overflow. */
#else
		true,  /* Set compare value as the top value. */
		false, /* Disable EM4 wakeup on compare match. */
		true,  /* Enable EM4 wakeup on overflow. */
#endif
	};

	/* Enable BURTC clock */
	err = clock_control_on(burtc_cfg->clock_dev, (clock_control_subsys_t)&burtc_cfg->clock_cfg);
	if (err < 0 && err != -EALREADY) {
		return err;
	}

	/* Initialize BURTC */
	sl_hal_burtc_init(&burtc_config);
	sl_hal_burtc_enable();
	/* A system clock must run before any Counter client has called start(). */
	sl_hal_burtc_start();

	/* Disable module's internal interrupt sources */
	sl_hal_burtc_disable_interrupts(BURTC_IF_COMP | BURTC_IF_OF);
	sl_hal_burtc_clear_interrupts(BURTC_IF_COMP | BURTC_IF_OF);

	/* Clear the counter */
	sl_hal_burtc_reset_counter();

#if defined(CONFIG_GECKO_BURTC_TIMER)
	z_clock_hw_cycles_per_sec = counter_silabs_get_freq(dev);
	if (z_clock_hw_cycles_per_sec == 0U) {
		return -EINVAL;
	}
	BUILD_ASSERT(CONFIG_SYS_CLOCK_TICKS_PER_SEC > 0,
		     "Invalid CONFIG_SYS_CLOCK_TICKS_PER_SEC value");
	data->cyc_per_tick = z_clock_hw_cycles_per_sec / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	__ASSERT(data->cyc_per_tick >= MIN_DELAY_CYC,
		 "%u cycle-long tick is too short to be scheduled (min is %u)",
		 data->cyc_per_tick, MIN_DELAY_CYC);
	data->max_timeout_ticks = MAX_TIMEOUT_CYC / data->cyc_per_tick;
	atomic_set(&data->last_count, 0);
	data->system_deadline = data->cyc_per_tick;
	data->system_deadline_armed = true;
	data->system_clock_initialized = true;
	system_clock_dev = dev;
#endif

	/* Configure & enable module interrupts */
	burtc_cfg->irq_config();
#if defined(CONFIG_GECKO_BURTC_TIMER)
	counter_silabs_rearm_compare(data, sl_hal_burtc_get_counter());
#endif
	LOG_INF("Device %s initialized", dev->name);
	return 0;
}

static DEVICE_API(counter, counter_silabs_driver_api) = {
	.start = counter_silabs_start,
	.stop = counter_silabs_stop,
	.get_value = counter_silabs_get_value,
	.set_alarm = counter_silabs_set_alarm,
	.cancel_alarm = counter_silabs_cancel_alarm,
	.set_top_value = counter_silabs_set_top_value,
	.get_pending_int = counter_silabs_get_pending_int,
	.get_top_value = counter_silabs_get_top_value,
	.get_freq = counter_silabs_get_freq,
};

void counter_silabs_isr_handler(const struct device *dev)
{
	struct counter_silabs_data *const burtc_data = dev->data;
	uint32_t flags = sl_hal_burtc_get_enabled_pending_interrupts();
#if defined(CONFIG_COUNTER_SILABS_BURTC_ALARM)
	counter_alarm_callback_t alarm_callback;
	void *alarm_user_data;
#endif
#if defined(CONFIG_COUNTER_SILABS_BURTC_ALARM) || defined(CONFIG_GECKO_BURTC_TIMER)
	uint32_t now;
	k_spinlock_key_t key;
#endif
#if defined(CONFIG_GECKO_BURTC_TIMER)
	uint32_t prev;
	uint32_t unannounced;
#endif
	#if defined(CONFIG_COUNTER_SILABS_BURTC_CLOCK_EVENT)
	struct k_clock_event *clock_event = NULL;
	uint32_t clock_event_generation = 0U;
	#endif

	sl_hal_burtc_clear_interrupts(flags);
	if (!(flags & BURTC_IF_COMP)) {
		return;
	}

#if defined(CONFIG_COUNTER_SILABS_BURTC_ALARM) || defined(CONFIG_GECKO_BURTC_TIMER)
	now = sl_hal_burtc_get_counter();
	key = k_spin_lock(&burtc_data->lock);
#endif

#if defined(CONFIG_COUNTER_SILABS_BURTC_ALARM)
	alarm_callback = NULL;
	alarm_user_data = NULL;
	if (burtc_data->alarm_armed
#if defined(CONFIG_GECKO_BURTC_TIMER)
	    && burtc_data->compare_is_alarm
#endif
	) {
		alarm_callback = burtc_data->callback;
		alarm_user_data = burtc_data->user_data;
		burtc_data->callback = NULL;
		burtc_data->user_data = NULL;
		burtc_data->alarm_armed = false;
	}
#endif
#if defined(CONFIG_GECKO_BURTC_TIMER)
	#if defined(CONFIG_COUNTER_SILABS_BURTC_CLOCK_EVENT)
	/* The generic event deadline is within its half-wrap validity window. */
	if (burtc_data->clock_event_armed &&
	    deadline_due(burtc_data->clock_event_deadline, now)) {
		clock_event = burtc_data->clock_event;
		clock_event_generation = burtc_data->clock_event_generation;
		burtc_data->clock_event_armed = false;
	}
	#endif
	prev = atomic_get(&burtc_data->last_count);
	unannounced = (now - prev) / burtc_data->cyc_per_tick;
	atomic_set(&burtc_data->last_count,
		   prev + unannounced * burtc_data->cyc_per_tick);
	if (burtc_data->system_deadline_armed &&
	    deadline_due(burtc_data->system_deadline, now)) {
		if (IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
			burtc_data->system_deadline_armed = false;
		} else {
			burtc_data->system_deadline = prev + burtc_data->cyc_per_tick;
			if ((burtc_data->system_deadline - now) < MIN_DELAY_CYC) {
				burtc_data->system_deadline += burtc_data->cyc_per_tick;
			}
		}
	}
	counter_silabs_rearm_compare(burtc_data, now);
#endif
#if defined(CONFIG_COUNTER_SILABS_BURTC_ALARM) || defined(CONFIG_GECKO_BURTC_TIMER)
	k_spin_unlock(&burtc_data->lock, key);
#endif

#if defined(CONFIG_GECKO_BURTC_TIMER)
	sys_clock_announce(unannounced);
#endif
	#if defined(CONFIG_COUNTER_SILABS_BURTC_CLOCK_EVENT)
	if (clock_event != NULL) {
		/* Do not take the generic event lock while holding the BURTC mux lock. */
		k_clock_event_fire(clock_event, clock_event_generation);
	}
	#endif
#if defined(CONFIG_COUNTER_SILABS_BURTC_ALARM)
	if (alarm_callback) {
		alarm_callback(dev, 0, now, alarm_user_data);
	}
#else
	if (burtc_data->top_callback) {
		burtc_data->top_callback(dev, burtc_data->top_user_data);
	}
#endif
}

#define BURTC_INIT(n)                                                                              \
	static void counter_silabs_isr_##n(const void *arg)                                    \
	{                                                                                          \
		counter_silabs_isr_handler(arg);                                                   \
	}                                                                                          \
                                                                                                   \
	static void counter_silabs_irq_config_##n(void)                                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),                             \
				    counter_silabs_isr_##n, DEVICE_DT_INST_GET(n), 0);             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static struct counter_silabs_data counter_silabs_data_##n;                                 \
                                                                                                   \
	static const struct counter_silabs_config counter_silabs_config_##n = {                    \
		.info =                                                                            \
			{                                                                          \
				.max_top_value = BURTC_MAX_VALUE,                                  \
				.flags = COUNTER_CONFIG_INFO_COUNT_UP,                             \
				.channels = BURTC_ALARM_NUM,                                       \
			},                                                                         \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_cfg = SILABS_DT_INST_CLOCK_CFG(n),                                          \
		.irq_config = counter_silabs_irq_config_##n,                                       \
		.prescaler = DT_INST_PROP(n, clock_div),                                           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, counter_silabs_init, NULL, &counter_silabs_data_##n,              \
			      &counter_silabs_config_##n, PRE_KERNEL_1,                            \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_silabs_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BURTC_INIT)
