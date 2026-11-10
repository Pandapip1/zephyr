/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel/deadline_miss.h>
#include <zephyr/irq.h>
#include <zephyr/ztest.h>
#include <cmsis_core.h>

/* A line no device of the test platform uses, raised by setting it pending. */
#define TEST_IRQ (CONFIG_NUM_IRQS - 1)

static struct k_deadline_reservation res;
static volatile uint32_t handler_us;
static volatile int handled;

static struct k_deadline_miss last_miss;
static volatile int misses;

void k_deadline_miss_handler(const struct k_deadline_miss *miss)
{
	last_miss = *miss;
	misses++;
}

static void test_isr(const void *arg)
{
	ARG_UNUSED(arg);
	handled++;
	if (handler_us != 0U) {
		k_busy_wait(handler_us);
	}
}

static void raise(void)
{
	NVIC_SetPendingIRQ(TEST_IRQ);
	__DSB();
	__ISB();
}

static void *setup(void)
{
	irq_connect_dynamic(TEST_IRQ, 1, test_isr, NULL, 0);
	irq_enable(TEST_IRQ);
	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	k_irq_unreserve(TEST_IRQ);
	res = (struct k_deadline_reservation){
		.isr_budget = k_us_to_cyc_ceil32(200U),
		.min_interarrival = k_us_to_cyc_ceil32(5000U),
	};
	zassert_ok(k_irq_reserve(TEST_IRQ, &res));
	handler_us = 0U;
	handled = 0;
	misses = 0;
	/* Let a previous case's last arrival age past the inter-arrival. */
	k_msleep(10);
}

/* Arrivals at the declared spacing, handlers within budget: nothing reported. */
ZTEST(deadline_irq_budget, test_within_model)
{
	raise();
	k_msleep(10);
	raise();
	k_msleep(1);
	zassert_equal(handled, 2);
	zassert_equal(misses, 0, "an arrival within the model was reported");
}

/* An arrival sooner than the declared inter-arrival is reported with the reservation. */
ZTEST(deadline_irq_budget, test_early_arrival_reported)
{
	raise();
	k_busy_wait(500U);
	raise();
	k_msleep(1);
	zassert_equal(handled, 2);
	zassert_equal(misses, 1);
	zassert_equal(last_miss.kind, K_DEADLINE_MISS_OVERRUN);
	zassert_equal_ptr(last_miss.object, &res);
}

/* A handler longer than its budget is reported. */
ZTEST(deadline_irq_budget, test_long_handler_reported)
{
	handler_us = 1000U;
	raise();
	k_msleep(5);
	zassert_equal(handled, 1);
	zassert_equal(misses, 1);
	zassert_equal(last_miss.kind, K_DEADLINE_MISS_OVERRUN);
	zassert_equal_ptr(last_miss.object, &res);
}

/* An unreserved line is not checked (CONFIG_SCHED_DEADLINE_IRQ_STRICT off). */
ZTEST(deadline_irq_budget, test_unreserved_line_not_checked)
{
	k_irq_unreserve(TEST_IRQ);
	handler_us = 1000U;
	raise();
	k_busy_wait(100U);
	raise();
	k_msleep(5);
	zassert_equal(misses, 0);
}

ZTEST_SUITE(deadline_irq_budget, NULL, setup, before, NULL, NULL);
