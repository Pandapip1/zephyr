/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Admission control hooks of deadline events (kernel/deadline_admission.c). */

#ifndef ZEPHYR_KERNEL_INCLUDE_DEADLINE_ADMISSION_H_
#define ZEPHYR_KERNEL_INCLUDE_DEADLINE_ADMISSION_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/deadline_miss.h>

/* An infeasible check or a refused release, reported once the scheduler lock is released. */
struct admission_verdict {
	enum k_deadline_miss_kind kind;
	struct k_deadline_event *victim;
	uint32_t now;
	uint32_t release;
	int deadline;
	uint32_t demand;
};

#ifdef CONFIG_SCHED_DEADLINE_ADMISSION
/* The scheduler lock is held by the caller of each of these but the last. */
void z_deadline_admission_publication(const struct k_deadline_event *event, uint32_t now,
				      int *deadline, uint32_t *budget);
bool z_deadline_admission_posted(struct k_deadline_event *event, bool idle, uint32_t now,
				 struct admission_verdict *verdict);
void z_deadline_admission_started(struct k_deadline_event *event, uint32_t now,
				  uint32_t budget);
void z_deadline_admission_finished(struct k_deadline_event *event);
void z_deadline_admission_reset(struct k_deadline_event *event);
void z_deadline_admission_report(const struct admission_verdict *verdict);
#else
static inline void z_deadline_admission_publication(const struct k_deadline_event *event,
						    uint32_t now, int *deadline, uint32_t *budget)
{
	ARG_UNUSED(event);
	ARG_UNUSED(now);
	ARG_UNUSED(deadline);
	ARG_UNUSED(budget);
}
static inline bool z_deadline_admission_posted(struct k_deadline_event *event, bool idle,
					       uint32_t now, struct admission_verdict *verdict)
{
	ARG_UNUSED(event);
	ARG_UNUSED(idle);
	ARG_UNUSED(now);
	ARG_UNUSED(verdict);
	return true;
}
static inline void z_deadline_admission_started(struct k_deadline_event *event, uint32_t now,
						uint32_t budget)
{
	ARG_UNUSED(event);
	ARG_UNUSED(now);
	ARG_UNUSED(budget);
}
static inline void z_deadline_admission_finished(struct k_deadline_event *event)
{
	ARG_UNUSED(event);
}
static inline void z_deadline_admission_reset(struct k_deadline_event *event)
{
	ARG_UNUSED(event);
}
static inline void z_deadline_admission_report(const struct admission_verdict *verdict)
{
	ARG_UNUSED(verdict);
}
#endif

#ifdef CONFIG_SCHED_DEADLINE_IRQ_BUDGETS
/* Around an interrupt handler (arch interrupt entry): arrival and handler checks. */
void z_deadline_irq_enter(int32_t irq);
void z_deadline_irq_exit(void);
#endif

#endif /* ZEPHYR_KERNEL_INCLUDE_DEADLINE_ADMISSION_H_ */
