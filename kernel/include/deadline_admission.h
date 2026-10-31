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

/* An infeasible check, reported once the scheduler lock is released. */
struct admission_verdict {
	struct k_deadline_event *victim;
	uint32_t now;
	uint32_t release;
	int deadline;
	uint32_t demand;
};

#ifdef CONFIG_SCHED_DEADLINE_ADMISSION
/* The scheduler lock is held by the caller of each of these but the last. */
bool z_deadline_admission_posted(struct k_deadline_event *event, bool idle, uint32_t now,
				 struct admission_verdict *verdict);
void z_deadline_admission_started(struct k_deadline_event *event, uint32_t now);
void z_deadline_admission_finished(struct k_deadline_event *event);
void z_deadline_admission_reset(struct k_deadline_event *event);
void z_deadline_admission_report(const struct admission_verdict *verdict);
#else
static inline bool z_deadline_admission_posted(struct k_deadline_event *event, bool idle,
					       uint32_t now, struct admission_verdict *verdict)
{
	ARG_UNUSED(event);
	ARG_UNUSED(idle);
	ARG_UNUSED(now);
	ARG_UNUSED(verdict);
	return true;
}
static inline void z_deadline_admission_started(struct k_deadline_event *event, uint32_t now)
{
	ARG_UNUSED(event);
	ARG_UNUSED(now);
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

#endif /* ZEPHYR_KERNEL_INCLUDE_DEADLINE_ADMISSION_H_ */
