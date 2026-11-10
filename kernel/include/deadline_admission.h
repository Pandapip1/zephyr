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
/* After a release has been recorded and its consumer readied: take the dispatch decision. */
void z_deadline_admission_decide(uint32_t now);
void z_deadline_admission_report(const struct admission_verdict *verdict);
#else
static inline void z_deadline_admission_decide(uint32_t now)
{
	ARG_UNUSED(now);
}
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

#ifdef CONFIG_SCHED_DEADLINE_DISPATCH
/* next_up(): the thread the dispatch rule runs instead of @p best, or @p best. Scheduler lock held. */
struct k_thread *z_deadline_dispatch(struct k_thread *best);
#endif

#ifdef CONFIG_SCHED_DEADLINE_BUDGET_ENFORCE
/* At every context switch (CONFIG_INSTRUMENT_THREAD_SWITCHING): the run in progress's time. */
void z_deadline_budget_switched_out(void);
void z_deadline_budget_switched_in(void);
#endif

#ifdef CONFIG_SCHED_DEADLINE_JOBS
/*
 * Jobs (kernel/deadline_admission.c): the running job, a run ending in a blocking call and
 * the wake that releases the next, and the job a kernel timeout runs as. The scheduler lock
 * is held by the callers of block, timeout and ready; resumed is called by the woken thread
 * with no lock held.
 */
struct k_deadline_event *z_deadline_current_job(void);
void z_deadline_job_block(struct k_thread *thread);
void z_deadline_job_timeout(struct k_thread *thread);
bool z_deadline_job_ready(struct k_thread *thread);
void z_deadline_job_resumed(void);
/* A kernel timeout armed with no job behind it. Timeout lock held. */
void z_deadline_job_unowned(const struct _timeout *to);
/* Around a timeout's handler: the job it runs as (NULL after it). */
void z_deadline_job_expiry(struct k_deadline_event *owner);
/* Around an interrupt handler (arch interrupt entry): the job is saved, none, restored. */
struct k_deadline_event *z_deadline_job_irq_enter(void);
void z_deadline_job_irq_exit(struct k_deadline_event *saved);
#endif

#ifdef CONFIG_SCHED_DEADLINE_IRQ_BUDGETS
/* Around an interrupt handler (arch interrupt entry): arrival and handler checks. */
void z_deadline_irq_enter(int32_t irq);
void z_deadline_irq_exit(void);
#endif

#endif /* ZEPHYR_KERNEL_INCLUDE_DEADLINE_ADMISSION_H_ */
