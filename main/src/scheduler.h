#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "list.h"

typedef void (*scheduler_cb_f)(void *ctx);

typedef struct scheduler_task {
	struct list_head list;
	int64_t deadline_us;
	scheduler_cb_f cb;
	void *ctx;
} scheduler_task_t;

void scheduler_init(void);
void scheduler_run(void);
void scheduler_task_init(scheduler_task_t *task);
void scheduler_schedule_task(scheduler_task_t *task, scheduler_cb_f cb, void *ctx, int64_t deadline_us);
void scheduler_schedule_task_relative(scheduler_task_t *task, scheduler_cb_f cb, void *ctx, int64_t timeout_us);
