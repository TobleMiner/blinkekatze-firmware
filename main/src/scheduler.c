#include "scheduler.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "main.h"

typedef struct scheduler {
	struct list_head tasks;
	esp_timer_handle_t timer;
	bool timer_running;
	int64_t timer_deadline_us;
	struct list_head tasks_schedule;
	struct list_head tasks_abort;
} scheduler_t;

static const char *TAG = "scheduler";

static scheduler_t scheduler_g;

void scheduler_timer_cb(void *arg) {
	scheduler_t *scheduler = arg;

	scheduler->timer_running = false;
	post_event(EVENT_SCHEDULER);
}

static void start_timer_for_task(scheduler_task_t *task) {
	scheduler_t *scheduler = &scheduler_g;
	int64_t now = esp_timer_get_time();

	scheduler->timer_deadline_us = task->deadline_us;
	scheduler->timer_running = true;
	esp_timer_start_once(scheduler->timer, task->deadline_us > now ? task->deadline_us - now : 0);
}

static void recalc_timer(void) {
	scheduler_t *scheduler = &scheduler_g;

	if (!LIST_IS_EMPTY(&scheduler->tasks)) {
		scheduler_task_t *task = LIST_GET_ENTRY(scheduler->tasks.next, scheduler_task_t, list);

		if (scheduler->timer_deadline_us > task->deadline_us && scheduler->timer_running) {
			esp_timer_stop(scheduler->timer);
			scheduler->timer_running = false;
		}
		if (!scheduler->timer_running) {
			start_timer_for_task(task);
		}
	}
}

void scheduler_run() {
	scheduler_t *scheduler = &scheduler_g;

	int64_t now;
	scheduler_task_t *cursor;
	struct list_head *next;

	now = esp_timer_get_time();
	LIST_FOR_EACH_ENTRY_SAFE(cursor, next, &scheduler->tasks, list) {
		if (now >= cursor->deadline_us) {
			LIST_DELETE(&cursor->list);
			cursor->cb(cursor->ctx);
		} else {
			break;
		}
	}
	recalc_timer();
}

void scheduler_init() {
	scheduler_t *scheduler = &scheduler_g;
	esp_timer_create_args_t timer_args = {
		.callback = scheduler_timer_cb,
		.arg = scheduler,
		.dispatch_method = ESP_TIMER_TASK,
		.skip_unhandled_events = true
	};

	INIT_LIST_HEAD(scheduler->tasks);
	ESP_ERROR_CHECK(esp_timer_create(&timer_args, &scheduler->timer));
	scheduler->timer_running = false;
}

void scheduler_task_init(scheduler_task_t *task) {
	INIT_LIST_HEAD(task->list);
}

static void enqueue_task(scheduler_task_t *task) {
	scheduler_t *scheduler = &scheduler_g;
	struct list_head *prior_deadline = &scheduler->tasks;
	scheduler_task_t *cursor;

	LIST_FOR_EACH_ENTRY(cursor, &scheduler->tasks, list) {
		if (cursor->deadline_us > task->deadline_us) {
			break;
		}
		prior_deadline = &cursor->list;
	}
	if (!LIST_IS_EMPTY(&task->list)) {
		LIST_DELETE(&task->list);
	}
	LIST_APPEND(&task->list, prior_deadline);
	recalc_timer();
}

void scheduler_schedule_task(scheduler_task_t *task, scheduler_cb_f cb, void *ctx, int64_t deadline_us) {
	task->deadline_us = deadline_us;
	task->cb = cb;
	task->ctx = ctx;
	enqueue_task(task);
}

void scheduler_schedule_task_relative(scheduler_task_t *task, scheduler_cb_f cb, void *ctx, int64_t timeout_us) {
	int64_t now = esp_timer_get_time();

	scheduler_schedule_task(task, cb, ctx, now + timeout_us);
}
