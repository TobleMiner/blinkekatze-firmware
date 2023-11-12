#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include "util.h"

#define EVENT_WIRELESS	BIT(0)
#define EVENT_SCHEDULER	BIT(1)
#define EVENT_LED	BIT(2)
#define EVENTS		(EVENT_WIRELESS | EVENT_SCHEDULER | EVENT_LED)

void post_event(EventBits_t bits);
