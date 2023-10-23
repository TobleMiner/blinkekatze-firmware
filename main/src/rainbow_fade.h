#pragma once

#include <stdbool.h>

#include "color.h"
#include "wireless.h"

void rainbow_fade_update(void);
void rainbow_fade_apply(color_hsv_t *color);
void rainbow_fade_rx(const wireless_packet_t *packet);
void rainbow_fade_set_cycle_time(unsigned long cycle_time_ms);
void rainbow_fade_set_enable(bool enable);
