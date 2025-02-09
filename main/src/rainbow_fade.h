#pragma once

#include <stdbool.h>

#include "color.h"
#include "neighbour_rssi_delay_model.h"
#include "wireless.h"

void rainbow_fade_init(void);
void rainbow_fade_apply(color_t *color);
void rainbow_fade_rx(const wireless_packet_t *packet);
void rainbow_fade_set_cycle_time(unsigned long cycle_time_ms);
void rainbow_fade_set_enable(bool enable);
void rainbow_fade_set_phase_shift_enable(bool enable);
void rainbow_fade_set_rssi_delay_model(const neighbour_rssi_delay_model_t *model);
