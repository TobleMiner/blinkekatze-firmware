#pragma once

#include <stdint.h>

#include "neighbour.h"

typedef struct neighbour_rssi_delay_model {
	int64_t us_delay_per_rssi_step;
	int64_t delay_limit_us;
	int delay_rssi_threshold;
	int delay_rssi_limit;
} neighbour_rssi_delay_model_t;

int64_t neighbour_calculate_rssi_delay(const neighbour_rssi_delay_model_t *model, const neighbour_t *neigh);
int64_t neighbour_calculate_rssi_delay_rssi(const neighbour_rssi_delay_model_t *model, int8_t rssi);
