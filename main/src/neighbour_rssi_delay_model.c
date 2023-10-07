#include "neighbour_rssi_delay_model.h"

int64_t neighbour_calculate_rssi_delay(const neighbour_rssi_delay_model_t *model, const neighbour_t *neigh) {
	if (!neigh) {
		return 0;
	}

	if (neigh->rssi > model->delay_rssi_threshold) {
		return 0;
	}

	int64_t effective_rssi = neigh->rssi;
	if (model->delay_rssi_limit && effective_rssi < model->delay_rssi_limit) {
		effective_rssi = model->delay_rssi_limit;
	}

	int64_t delay_us = (model->delay_rssi_threshold - effective_rssi) * model->us_delay_per_rssi_step;
	if (model->delay_limit_us && delay_us > model->delay_limit_us) {
		delay_us = model->delay_limit_us;
	}

	return delay_us;
}
