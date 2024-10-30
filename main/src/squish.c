#include "squish.h"

#include <string.h>

#include <esp_log.h>
#include <esp_timer.h>

#include "neighbour_rssi_delay_model.h"

#define NUM_PRESSURE_SAMPLES_DISCARD	 5
#define NUM_PRESSURE_SAMPLES_INIT	20

#define MAX_SQUISHEDNESS		25000
#define SQUISH_FACTOR			2
#define SQUISH_RECOVERY_INTERVAL_MS	3000
#define SQUISH_MAX_TX_INTERVAL_MS	33
#define PRESSURE_ADJUST_PER_SECOND	1
#define SQUISH_MIN_CHANGE_TX_MILLI	2

typedef struct squish_packet {
	uint8_t packet_type;
	int64_t timestamp_us;
	uint16_t squish;
} __attribute__((packed)) squish_packet_t;

static const char *TAG = "squish";

static const neighbour_rssi_delay_model_t squish_delay_model = {
	10000,
	0,
	-20,
	-90
};

static uint16_t squish_calculate_remote(squish_t *squish) {
	uint16_t squishedness = 0;
	int64_t now = esp_timer_get_time();
	for (int i = 0; i < ARRAY_SIZE(squish->remote_squishes); i++) {
		squish_remote_t *remote_squish = &squish->remote_squishes[i];
		if (remote_squish->timestamp_us < now) {
			uint64_t delta_us = now - remote_squish->timestamp_us;
			uint32_t squish_decay = (uint32_t)MAX_SQUISHEDNESS * delta_us / (uint32_t)SQUISH_RECOVERY_INTERVAL_MS / 1000UL;
			if (squish_decay < remote_squish->squish) {
				squishedness = MAX(squishedness, remote_squish->squish - squish_decay);
			}
		}
	}

	return squishedness;
}

static void squish_tx(squish_t *squish) {
	squish_packet_t squish_packet = {
		.packet_type = WIRELESS_PACKET_TYPE_SQUISH,
		.timestamp_us = esp_timer_get_time(),
		.squish = squish->local_squishedness
	};
	wireless_broadcast((const uint8_t *)&squish_packet, sizeof(squish_packet));
}

static void squish_tx_peak(squish_t *squish) {
	int64_t now = esp_timer_get_time();
	int64_t delta_us = now - squish->timestamp_last_tx_us;

	if (delta_us / 1000LL >= SQUISH_MAX_TX_INTERVAL_MS) {
		squish_tx(squish);
	} else {
		squish->peak_not_sent = true;
	}
}

static void squish_tx_delayed(squish_t *squish) {
	if (squish->peak_not_sent) {
		int64_t now = esp_timer_get_time();
		int64_t delta_us = now - squish->timestamp_last_tx_us;

		if (delta_us / 1000LL >= SQUISH_MAX_TX_INTERVAL_MS) {
			squish_tx(squish);
			squish->peak_not_sent = false;
		}
	}
}

static esp_err_t squish_update_(squish_t *squish) {
	if (squish->baro) {
		esp_err_t err = spl06_update(squish->baro);
		if (err) {
			ESP_LOGE(TAG, "Failed to update barometer: %d", err);
			return err;
		}
	}

	int64_t now = esp_timer_get_time();
	int64_t delta_us = now - squish->timestamp_last_update_us;
	if (squish->local_squishedness) {
		uint32_t squish_decay = (uint32_t)MAX_SQUISHEDNESS * delta_us / (uint32_t)SQUISH_RECOVERY_INTERVAL_MS / 1000UL;

		if (squish_decay > squish->local_squishedness) {
			squish->local_squishedness = 0;
		} else {
			squish->local_squishedness -= squish_decay;
		}
	}

	if (squish->baro) {
		int32_t pressure = spl06_get_pressure(squish->baro);
		if (squish->num_pressure_samples < NUM_PRESSURE_SAMPLES_INIT) {
			if (squish->num_pressure_samples == NUM_PRESSURE_SAMPLES_DISCARD) {
				squish->pressure_at_rest_milli = (int64_t)pressure * 1000LL;
			}
			if (squish->num_pressure_samples > NUM_PRESSURE_SAMPLES_DISCARD) {
				squish->pressure_at_rest_milli =
					(squish->pressure_at_rest_milli * 8LL + (int64_t)pressure * 2LL * 1000) / 10;
			}
			squish->num_pressure_samples++;
		} else {
			int32_t delta = squish->pressure_at_rest_milli / 1000LL - pressure;

			if (delta > 0) {
				uint32_t additional_squish = delta * SQUISH_FACTOR * (delta_us / 1000) / 8192;
				squish->local_squishedness = MIN(squish->local_squishedness + additional_squish, MAX_SQUISHEDNESS);
				if (additional_squish >= DIV_ROUND(MAX_SQUISHEDNESS * SQUISH_MIN_CHANGE_TX_MILLI, 1000)) {
					squish_tx_peak(squish);
				}
			}
			int64_t delta_ms = delta_us / 1000LL;
			int64_t pressure_fraction = delta_ms;
			if (pressure_fraction > 1000) {
				pressure_fraction = 1000;
			}
			int64_t pressure_max_adjust = DIV_ROUND(squish->pressure_at_rest_milli * (100LL - PRESSURE_ADJUST_PER_SECOND) + (int64_t)pressure * 1000LL * (int64_t)PRESSURE_ADJUST_PER_SECOND, 100);
			int64_t pressure_adjust = DIV_ROUND((pressure_max_adjust - squish->pressure_at_rest_milli)  * pressure_fraction, 1000);
			squish->pressure_at_rest_milli += pressure_adjust;
		}
	}
	squish_tx_delayed(squish);
	squish->squishedness = MAX(squish->local_squishedness, squish_calculate_remote(squish));
	squish->timestamp_last_update_us = now;
	return ESP_OK;
}

static void squish_update(void *arg);
static void squish_update(void *arg) {
	squish_t *squish = arg;
	squish_update_(squish);
	scheduler_schedule_task_relative(&squish->update_task, squish_update, squish, MS_TO_US(20));
}

void squish_init(squish_t *squish, spl06_t *baro) {
	memset(squish, 0, sizeof(*squish));
	squish->baro = baro;
	scheduler_task_init(&squish->update_task);
	scheduler_schedule_task_relative(&squish->update_task, squish_update, squish, MS_TO_US(100));
}

void squish_apply(const squish_t *squish, color_hsv_t *color) {
	/*
	 * Squishing is a three step process:
	 *  1. Desaturate current color to white (0 - 1/5 squish)
	 *  2. Change color from white to red and saturate (1/5 - 3/5 squish)
	 *  3. Change color from red to blue (3/5 - full squish)
	 */
//	const int32_t mid_hue = 40 << 5;
	const int32_t mid_hue = HSV_HUE_MAX;
	const int32_t mid_saturation = 220 << 8;
//	const int32_t mid_saturation = 0;
	const int32_t final_hue = 845 << 5;
	const int32_t final_saturation = 200 << 8;
//	const int32_t final_saturation = HSV_SAT_MAX;
	int32_t bend = squish->squishedness;
	const int32_t scale1 = MAX_SQUISHEDNESS / 5;
	const int32_t scale2 = MAX_SQUISHEDNESS * 2 / 5;
	const int32_t scale3 = MAX_SQUISHEDNESS * 2 / 5;
	if (bend <= scale1) {
		/* Desaturate to white */
		color->s -= (int32_t)color->s * bend  / scale1;
	} else if (bend <= scale1 + scale2) {
		/* Stay white for a while */
		color->s = 0;


		/* Force to red and saturate */
/*
		int32_t local_bend = bend - scale1;
		color->h = mid_hue;
		color->s = mid_saturation * local_bend / scale2;
*/
	} else {
		/* Force to red and saturate */
		int32_t local_bend = bend - scale1 - scale2;
		color->h = mid_hue;
		color->s = mid_saturation * local_bend / scale3;

		/* Morph to blue */
/*
		int32_t local_bend = bend - scale1 - scale2;
		color->h = mid_hue + (final_hue - mid_hue) * local_bend / scale3;
		color->s = mid_saturation + (final_saturation - mid_saturation) * local_bend / scale2;
		ESP_LOGI(TAG, "Saturation: %u, local_bend: %ld, scale2: %ld", color->s, local_bend, scale2);
*/
	}
}

void squish_rx(squish_t *squish, const wireless_packet_t *packet, const neighbour_t *neigh) {
	squish_packet_t squish_packet;
	if (packet->len < sizeof(squish_packet)) {
		ESP_LOGD(TAG, "Received short packet. Expected %u bytes but got %u bytes", sizeof(squish_packet), packet->len);
		return;
	}
	memcpy(&squish_packet, packet->data, sizeof(squish_packet));
	int64_t timestamp_us = packet->rx_timestamp;
	if (neigh) {
		int8_t rssi = neighbour_get_rssi(neigh);
		int64_t delay_us = neighbour_calculate_rssi_delay_rssi(&squish_delay_model, rssi);
		timestamp_us = neighbour_remote_to_local_time(neigh, squish_packet.timestamp_us) + delay_us;
	}
	squish_remote_t *remote_squish = &squish->remote_squishes[squish->remote_squish_write_pos];
	remote_squish->timestamp_us = timestamp_us;
	remote_squish->squish = squish_packet.squish;
	squish->remote_squish_write_pos++;
	squish->remote_squish_write_pos %= ARRAY_SIZE(squish->remote_squishes);
}
