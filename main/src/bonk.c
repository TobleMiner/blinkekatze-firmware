#include "bonk.h"

#include <string.h>

#include <esp_log.h>
#include <esp_timer.h>

#include "util.h"

#define BONK_MIN_INTENSITY_THRESHOLD	2000
#define BONK_MAX_INTENSITY_THRESHOLD	20000
#define BONK_DURATION_MS		1000
#define CONFIG_TX_INTERVAL_MS		10000
#define CONFIG_CHANGE_TX_TIMES		3

static const char *TAG = "bonk";

typedef enum bonk_packet_type {
	BONK_PACKET_TYPE_BONK	= 0,
	BONK_PACKET_TYPE_CONFIG	= 1
} bonk_packet_type_t;

#define FLAG_BONK_ENABLE	BIT(0)
#define FLAG_DECAY_ENABLE	BIT(1)
#define FLAG_DELAY_ENABLE	BIT(2)

typedef struct bonk_packet {
	uint8_t packet_type;
	uint8_t bonk_packet_type;
	union {
		struct {
			uint32_t magnitude;
			int64_t timestamp_us;
		} __attribute__((packed)) bonk;
		struct {
			uint8_t flags;
			int8_t delay_model_rssi_treshold;
			int64_t config_timestamp_us;
			int32_t delay_model_us_delay_per_rssi_step;
			int32_t delay_model_delay_limit_us;
			uint16_t bonk_duration_ms;
			int8_t delay_model_rssi_limit;
		} __attribute__((packed)) config;
	};
} __attribute__((packed)) bonk_packet_t;

void bonk_init(bonk_t *bonk, lis3dh_t *accel) {
	memset(bonk, 0, sizeof(*bonk));
	bonk->accel = accel;
	bonk->delay_model.delay_rssi_threshold = -20;
	bonk->delay_model.delay_rssi_limit = -90;
	bonk->delay_model.us_delay_per_rssi_step = 10000;
	bonk->delay_model.delay_limit_us = 0;
	bonk->bonk_duration_ms = BONK_DURATION_MS;
	bonk->enable = true;
	bonk->enable_decay = true;
	bonk->enable_delay = true;
}

static void bonk_tx_bonk(int64_t timestamp, uint32_t magnitude) {
	bonk_packet_t packet = {
		WIRELESS_PACKET_TYPE_BONK,
		BONK_PACKET_TYPE_BONK,
		.bonk = {
			.magnitude = magnitude,
			.timestamp_us = timestamp
		}
	};

	wireless_broadcast((const uint8_t *)&packet, sizeof(packet));
}

static void process_bonk(bonk_t *bonk, int64_t now, int64_t timestamp, uint32_t magnitude) {
	uint32_t bonk_decay = (int64_t)BONK_MAX_INTENSITY_THRESHOLD * (now - timestamp) / MS_TO_US((int64_t)bonk->bonk_duration_ms);
	if (bonk_decay < magnitude) {
		magnitude -= bonk_decay;
		if (bonk->magnitude + magnitude <= BONK_MAX_INTENSITY_THRESHOLD) {
			bonk->magnitude += magnitude;
		} else {
			bonk->magnitude = BONK_MAX_INTENSITY_THRESHOLD;
		}
	}
}

static void bonk_tx_config(bonk_t *bonk) {
	bonk_packet_t packet = {
		WIRELESS_PACKET_TYPE_BONK,
		BONK_PACKET_TYPE_CONFIG,
		.config = {
			.flags =
				(bonk->enable ? FLAG_BONK_ENABLE : 0) |
				(bonk->enable_decay ? FLAG_DECAY_ENABLE : 0) |
				(bonk->enable_delay ? FLAG_DELAY_ENABLE : 0),
			.delay_model_rssi_treshold = bonk->delay_model.delay_rssi_threshold,
			.config_timestamp_us = bonk->config_timestamp_us,
			.bonk_duration_ms = bonk->bonk_duration_ms,
			.delay_model_rssi_limit = bonk->delay_model.delay_rssi_limit,
			.delay_model_us_delay_per_rssi_step = bonk->delay_model.us_delay_per_rssi_step,
			.delay_model_delay_limit_us = bonk->delay_model.delay_limit_us
		}
	};

	bonk->last_tx_timestamp_us = esp_timer_get_time();
	wireless_broadcast((const uint8_t *)&packet, sizeof(packet));
}

static void config_changed(bonk_t *bonk) {
	int64_t now = neighbour_get_global_clock();
	bonk->config_timestamp_us = now;
	bonk->last_rx_timestamp_us = 0;

	for (int i = 0; i < CONFIG_CHANGE_TX_TIMES; i++) {
		bonk_tx_config(bonk);
	}
}

static void config_update(bonk_t *bonk) {
	if (!bonk->config_timestamp_us) {
		// We are in default config, no need to announce that
		return;
	}

	int64_t now = esp_timer_get_time();
	int64_t delta_us_last_rx = now - bonk->last_rx_timestamp_us;
	if (delta_us_last_rx / 1000LL < CONFIG_TX_INTERVAL_MS * 2) {
		// Up to date config received from neighbour already, no need to distribute
		return;
	}

	int64_t delta_us_last_tx = now - bonk->last_tx_timestamp_us;
	if (delta_us_last_tx / 1000LL < CONFIG_TX_INTERVAL_MS) {
		// Up to date config sent recently, do not flood the network
		return;
	}

	bonk_tx_config(bonk);
}

esp_err_t bonk_update(bonk_t *bonk) {
	esp_err_t err = lis3dh_update(bonk->accel);
	if (err) {
		ESP_LOGE(TAG, "Failed to update accelerometer: %d", err);
		return err;
	}
	int64_t now = esp_timer_get_time();
	uint32_t bonk_decay = (int64_t)BONK_MAX_INTENSITY_THRESHOLD * (now - bonk->timestamp_last_update) / MS_TO_US((int64_t)bonk->bonk_duration_ms);
	if (bonk_decay <= bonk->magnitude) {
		bonk->magnitude -= bonk_decay;
	} else {
		bonk->magnitude = 0;
	}
	if (lis3dh_has_click_been_detected(bonk->accel)) {
		uint32_t velocity_magnitude = ABS(lis3dh_get_click_velocity(bonk->accel));
		bonk_tx_bonk(now, velocity_magnitude);
		process_bonk(bonk, now, now, velocity_magnitude);
		ESP_LOGD(TAG, "BONK! intensity: %lu", (long unsigned int)velocity_magnitude);
	}
	bonk->timestamp_last_update = now;
	config_update(bonk);
	return ESP_OK;
}

static void rx_bonk(bonk_t *bonk, const bonk_packet_t *bonk_packet, const neighbour_t *neigh) {
	int64_t now = esp_timer_get_time();
	int64_t local_timestamp = now;
	if (neigh) {
		local_timestamp = neighbour_remote_to_local_time(neigh, bonk_packet->bonk.timestamp_us);
	}
	process_bonk(bonk, now, local_timestamp, bonk_packet->bonk.magnitude);
}

static void rx_config(bonk_t *bonk, const wireless_packet_t *packet, const bonk_packet_t *bonk_packet) {
	if (bonk_packet->config.config_timestamp_us > bonk->config_timestamp_us) {
		bonk->bonk_duration_ms = bonk_packet->config.bonk_duration_ms;
		bonk->enable = !!(bonk_packet->config.flags & FLAG_BONK_ENABLE);
		bonk->enable_decay = !!(bonk_packet->config.flags & FLAG_DECAY_ENABLE);
		bonk->enable_delay = !!(bonk_packet->config.flags & FLAG_DELAY_ENABLE);
		bonk->delay_model.delay_rssi_threshold = bonk_packet->config.delay_model_rssi_treshold;
		bonk->delay_model.delay_rssi_limit = bonk_packet->config.delay_model_rssi_limit;
		bonk->delay_model.us_delay_per_rssi_step = bonk_packet->config.delay_model_us_delay_per_rssi_step;
		bonk->delay_model.delay_limit_us = bonk_packet->config.delay_model_delay_limit_us;
	}

	if (bonk_packet->config.config_timestamp_us >= bonk->config_timestamp_us) {
		bonk->last_rx_timestamp_us = packet->rx_timestamp;
	}
}

void bonk_rx(bonk_t *bonk, const wireless_packet_t *packet, const neighbour_t *neigh) {
	if (packet->len < sizeof(bonk_packet_t)) {
		ESP_LOGE(TAG, "Short packet received. Expected %zu bytes but got %u bytes", sizeof(bonk_packet_t), packet->len);
		return;
	}

	bonk_packet_t bonk_packet;
	memcpy(&bonk_packet, packet->data, sizeof(bonk_packet_t));
	switch (bonk_packet.bonk_packet_type) {
	case BONK_PACKET_TYPE_BONK:
		rx_bonk(bonk, &bonk_packet, neigh);
		break;
	case BONK_PACKET_TYPE_CONFIG:
		rx_config(bonk, packet, &bonk_packet);
	}
}

unsigned int bonk_get_intensity(const bonk_t *bonk) {
	return bonk->magnitude * BONK_MAX_INTENSITY / BONK_MAX_INTENSITY_THRESHOLD;
}

void bonk_apply(bonk_t *bonk, color_hsv_t *color) {
	if (bonk->enable) {
		uint32_t intensity = bonk_get_intensity(bonk);
		uint32_t brightness = intensity * HSV_VAL_MAX / BONK_MAX_INTENSITY;
		color->v = MIN((uint32_t)color->v + brightness, HSV_VAL_MAX);
	}
}

void bonk_set_enable(bonk_t *bonk, bool enable) {
	if (bonk->enable != enable) {
		bonk->enable = enable;
		config_changed(bonk);
	}
}
