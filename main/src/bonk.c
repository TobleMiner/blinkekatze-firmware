#include "bonk.h"

#include <string.h>

#include <esp_log.h>
#include <esp_timer.h>

#include "main.h"
#include "platform_laempan.h"
#include "util.h"

#define BONK_MAX_INTENSITY_THRESHOLD	20000
#define BONK_DURATION_MS		1000

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
			shared_config_hdr_t shared_cfg_hdr;
			int32_t delay_model_us_delay_per_rssi_step;
			int32_t delay_model_delay_limit_us;
			uint16_t bonk_duration_ms;
			int8_t delay_model_rssi_limit;
		} __attribute__((packed)) config;
	};
} __attribute__((packed)) bonk_packet_t;

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

static void process_bonk(bonk_t *bonk, int64_t timestamp, uint32_t magnitude) {
	bonk_event_t *buffered_bonk = &bonk->bonks[bonk->bonk_write_pos];
	buffered_bonk->timestamp_us = timestamp;
	buffered_bonk->magnitude = MIN(magnitude, BONK_MAX_INTENSITY_THRESHOLD);
	bonk->bonk_write_pos++;
	bonk->bonk_write_pos %= ARRAY_SIZE(bonk->bonks);
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
			.bonk_duration_ms = bonk->bonk_duration_ms,
			.delay_model_rssi_limit = bonk->delay_model.delay_rssi_limit,
			.delay_model_us_delay_per_rssi_step = bonk->delay_model.us_delay_per_rssi_step,
			.delay_model_delay_limit_us = bonk->delay_model.delay_limit_us
		}
	};
	shared_config_hdr_init(&bonk->shared_cfg, &packet.config.shared_cfg_hdr);
	wireless_broadcast((const uint8_t *)&packet, sizeof(packet));
	shared_config_tx_done(&bonk->shared_cfg);
}

static void config_changed(bonk_t *bonk) {
	shared_config_update_local(&bonk->shared_cfg);

	for (int i = 0; i < SHARED_CONFIG_TX_TIMES; i++) {
		bonk_tx_config(bonk);
	}
}

static void config_update(bonk_t *bonk) {
	if (shared_config_should_tx(&bonk->shared_cfg)) {
		bonk_tx_config(bonk);
	}
}

static void update_magnitude(bonk_t *bonk) {
	int64_t now = esp_timer_get_time();
	uint32_t magnitude = 0;
	for (int i = 0; i < ARRAY_SIZE(bonk->bonks); i++) {
		const bonk_event_t *buffered_bonk = &bonk->bonks[i];
		int64_t bonk_age = now - buffered_bonk->timestamp_us;
		if (now >= buffered_bonk->timestamp_us &&
		    bonk_age < MS_TO_US((int64_t)bonk->bonk_duration_ms)) {
			uint32_t bonk_decay = 0;
			if (bonk->enable_decay) {
				bonk_decay = (int64_t)BONK_MAX_INTENSITY_THRESHOLD * bonk_age / MS_TO_US((int64_t)bonk->bonk_duration_ms);
			}
			if (bonk_decay < buffered_bonk->magnitude) {
				uint32_t local_magnitude = buffered_bonk->magnitude - bonk_decay;
				if (magnitude + local_magnitude <= BONK_MAX_INTENSITY_THRESHOLD) {
					magnitude += local_magnitude;
				} else {
					magnitude = BONK_MAX_INTENSITY_THRESHOLD;
					break;
				}
			}
		}
	}

	bonk->magnitude = magnitude;
}

void bonk_trigger(bonk_t *bonk, uint32_t magnitude) {
	int64_t now = esp_timer_get_time();
	bonk_tx_bonk(now, magnitude);
	process_bonk(bonk, now, magnitude);
	ESP_LOGD(TAG, "BONK! intensity: %lu", (long unsigned int)magnitude);
}

static esp_err_t bonk_update_(bonk_t *bonk) {
	if (bonk->accel) {
		esp_err_t err = lis3dh_update(bonk->accel);
		if (err) {
			ESP_LOGE(TAG, "Failed to update accelerometer: %d", err);
			return err;
		}
		if (lis3dh_has_click_been_detected(bonk->accel)) {
			uint32_t velocity_magnitude = ABS(lis3dh_get_click_velocity(bonk->accel));
			bonk_trigger(bonk, velocity_magnitude);
		}
	}
	update_magnitude(bonk);
	config_update(bonk);
	return ESP_OK;
}

static void bonk_update(void *priv);
static void bonk_update(void *priv) {
	bonk_t *bonk = priv;
	bonk_update_(bonk);
	scheduler_schedule_task_relative(&bonk->update_task, bonk_update, bonk, MS_TO_US(20));
}

void bonk_init(bonk_t *bonk, lis3dh_t *accel, platform_t *platform) {
	memset(bonk, 0, sizeof(*bonk));
	bonk->accel = accel;
	bonk->delay_model.delay_rssi_threshold = -20;
	bonk->delay_model.delay_rssi_limit = -90;
	bonk->delay_model.us_delay_per_rssi_step = 10000;
	bonk->delay_model.delay_limit_us = 0;
	bonk->bonk_duration_ms = BONK_DURATION_MS;
	bonk->enable = true;
	bonk->bonk_max_val = HSV_VAL_MAX;
	bonk->bonk_intensity_divisor = 1;
	if (platform_is_laempan(platform)) {
		bonk->bonk_max_val = HSV_VAL_MAX / 4;
		bonk->bonk_intensity_divisor = 4;
	}

	scheduler_task_init(&bonk->update_task);
	scheduler_schedule_task_relative(&bonk->update_task, bonk_update, bonk, MS_TO_US(100));
}

static void rx_bonk(bonk_t *bonk, const wireless_packet_t *packet, const bonk_packet_t *bonk_packet, const neighbour_t *neigh) {
	int64_t local_timestamp = packet->rx_timestamp;
	if (neigh) {
		local_timestamp = neighbour_remote_to_local_time(neigh, bonk_packet->bonk.timestamp_us);
		if (bonk->enable_delay) {
			local_timestamp += neighbour_calculate_rssi_delay(&bonk->delay_model, neigh);
		}
	}
	process_bonk(bonk, local_timestamp, bonk_packet->bonk.magnitude);
}

static void rx_config(bonk_t *bonk, const wireless_packet_t *packet, const bonk_packet_t *bonk_packet) {
	if (shared_config_update_remote(&bonk->shared_cfg, &bonk_packet->config.shared_cfg_hdr)) {
		bonk->bonk_duration_ms = bonk_packet->config.bonk_duration_ms;
		bonk->enable = !!(bonk_packet->config.flags & FLAG_BONK_ENABLE);
		bonk->enable_decay = !!(bonk_packet->config.flags & FLAG_DECAY_ENABLE);
		bonk->enable_delay = !!(bonk_packet->config.flags & FLAG_DELAY_ENABLE);
		bonk->delay_model.delay_rssi_threshold = bonk_packet->config.delay_model_rssi_treshold;
		bonk->delay_model.delay_rssi_limit = bonk_packet->config.delay_model_rssi_limit;
		bonk->delay_model.us_delay_per_rssi_step = bonk_packet->config.delay_model_us_delay_per_rssi_step;
		bonk->delay_model.delay_limit_us = bonk_packet->config.delay_model_delay_limit_us;
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
		rx_bonk(bonk, packet, &bonk_packet, neigh);
		post_event(EVENT_LED);
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
		uint32_t brightness = intensity * (uint32_t)bonk->bonk_max_val / BONK_MAX_INTENSITY;
		brightness /= bonk->bonk_intensity_divisor;
		color->v = MIN((uint32_t)color->v + brightness, HSV_VAL_MAX);
	}
}

void bonk_set_enable(bonk_t *bonk, bool enable) {
	if (bonk->enable != enable) {
		bonk->enable = enable;
		config_changed(bonk);
	}
}

void bonk_set_decay_enable(bonk_t *bonk, bool enable) {
	if (bonk->enable_decay != enable) {
		bonk->enable_decay = enable;
		config_changed(bonk);
	}
}

void bonk_set_delay_enable(bonk_t *bonk, bool enable) {
	if (bonk->enable_delay != enable) {
		bonk->enable_delay = enable;
		config_changed(bonk);
	}
}

void bonk_set_duration(bonk_t *bonk, unsigned int duration) {
	if (bonk->bonk_duration_ms != duration) {
		bonk->bonk_duration_ms = duration;
		config_changed(bonk);
	}
}

void bonk_set_rssi_delay_model(bonk_t *bonk, const neighbour_rssi_delay_model_t *model) {
	if (memcmp(model, &bonk->delay_model, sizeof(*model))) {
		bonk->delay_model = *model;
		config_changed(bonk);
	}
}
