#include "bonk.h"

#include <string.h>

#include <esp_log.h>
#include <esp_timer.h>

#include "util.h"

#define BONK_MIN_INTENSITY_THRESHOLD	2000
#define BONK_MAX_INTENSITY_THRESHOLD	20000
#define BONK_DURATION_US		MS_TO_US(100)

static const char *TAG = "bonk";

typedef struct bonk_packet {
	uint8_t packet_type;
	int64_t timestamp_us;
	uint32_t magnitude;
} __attribute__((packed)) bonk_packet_t;

void bonk_init(bonk_t *bonk, lis3dh_t *accel) {
	memset(bonk, 0, sizeof(*bonk));
	bonk->accel = accel;
}

static void bonk_tx(int64_t timestamp, uint32_t magnitude) {
	bonk_packet_t packet = {
		WIRELESS_PACKET_TYPE_BONK,
		timestamp,
		magnitude
	};

	wireless_broadcast((const uint8_t *)&packet, sizeof(packet));
}

static void process_bonk(bonk_t *bonk, int64_t now, int64_t timestamp, uint32_t magnitude) {
	ESP_LOGD(TAG, "bonk active for %dms at intensity: %lu", (int)(((int64_t)BONK_DURATION_US - (now - timestamp)) / 1000LL), (long unsigned int)magnitude);
	bool last_bonk_timeout = now > bonk->last_bonk_us + BONK_DURATION_US;
	bool stronger_than_last_bonk = magnitude >= bonk->last_bonk_magnitude;
	if (last_bonk_timeout || stronger_than_last_bonk) {
		bonk->last_bonk_us = timestamp;
		bonk->last_bonk_magnitude = magnitude;
	}
}

esp_err_t bonk_update(bonk_t *bonk) {
	esp_err_t err = lis3dh_update(bonk->accel);
	if (err) {
		ESP_LOGE(TAG, "Failed to update accelerometer: %d", err);
		return err;
	}
	if (lis3dh_has_click_been_detected(bonk->accel)) {
		int64_t now = esp_timer_get_time();
		uint32_t velocity_magnitude = ABS(lis3dh_get_click_velocity(bonk->accel));
		bonk_tx(now, velocity_magnitude);
		process_bonk(bonk, now, now, velocity_magnitude);
		ESP_LOGD(TAG, "BONK! intensity: %lu", (long unsigned int)velocity_magnitude);
	}
	return ESP_OK;
}

void bonk_rx(bonk_t *bonk, const wireless_packet_t *packet, const neighbour_t *neigh) {
	if (packet->len < sizeof(bonk_packet_t)) {
		ESP_LOGE(TAG, "Short packet received. Expected %zu bytes but got %u bytes", sizeof(bonk_packet_t), packet->len);
		return;
	}

	bonk_packet_t bonk_packet;
	memcpy(&bonk_packet, packet->data, sizeof(bonk_packet_t));
	int64_t now = esp_timer_get_time();
	int64_t local_timestamp = now;
	if (neigh) {
		local_timestamp = neighbour_remote_to_local_time(neigh, bonk_packet.timestamp_us);
	}
	process_bonk(bonk, now, local_timestamp, bonk_packet.magnitude);
}

unsigned int bonk_get_intensity(const bonk_t *bonk) {
	int64_t now = esp_timer_get_time();
	bool last_bonk_timeout = now > bonk->last_bonk_us + BONK_DURATION_US;
	if (last_bonk_timeout) {
		return 0;
	}

	uint32_t magnitude_clipped = MIN(MAX(bonk->last_bonk_magnitude, BONK_MIN_INTENSITY_THRESHOLD), BONK_MAX_INTENSITY_THRESHOLD);
	return magnitude_clipped * (uint32_t)BONK_MAX_INTENSITY / (uint32_t)BONK_MAX_INTENSITY_THRESHOLD;
}

void bonk_apply(bonk_t *bonk, color_hsv_t *color) {
	uint32_t intensity = bonk_get_intensity(bonk);
	uint16_t brightness = intensity * HSV_VAL_MAX / BONK_MAX_INTENSITY;
	color->v = MIN(color->v + brightness, HSV_VAL_MAX);
}
