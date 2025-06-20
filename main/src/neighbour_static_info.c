#include "neighbour_static_info.h"

#include <stdio.h>

#include <esp_app_desc.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_timer.h>

#include "scheduler.h"
#include "util.h"

#define STATIC_INFO_TX_INTERVAL_MS	60000

static const char *TAG = "node_static_info";

typedef struct neighbour_static_info {
	int64_t last_tx_timestamp;
	scheduler_task_t update_task;
	const platform_t *platform;
} neighbour_static_info_t;

static neighbour_static_info_t neighbour_static_info = { 0 };

void neighbour_static_info_rx(const wireless_packet_t *packet, const neighbour_t *neigh) {
	if (packet->len < sizeof(neighbour_static_info_packet_t)) {
		ESP_LOGI(TAG, "Short packet received, expected %u bytes but got %u bytes", sizeof(neighbour_static_info_packet_t), packet->len);
		return;
	}
	neighbour_static_info_packet_t static_info;
	memcpy(&static_info, packet->data, sizeof(neighbour_static_info_packet_t));
	if (neigh) {
		neighbour_update_static_info(neigh, &static_info);
	}
}

static void neighbour_static_info_update(void *arg);
static void neighbour_static_info_update(void *arg) {
	int64_t now = neighbour_get_global_clock();
	int64_t delta_ms = (now - neighbour_static_info.last_tx_timestamp) / 1000LL;
	if (delta_ms >= STATIC_INFO_TX_INTERVAL_MS || !neighbour_static_info.last_tx_timestamp) {
		neighbour_static_info_packet_t info = { .packet_type = WIRELESS_PACKET_TYPE_NEIGHBOUR_STATIC_INFO };
		memcpy(info.ap_password, wireless_get_ap_password(), WIRELESS_AP_PASSWORD_LENGTH);
		const esp_app_desc_t *app_desc = esp_app_get_description();
		memcpy(info.firmware_version, app_desc->version, sizeof(info.firmware_version));
		memcpy(info.firmware_sha256_hash, app_desc->app_elf_sha256, sizeof(info.firmware_sha256_hash));
		const char *platform_name = platform_get_name(neighbour_static_info.platform);
		memcpy(info.platform_name, platform_name, MIN(sizeof(info.platform_name), strlen(platform_name)));
		wireless_broadcast((const uint8_t *)&info, sizeof(info));
		neighbour_static_info.last_tx_timestamp = now;
	}
	scheduler_schedule_task_relative(&neighbour_static_info.update_task, neighbour_static_info_update, NULL, MS_TO_US(1000));
}

void neighbour_static_info_init(const platform_t *platform) {
	neighbour_static_info.platform = platform;
	scheduler_task_init(&neighbour_static_info.update_task);
	scheduler_schedule_task_relative(&neighbour_static_info.update_task, neighbour_static_info_update, NULL, 0);
}

void neighbour_static_info_get_ap_ssid(const neighbour_t *neigh, char *buf, size_t len) {
	snprintf(buf, len, "blinkekatze_"MACSTR, MAC2STR(neigh->address));
}

bool neighbour_static_info_get_ap_password(const neighbour_t *neigh, char *buf, size_t len) {
	if (!neigh->last_static_info.packet_type) {
		return false;
	}

	size_t psk_len = strnlen(neigh->last_static_info.ap_password, sizeof(neigh->last_static_info.ap_password));
	if (len - 1 < psk_len) {
		psk_len = len - 1;
	}
	memcpy(buf, neigh->last_static_info.ap_password, psk_len);
	buf[psk_len] = 0;
	return true;
}

const uint8_t *neighbour_static_info_get_firmware_sha256_hash(const neighbour_t *neigh) {
	if (!neigh->last_static_info.packet_type) {
		return false;
	}

	return neigh->last_static_info.firmware_sha256_hash;
}
