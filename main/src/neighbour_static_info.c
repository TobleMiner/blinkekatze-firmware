#include "neighbour_static_info.h"

#include <stdio.h>

#include <esp_app_desc.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_timer.h>

#define STATIC_INFO_TX_INTERVAL_MS	60000

static const char *TAG = "node_static_info";

typedef struct neighbour_static_info {
	int64_t last_tx_timestamp;
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

void neighbour_static_info_update() {
	int64_t now = esp_timer_get_time();
	int64_t delta_ms = (now - neighbour_static_info.last_tx_timestamp) / 1000LL;
	if (delta_ms >= STATIC_INFO_TX_INTERVAL_MS || !neighbour_static_info.last_tx_timestamp) {
		neighbour_static_info_packet_t info = { .packet_type = WIRELESS_PACKET_TYPE_NEIGHBOUR_STATIC_INFO };
		memcpy(info.ap_password, wireless_get_ap_password(), WIRELESS_AP_PASSWORD_LENGTH);
		const esp_app_desc_t *app_desc = esp_app_get_description();
		memcpy(info.firmware_version, app_desc->version, sizeof(info.firmware_version));
		memcpy(info.firmware_sha256_hash, app_desc->app_elf_sha256, sizeof(info.firmware_sha256_hash));
		wireless_broadcast((const uint8_t *)&info, sizeof(info));
		neighbour_static_info.last_tx_timestamp = now;
	}
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
