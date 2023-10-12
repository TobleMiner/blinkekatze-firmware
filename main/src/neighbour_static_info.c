#include "neighbour_static_info.h"

#include <esp_app_desc.h>
#include <esp_log.h>
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
	if (delta_ms >= STATIC_INFO_TX_INTERVAL_MS) {
		neighbour_static_info_packet_t info = { .packet_type = WIRELESS_PACKET_TYPE_NEIGHBOUR_STATIC_INFO };
		memcpy(info.ap_password, wireless_get_ap_password(), WIRELESS_AP_PASSWORD_LENGTH);
		const esp_app_desc_t *app_desc = esp_app_get_description();
		memcpy(info.firmware_version, app_desc->version, sizeof(info.firmware_version));
		memcpy(info.firmware_sha256_hash, app_desc->app_elf_sha256, sizeof(info.firmware_sha256_hash));
		wireless_broadcast((const uint8_t *)&info, sizeof(info));
		neighbour_static_info.last_tx_timestamp = now;
	}
}
