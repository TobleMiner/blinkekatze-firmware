#include "uid.h"

#include <stdbool.h>
#include <stdint.h>

#include <esp_log.h>
#include <esp_mac.h>

#include "neighbour.h"

#define UID_BLINK_INTERVAL_MS 500

typedef struct uid_packet {
	uint8_t packet_type;
	uint8_t enable;
	uint8_t node_address[ESP_NOW_ETH_ALEN];
} __attribute__((packed)) uid_packet_t;

static const char *TAG = "uid";

static bool uid_enabled = false;

void uid_enable(const uint8_t *address, bool enable) {
	uid_packet_t uid_packet = {
		.packet_type = WIRELESS_PACKET_TYPE_UID,
		.enable = enable ? 1: 0
	};
	memcpy(uid_packet.node_address, address, sizeof(uid_packet.node_address));
	ESP_LOGD(TAG, "%s UID on "MACSTR,
		 enable ? "enabling" : "disabling",
		 MAC2STR(uid_packet.node_address));
	wireless_broadcast((const uint8_t *)&uid_packet, sizeof(uid_packet));
}

esp_err_t uid_rx(const wireless_packet_t *packet) {
	uid_packet_t uid_packet;
	if (packet->len < sizeof(uid_packet)) {
		ESP_LOGD(TAG, "Short uid packet received. Expected %u byte but got only %u bytes\n",
			 sizeof(uid_packet), (unsigned int)packet->len);
		return ESP_ERR_INVALID_ARG;
	}
	memcpy(&uid_packet, packet->data, sizeof(uid_packet));

	ESP_LOGV(TAG, "Got %s UID for "MACSTR,
		 uid_packet.enable ? "enable" : "disable",
		 MAC2STR(uid_packet.node_address));
	const uint8_t *mac_address = wireless_get_mac_address();
	const uint8_t *bcast_address = wireless_get_broadcast_address();
	if (!memcmp(mac_address, uid_packet.node_address, sizeof(uid_packet.node_address)) ||
	    !memcmp(bcast_address, uid_packet.node_address, sizeof(uid_packet.node_address))) {
		uid_enabled = !!uid_packet.enable;
	}

	return ESP_OK;
}

void uid_apply(color_hsv_t *color) {
	if (uid_enabled) {
		int64_t now = neighbour_get_global_clock();
		int32_t now_ms = now / 1000LL;
		unsigned int cycle_ms = now_ms % (UID_BLINK_INTERVAL_MS * 2);

		color->s = 0;
		if (cycle_ms >= UID_BLINK_INTERVAL_MS) {
			color->v = HSV_VAL_MAX;
		} else {
			color->v = 0;
		}
	}
}
