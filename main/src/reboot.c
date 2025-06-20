#include "reboot.h"

#include <string.h>

#include <esp_log.h>
#include <esp_mac.h>
#include <esp_system.h>

typedef struct reboot_packet {
	uint8_t packet_type;
	uint8_t node_address[ESP_NOW_ETH_ALEN];
} __attribute__((packed)) reboot_packet_t;

static const char *TAG = "reboot";

void reboot_tx(const uint8_t *address) {
	reboot_packet_t reboot_packet = {
		.packet_type = WIRELESS_PACKET_TYPE_UID
	};
	memcpy(reboot_packet.node_address, address, sizeof(reboot_packet.node_address));
	ESP_LOGD(TAG, "rebooting "MACSTR,
		 MAC2STR(reboot_packet.node_address));
	wireless_broadcast((const uint8_t *)&reboot_packet, sizeof(reboot_packet));
}

esp_err_t reboot_rx(const wireless_packet_t *packet) {
	reboot_packet_t reboot_packet;
	if (packet->len < sizeof(reboot_packet)) {
		ESP_LOGD(TAG, "Short reboot packet received. Expected %u byte but got only %u bytes\n",
			 sizeof(reboot_packet), (unsigned int)packet->len);
		return ESP_ERR_INVALID_ARG;
	}
	memcpy(&reboot_packet, packet->data, sizeof(reboot_packet));

	ESP_LOGV(TAG, "Got reboot for "MACSTR,
		 MAC2STR(reboot_packet.node_address));
	const uint8_t *mac_address = wireless_get_mac_address();
	const uint8_t *bcast_address = wireless_get_broadcast_address();
	if (!memcmp(mac_address, reboot_packet.node_address, sizeof(reboot_packet.node_address)) ||
	    !memcmp(bcast_address, reboot_packet.node_address, sizeof(reboot_packet.node_address))) {
		esp_restart();
	}

	return ESP_OK;
}
