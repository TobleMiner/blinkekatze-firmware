#include "color_channel_offset.h"

#include <string.h>

#include <esp_log.h>

#include "settings.h"

static const char *TAG = "color_channel_offset";

typedef struct color_channel_offset_packet {
	uint8_t packet_type;
	uint8_t color_channel;
	uint16_t offset;
	wireless_address_t addr;
} __attribute__((packed)) color_channel_offset_packet_t;


static void load_default_offsets(color_channel_offset_t *offset) {
	for (unsigned int i = 0; i < offset->num_offsets; i++) {
		unsigned int *channel_offset = &offset->offsets[i];
		*channel_offset = settings_get_color_channel_zero_offset(i);
	}
}

void color_channel_offset_init(color_channel_offset_t *offset, unsigned int *offsets, unsigned int num_offsets) {
	offset->offsets = offsets;
	offset->num_offsets = num_offsets;
	load_default_offsets(offset);
}

esp_err_t color_channel_set_offset(color_channel_offset_t *offset, unsigned int channel, unsigned int val) {
	if (channel >= offset->num_offsets) {
		return ESP_ERR_INVALID_ARG;
	}

	offset->offsets[channel] = val;
	settings_set_color_channel_zero_offset(channel, val);

	return 0;
}

void color_channel_offset_rx(color_channel_offset_t *offset, const wireless_packet_t *packet) {
	if (packet->len < sizeof(color_channel_offset_packet_t)) {
		ESP_LOGE(TAG, "Short packet received. Expected %zu bytes but got %u bytes", sizeof(color_channel_offset_packet_t), packet->len);
		return;
	}

	color_channel_offset_packet_t cc_packet;
	memcpy(&cc_packet, packet->data, sizeof(cc_packet));
	if (wireless_is_broadcast_address(cc_packet.addr) ||
	    wireless_is_local_address(cc_packet.addr)) {
		color_channel_set_offset(offset, cc_packet.color_channel, cc_packet.offset);
	}
}

void color_channel_offset_tx(uint8_t channel, uint16_t offset, const uint8_t *address) {
	color_channel_offset_packet_t cc_packet = {
		WIRELESS_PACKET_TYPE_COLOR_CHANNEL_ZERO_OFFSET,
		channel,
		offset,
		{ }
	};
	memcpy(cc_packet.addr, address, sizeof(wireless_address_t));
	wireless_broadcast((const uint8_t *)&cc_packet, sizeof(cc_packet));
}
