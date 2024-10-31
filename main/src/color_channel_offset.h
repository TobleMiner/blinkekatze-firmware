#pragma once

#include <stdint.h>

#include "wireless.h"

typedef struct color_channel_offset {
	unsigned int *offsets;
	unsigned int num_offsets;
} color_channel_offset_t;

void color_channel_offset_init(color_channel_offset_t *offset, unsigned int *offsets, unsigned int num_offset);
void color_channel_offset_rx(color_channel_offset_t *offset, const wireless_packet_t *packet);
esp_err_t color_channel_set_offset(color_channel_offset_t *offset, unsigned int channel, unsigned int val);
void color_channel_offset_tx(uint8_t channel, uint16_t offset, const uint8_t *address);
