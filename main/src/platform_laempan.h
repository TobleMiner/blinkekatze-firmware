#pragma once

#include "color_channel_offset.h"
#include "platform.h"

typedef struct platform_laempan {
	platform_t base;
	color_channel_offset_t cc_offset;
	unsigned int color_channel_offsets[4];
	unsigned int default_color;
	unsigned int white_brightness;
} platform_laempan_t;

extern platform_def_t platform_laempan;

esp_err_t platform_laempan_probe(platform_t **ret);
bool platform_is_laempan(const platform_t *platform);
void platform_brightness_white_tx(platform_t *platform, uint16_t brightness, const uint8_t *address);
