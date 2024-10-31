#pragma once

#include "color_channel_offset.h"
#include "platform.h"

typedef struct platform_laempan {
	platform_t base;
	color_channel_offset_t cc_offset;
	unsigned int color_channel_offsets[4];
} platform_laempan_t;

esp_err_t platform_laempan_probe(platform_t **ret);
