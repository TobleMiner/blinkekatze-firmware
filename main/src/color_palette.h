#pragma once

#include "color.h"

typedef struct color_palette_color {
	rgb16_t color;
	unsigned int duration;
} color_palette_color_t;

typedef struct color_palette {
	const color_palette_color_t *colors;
	unsigned int num_colors;
	unsigned int color_maxval;
} color_palette_t;

extern const color_palette_t rainbow_stripe_palette;
