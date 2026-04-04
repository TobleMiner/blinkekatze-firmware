#pragma once

#include <stdint.h>

#include "color.h"

typedef struct color_palette_color {
	rgb16_t color;
	unsigned int position;
} color_palette_color_t;

typedef struct color_palette_ops color_palette_ops_t;

typedef struct color_palette {
	const void *data;
	const color_palette_ops_t *ops;
	unsigned int num_colors;
	unsigned int color_maxval;
} color_palette_t;

struct color_palette_ops {
	unsigned int (*get_total_length)(const color_palette_t *palette);
	void (*get_color_at)(color_t *color, const color_palette_t *palette, unsigned int pos);
};

extern const color_palette_t rainbow_stripe_palette;
extern const color_palette_t aurora_palette;
extern const color_palette_t aurora2_palette;
extern const color_palette_t autumn_palette;

static unsigned int color_palette_get_total_length(const color_palette_t *palette) {
	return palette->ops->get_total_length(palette);
}

static void color_palette_get_color_at(color_t *color, const color_palette_t *palette, unsigned int pos) {
	palette->ops->get_color_at(color, palette, pos);
}
