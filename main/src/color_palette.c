#include "color_palette.h"

#include "util.h"

static const color_palette_color_t rainbow_stripe_colors[] = {
	{ { 0xff, 0x00, 0x00 }, 1 },
	{ { 0x00, 0x00, 0x00 }, 1 },
	{ { 0xab, 0x55, 0x00 }, 1 },
	{ { 0x00, 0x00, 0x00 }, 1 },
	{ { 0xab, 0xab, 0x00 }, 1 },
	{ { 0x00, 0x00, 0x00 }, 1 },
	{ { 0x00, 0xff, 0x00 }, 1 },
	{ { 0x00, 0x00, 0x00 }, 1 },
	{ { 0x00, 0xab, 0x55 }, 1 },
	{ { 0x00, 0x00, 0x00 }, 1 },
	{ { 0x00, 0x00, 0xff }, 1 },
	{ { 0x00, 0x00, 0x00 }, 1 },
	{ { 0x55, 0x00, 0xab }, 1 },
	{ { 0x00, 0x00, 0x00 }, 1 },
	{ { 0xab, 0x00, 0x55 }, 1 },
	{ { 0x00, 0x00, 0x00 }, 1 },
};

const color_palette_t rainbow_stripe_palette = {
	.colors = rainbow_stripe_colors,
	.num_colors = ARRAY_SIZE(rainbow_stripe_colors),
	.color_maxval = 0xff
};

static const color_palette_color_t aurora_colors[] = {
	{ { 1, 5, 45 }, 1 },
	{ { 0, 200, 23 }, 1 },
	{ { 0, 255, 0 }, 1 },
	{ { 0, 243, 45 }, 1 },
	{ { 0, 135, 7 }, 1 },
	{ { 1, 5, 45 }, 1 },
};

const color_palette_t aurora_palette = {
	.colors = aurora_colors,
	.num_colors = ARRAY_SIZE(aurora_colors),
	.color_maxval = 0xff
};

static const color_palette_color_t aurora2_colors[] = {
	{ { 17, 177, 13 }, 1 },
	{ { 121, 142, 5 }, 1 },
	{ { 25, 173, 121 }, 1 },
	{ { 250, 77, 127 }, 1 },
	{ { 171, 101, 221 }, 1 },
};

const color_palette_t aurora2_palette = {
	.colors = aurora2_colors,
	.num_colors = ARRAY_SIZE(aurora2_colors),
	.color_maxval = 0xff
};
