#include "color_palette_wled.h"

#include <stdint.h>

#include <esp_log.h>

#include "color.h"
#include "util.h"

#define PROGMEM
#define byte uint8_t

#include "color_palette_wled.inc"

#define WLED_MUM_COLORS(palette) (sizeof(palette) / 4)

#define WLED_DECLARE_PALETTE(name, palette_data) \
color_palette_t name = { \
	.data = palette_data, \
	.ops = &wled_color_palette_ops, \
	.num_colors = WLED_MUM_COLORS(palette_data), \
	.color_maxval = 0xffff \
};

#define WLED_STEP(palette_data, idx) (palette_data)[(idx) * 4]
#define WLED_R(palette_data, idx) (palette_data)[(idx) * 4 + 1]
#define WLED_G(palette_data, idx) (palette_data)[(idx) * 4 + 2]
#define WLED_B(palette_data, idx) (palette_data)[(idx) * 4 + 3]

static const char *TAG = "WLED";

static unsigned int wled_color_palette_get_num_steps(const color_palette_t *palette) {
	const uint8_t *palette_data = palette->data;

	unsigned int max_step = 0;
	for (int i = 0; i < palette->num_colors; i++) {
		max_step = MAX(max_step, WLED_STEP(palette_data, i));
	}

	return max_step;
}

static unsigned int wled_get_color_idx_before_pos(const color_palette_t *palette, unsigned int pos) {
	const uint8_t *palette_data = palette->data;

	for (int i = 1; i < palette->num_colors; i++) {
		if (WLED_STEP(palette_data, i) > pos) {
			return i - 1;
		}
	}

	return palette->num_colors - 1;
}

static unsigned int wled_get_color_idx_after_pos(const color_palette_t *palette, unsigned int pos) {
	const uint8_t *palette_data = palette->data;

	for (int i = 1; i < palette->num_colors; i++) {
		if (WLED_STEP(palette_data, i) >= pos) {
			return i;
		}
	}

	return palette->num_colors - 1;
}

static void wled_get_color(rgb16_t *color, const color_palette_t *palette, unsigned int idx) {
	const uint8_t *palette_data = palette->data;

	color->r = WLED_R(palette_data, idx);
	color->g = WLED_G(palette_data, idx);
	color->b = WLED_B(palette_data, idx);
}

static void wled_color_palette_get_color_at(color_t *color, const color_palette_t *palette, unsigned int pos) {
	const uint8_t *palette_data = palette->data;

	unsigned int num_steps = wled_color_palette_get_num_steps(palette);
	unsigned int idx_before = wled_get_color_idx_before_pos(palette, pos);
	unsigned int idx_after = wled_get_color_idx_after_pos(palette, pos);

	rgb16_t color_before, color_after;
	wled_get_color(&color_before, palette, idx_before);
	color_scale_rgb(&color_before, 0xff, 0xffff);

	wled_get_color(&color_after, palette, idx_after);
	color_scale_rgb(&color_after, 0xff, 0xffff);


	if (idx_before == idx_after) {
		ESP_LOGV(TAG, "Showing %u, pos: %u", idx_before, pos);
		color->format = COLOR_FORMAT_RGB16;
		color->rgb = color_before;
	} else {
		// TODO: Blend prevision is inherently limited by th enumber of steps between colors in the palette
		unsigned int step_before = WLED_STEP(palette_data, idx_before);
		unsigned int step_after = WLED_STEP(palette_data, idx_after);
		unsigned int step_length = step_after - step_before;
		unsigned int step_progress = pos - step_before;

		uint16_t blend_progress = 0;
		if (step_after > step_before && pos >= step_before) {
			blend_progress = (uint32_t)step_progress * COLOR_BLEND_MAX / step_length;
		}

		ESP_LOGV(TAG, "Blending %u -> %u, pos: %u, blend: %u", idx_before, idx_after, pos, blend_progress);
		color_blend_rgb(color, &color_before, &color_after, blend_progress);
	}
}

static const color_palette_ops_t wled_color_palette_ops = {
	.get_total_length = wled_color_palette_get_num_steps,
	.get_color_at = wled_color_palette_get_color_at
};

const WLED_DECLARE_PALETTE(wled_palette_autumn, es_autumn_19_gp);
