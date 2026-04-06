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
	unsigned int step_in_palette = DIV_ROUND((uint32_t)pos * num_steps, COLOR_PALETTE_LENGTH);
	unsigned int idx_before = wled_get_color_idx_before_pos(palette, step_in_palette);
	unsigned int idx_after = wled_get_color_idx_after_pos(palette, step_in_palette);

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
		unsigned int step_before = WLED_STEP(palette_data, idx_before);
		unsigned int step_after = WLED_STEP(palette_data, idx_after);

		unsigned int pos_before = DIV_ROUND((uint32_t)step_before * COLOR_PALETTE_LENGTH, num_steps);
		unsigned int pos_after = DIV_ROUND((uint32_t)step_after * COLOR_PALETTE_LENGTH, num_steps);
		unsigned int pos_length = pos_after - pos_before;
		unsigned int pos_progress = pos - pos_before;

		uint16_t blend_progress = 0;
		if (pos_after > pos_before && pos >= pos_before) {
			blend_progress = (uint32_t)pos_progress * COLOR_BLEND_MAX / pos_length;
		}

		ESP_LOGV(TAG, "Blending %u -> %u, pos: %u, blend: %u", idx_before, idx_after, pos, blend_progress);
		color_blend_rgb(color, &color_before, &color_after, blend_progress);
	}
}

static const color_palette_ops_t wled_color_palette_ops = {
	.get_total_length = wled_color_palette_get_num_steps,
	.get_color_at = wled_color_palette_get_color_at
};

/* Manually imported palettes */
const uint8_t CloudColors_p[] = {
	 0, 0x00, 0x00, 0xff, // Blue
	 1, 0x00, 0x00, 0x8b, // DarkBlue
	 7, 0x00, 0x00, 0x8b, // DarkBlue
	 8, 0x00, 0x00, 0xff, // Blue
	 9, 0x00, 0x00, 0x8b, // DarkBlue
	10, 0x87, 0xce, 0xeb, // SkyBlue
	11, 0x87, 0xce, 0xeb, // SkyBlue
	12, 0xad, 0xd8, 0xe6, // LightBlue
	13, 0xff, 0xff, 0xff, // White
	14, 0xad, 0xd8, 0xe6, // LightBlue
	15, 0x87, 0xce, 0xeb, // SkyBlue
};

const uint8_t LavaColors_p[] = {
	 0, 0x00, 0x00, 0x00, // Black
	 1, 0x80, 0x00, 0x00, // Maroon
	 2, 0x00, 0x00, 0x00, // Black
	 3, 0x80, 0x00, 0x00, // Maroon
	 4, 0x8b, 0x00, 0x00, // DarkRed
	 5, 0x8b, 0x00, 0x00, // DarkRed
	 6, 0x80, 0x00, 0x00, // Maroon
	 7, 0x8b, 0x00, 0x00, // DarkRed
	 9, 0x8b, 0x00, 0x00, // DarkRed
	10, 0xff, 0x00, 0x00, // Red
	11, 0xff, 0xa5, 0x00, // Orange
	12, 0xff, 0xff, 0xff, // White
	13, 0xff, 0xa5, 0x00, // Orange
	14, 0xff, 0x00, 0x00, // Red
	15, 0x8b, 0x00, 0x00, // DarkRed
};

const uint8_t OceanColors_p[] = {
	 0, 0x19, 0x19, 0x70, // MidnightBlue
	 1, 0x00, 0x00, 0x8b, // DarkBlue
	 2, 0x19, 0x19, 0x70, // MidnightBlue
	 3, 0x00, 0x00, 0x80, // Navy
	 4, 0x00, 0x00, 0x8b, // DarkBlue
	 5, 0x00, 0x00, 0xcd, // MediumBlue
	 6, 0x2e, 0x8B, 0x57, // SeaGreen
	 7, 0x00, 0x80, 0x80, // Teal
	 8, 0x5f, 0x9e, 0xa0, // CadetBlue
	 9, 0x00, 0x00, 0xff, // Blue
	10, 0x00, 0x8b, 0x8b, // DarkCyan
	11, 0x64, 0x95, 0xed, // CornflowerBlue
	12, 0x7f, 0xff, 0xd4, // Aquamarine
	13, 0x2e, 0x8B, 0x57, // SeaGreen
	14, 0x00, 0xff, 0xff, // Aqua
	15, 0x87, 0xce, 0xfA, // LightSkyBlue
};

const uint8_t ForestColors_p[] = {
	 0, 0x00, 0x64, 0x00, // DarkGreen
	 1, 0x00, 0x64, 0x00, // DarkGreen
	 2, 0x55, 0x6b, 0x2f, // DarkOliveGreen
	 3, 0x00, 0x64, 0x00, // DarkGreen
	 4, 0x00, 0xff, 0x00, // Green
	 5, 0x22, 0x8b, 0x22, // ForestGreen
	 6, 0x6b, 0x8e, 0x23, // OliveDrab
	 7, 0x00, 0xff, 0x00, // Green
	 8, 0x2e, 0x8B, 0x57, // SeaGreen
	 9, 0x66, 0xcd, 0xaa, // MediumAquamarine
	10, 0x32, 0xcd, 0x32, // LimeGreen
	11, 0x9a, 0xcd, 0x32, // YellowGreen
	12, 0x90, 0xee, 0x90, // LightGreen
	13, 0x7c, 0xfc, 0x00, // LawnGreen
	14, 0x66, 0xcd, 0xaa, // MediumAquamarine
	15, 0x22, 0x8b, 0x22, // ForestGreen
};

// Party colors
const uint8_t PartyColors_gc22[] = {
	 0, 0x9B, 0x00, 0xD5,
	 1, 0xBD, 0x00, 0xB8,
	 2, 0xDA, 0x00, 0x92,
	 3, 0xF3, 0x00, 0x5C,
	 4, 0xF4, 0x55, 0x00,
	 5, 0xDC, 0x8F, 0x00,
	 6, 0xD5, 0xB4, 0x00,
	 7, 0xD5, 0xD5, 0x00,
	 8, 0xD5, 0x9B, 0x00,
	 9, 0xEF, 0x66, 0x00,
	10, 0xF9, 0x00, 0x44,
	11, 0xE1, 0x00, 0x86,
	12, 0xC4, 0x00, 0xB0,
	13, 0xA3, 0x00, 0xCF,
	14, 0x76, 0x00, 0xE8,
	15, 0x00, 0x32, 0xFC
};

// Rainbow colors
const uint8_t RainbowColors_gc22[] = {
	 0, 0xFF, 0x00, 0x00,
	 1, 0xEB, 0x70, 0x00,
	 2, 0xD5, 0x9B, 0x00,
	 3, 0xD5, 0xBA, 0x00,
	 4, 0xD5, 0xD5, 0x00,
	 5, 0x9C, 0xEB, 0x00,
	 6, 0x00, 0xFF, 0x00,
	 7, 0x00, 0xEB, 0x70,
	 8, 0x00, 0xD5, 0x9B,
	 9, 0x00, 0x9C, 0xD4,
	10, 0x00, 0x00, 0xFF,
	11, 0x70, 0x00, 0xEB,
	12, 0x9B, 0x00, 0xD5,
	13, 0xBA, 0x00, 0xBB,
	14, 0xD5, 0x00, 0x9B,
	15, 0xEB, 0x00, 0x72
};

// Rainbow colors with alternatating stripes of black
const uint8_t RainbowStripeColors_gc22[] = {
	 0, 0xFF, 0x00, 0x00,
	 1, 0x00, 0x00, 0x00,
	 2, 0xD5, 0x9B, 0x00,
	 3, 0x00, 0x00, 0x00,
	 4, 0xD5, 0xD5, 0x00,
	 5, 0x00, 0x00, 0x00,
	 6, 0x00, 0xFF, 0x00,
	 7, 0x00, 0x00, 0x00,
	 8, 0x00, 0xD5, 0x9B,
	 9, 0x00, 0x00, 0x00,
	10, 0x00, 0x00, 0xFF,
	11, 0x00, 0x00, 0x00,
	12, 0x9B, 0x00, 0xD5,
	13, 0x00, 0x00, 0x00,
	14, 0xD5, 0x00, 0x9B,
	15, 0x00, 0x00, 0x00
};

const WLED_DECLARE_PALETTE(wled_CloudColors_p, CloudColors_p);
const WLED_DECLARE_PALETTE(wled_LavaColors_p, LavaColors_p);
const WLED_DECLARE_PALETTE(wled_OceanColors_p, OceanColors_p);
const WLED_DECLARE_PALETTE(wled_ForestColors_p, ForestColors_p);

const WLED_DECLARE_PALETTE(wled_ib_jul01_gp, ib_jul01_gp);
const WLED_DECLARE_PALETTE(wled_es_vintage_57_gp, es_vintage_57_gp);
const WLED_DECLARE_PALETTE(wled_es_vintage_01_gp, es_vintage_01_gp);
const WLED_DECLARE_PALETTE(wled_es_rivendell_15_gp, es_rivendell_15_gp);
const WLED_DECLARE_PALETTE(wled_rgi_15_gp, rgi_15_gp);
const WLED_DECLARE_PALETTE(wled_retro2_16_gp, retro2_16_gp);
const WLED_DECLARE_PALETTE(wled_Analogous_1_gp, Analogous_1_gp);
const WLED_DECLARE_PALETTE(wled_es_pinksplash_08_gp, es_pinksplash_08_gp);
const WLED_DECLARE_PALETTE(wled_es_ocean_breeze_036_gp, es_ocean_breeze_036_gp);
const WLED_DECLARE_PALETTE(wled_departure_gp, departure_gp);
const WLED_DECLARE_PALETTE(wled_es_landscape_64_gp, es_landscape_64_gp);
const WLED_DECLARE_PALETTE(wled_es_landscape_33_gp, es_landscape_33_gp);
const WLED_DECLARE_PALETTE(wled_rainbowsherbet_gp, rainbowsherbet_gp);
const WLED_DECLARE_PALETTE(wled_gr65_hult_gp, gr65_hult_gp);
const WLED_DECLARE_PALETTE(wled_gr64_hult_gp, gr64_hult_gp);
const WLED_DECLARE_PALETTE(wled_GMT_drywet_gp, GMT_drywet_gp);
const WLED_DECLARE_PALETTE(wled_ib15_gp, ib15_gp);
const WLED_DECLARE_PALETTE(wled_Tertiary_01_gp, Tertiary_01_gp);
const WLED_DECLARE_PALETTE(wled_lava_gp, lava_gp);
const WLED_DECLARE_PALETTE(wled_fierce_ice_gp, fierce_ice_gp);
const WLED_DECLARE_PALETTE(wled_Colorfull_gp, Colorfull_gp);
const WLED_DECLARE_PALETTE(wled_Pink_Purple_gp, Pink_Purple_gp);
const WLED_DECLARE_PALETTE(wled_Sunset_Real_gp, Sunset_Real_gp);
const WLED_DECLARE_PALETTE(wled_Sunset_Yellow_gp, Sunset_Yellow_gp);
const WLED_DECLARE_PALETTE(wled_Beech_gp, Beech_gp);
const WLED_DECLARE_PALETTE(wled_Another_Sunset_gp, Another_Sunset_gp);
const WLED_DECLARE_PALETTE(wled_es_autumn_19_gp, es_autumn_19_gp);
const WLED_DECLARE_PALETTE(wled_BlacK_Blue_Magenta_White_gp, BlacK_Blue_Magenta_White_gp);
const WLED_DECLARE_PALETTE(wled_BlacK_Magenta_Red_gp, BlacK_Magenta_Red_gp);
const WLED_DECLARE_PALETTE(wled_BlacK_Red_Magenta_Yellow_gp, BlacK_Red_Magenta_Yellow_gp);
const WLED_DECLARE_PALETTE(wled_Blue_Cyan_Yellow_gp, Blue_Cyan_Yellow_gp);
const WLED_DECLARE_PALETTE(wled_Orange_Teal_gp, Orange_Teal_gp);
const WLED_DECLARE_PALETTE(wled_Tiamat_gp, Tiamat_gp);
const WLED_DECLARE_PALETTE(wled_April_Night_gp, April_Night_gp);
const WLED_DECLARE_PALETTE(wled_Orangery_gp, Orangery_gp);
const WLED_DECLARE_PALETTE(wled_C9_gp, C9_gp);
const WLED_DECLARE_PALETTE(wled_Sakura_gp, Sakura_gp);
const WLED_DECLARE_PALETTE(wled_Aurora_gp, Aurora_gp);
const WLED_DECLARE_PALETTE(wled_Atlantica_gp, Atlantica_gp);
const WLED_DECLARE_PALETTE(wled_C9_2_gp, C9_2_gp);
const WLED_DECLARE_PALETTE(wled_C9_new_gp, C9_new_gp);
const WLED_DECLARE_PALETTE(wled_temperature_gp, temperature_gp);
const WLED_DECLARE_PALETTE(wled_retro_clown_gp, retro_clown_gp);
const WLED_DECLARE_PALETTE(wled_candy_gp, candy_gp);
const WLED_DECLARE_PALETTE(wled_toxy_reaf_gp, toxy_reaf_gp);
const WLED_DECLARE_PALETTE(wled_fairy_reaf_gp, fairy_reaf_gp);
const WLED_DECLARE_PALETTE(wled_semi_blue_gp, semi_blue_gp);
const WLED_DECLARE_PALETTE(wled_pink_candy_gp, pink_candy_gp);
const WLED_DECLARE_PALETTE(wled_red_reaf_gp, red_reaf_gp);
const WLED_DECLARE_PALETTE(wled_aqua_flash_gp, aqua_flash_gp);
const WLED_DECLARE_PALETTE(wled_yelblu_hot_gp, yelblu_hot_gp);
const WLED_DECLARE_PALETTE(wled_lite_light_gp, lite_light_gp);
const WLED_DECLARE_PALETTE(wled_red_flash_gp, red_flash_gp);
const WLED_DECLARE_PALETTE(wled_blink_red_gp, blink_red_gp);
const WLED_DECLARE_PALETTE(wled_red_shift_gp, red_shift_gp);
const WLED_DECLARE_PALETTE(wled_red_tide_gp, red_tide_gp);
const WLED_DECLARE_PALETTE(wled_candy2_gp, candy2_gp);
const WLED_DECLARE_PALETTE(wled_trafficlight_gp, trafficlight_gp);
const WLED_DECLARE_PALETTE(wled_Aurora2_gp, Aurora2_gp);

const WLED_DECLARE_PALETTE(wled_PartyColors_gc22, PartyColors_gc22);
const WLED_DECLARE_PALETTE(wled_RainbowColors_gc22, RainbowColors_gc22);
const WLED_DECLARE_PALETTE(wled_RainbowStripeColors_gc22, RainbowStripeColors_gc22);

const color_palette_t *wled_color_palettes[] = {
	&wled_CloudColors_p,
	&wled_LavaColors_p,
	&wled_OceanColors_p,
	&wled_ForestColors_p,
	&wled_ib_jul01_gp,
	&wled_es_vintage_57_gp,
	&wled_es_vintage_01_gp,
	&wled_es_rivendell_15_gp,
	&wled_rgi_15_gp,
	&wled_retro2_16_gp,
	&wled_Analogous_1_gp,
	&wled_es_pinksplash_08_gp,
	&wled_es_ocean_breeze_036_gp,
	&wled_departure_gp,
	&wled_es_landscape_64_gp,
	&wled_es_landscape_33_gp,
	&wled_rainbowsherbet_gp,
	&wled_gr65_hult_gp,
	&wled_gr64_hult_gp,
	&wled_GMT_drywet_gp,
	&wled_ib15_gp,
	&wled_Tertiary_01_gp,
	&wled_lava_gp,
	&wled_fierce_ice_gp,
	&wled_Colorfull_gp,
	&wled_Pink_Purple_gp,
	&wled_Sunset_Real_gp,
	&wled_Sunset_Yellow_gp,
	&wled_Beech_gp,
	&wled_Another_Sunset_gp,
	&wled_es_autumn_19_gp,
	&wled_BlacK_Blue_Magenta_White_gp,
	&wled_BlacK_Magenta_Red_gp,
	&wled_BlacK_Red_Magenta_Yellow_gp,
	&wled_Blue_Cyan_Yellow_gp,
	&wled_Orange_Teal_gp,
	&wled_Tiamat_gp,
	&wled_April_Night_gp,
	&wled_Orangery_gp,
	&wled_C9_gp,
	&wled_Sakura_gp,
	&wled_Aurora_gp,
	&wled_Atlantica_gp,
	&wled_C9_2_gp,
	&wled_C9_new_gp,
	&wled_temperature_gp,
	&wled_retro_clown_gp,
	&wled_candy_gp,
	&wled_toxy_reaf_gp,
	&wled_fairy_reaf_gp,
	&wled_semi_blue_gp,
	&wled_pink_candy_gp,
	&wled_red_reaf_gp,
	&wled_aqua_flash_gp,
	&wled_yelblu_hot_gp,
	&wled_lite_light_gp,
	&wled_red_flash_gp,
	&wled_blink_red_gp,
	&wled_red_shift_gp,
	&wled_red_tide_gp,
	&wled_candy2_gp,
	&wled_trafficlight_gp,
	&wled_Aurora2_gp,
	&wled_PartyColors_gc22,
	&wled_RainbowColors_gc22,
	&wled_RainbowStripeColors_gc22
};
