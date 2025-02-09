#pragma once

#include <stdint.h>

#define HSV_HUE_SEXTANT		8192
#define HSV_HUE_STEPS		(6 * HSV_HUE_SEXTANT)

#define HSV_HUE_MIN		0
#define HSV_HUE_MAX		(HSV_HUE_STEPS - 1)
#define HSV_SAT_MIN		0
#define HSV_SAT_MAX		65535
#define HSV_VAL_MIN		0
#define HSV_VAL_MAX		65535

#define COLOR_BLEND_MIN		0
#define COLOR_BLEND_MAX		65535
#define COLOR_BLEND_STEPS	65536L

typedef struct color_hsv {
	uint16_t h;
	uint16_t s;
	uint16_t v;
} color_hsv_t;

typedef struct rgb16 {
	uint16_t r;
	uint16_t g;
	uint16_t b;
} __attribute__((packed)) rgb16_t;

typedef enum color_format {
	COLOR_FORMAT_RGB16,
	COLOR_FORMAT_HSV16
} color_format_t;

typedef struct color {
	color_format_t format;
	union {
		color_hsv_t hsv;
		rgb16_t rgb;
	};
} color_t;

color_hsv_t *color_to_hsv(color_t *color);
rgb16_t *color_to_rgb(color_t *color);
void color_blend(color_t *result, const color_t *from, const color_t *to, uint16_t progress);
void color_blend_rgb(color_t *result, const rgb16_t *rgb_from, const rgb16_t *rgb_to, uint16_t progress);
void color_scale_rgb(rgb16_t *rgb, unsigned int max_from, unsigned int max_to);
