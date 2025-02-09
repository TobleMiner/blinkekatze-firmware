#include "color.h"

#include <stdint.h>

#include "fast_hsv2rgb.h"
#include "util.h"

// Safe to use for in-place modification,
// out and in memory locations may overlap.
void rgb16_to_hsv(color_hsv_t *out, const rgb16_t *in) {
	rgb16_t rgb = *in;
	uint16_t cmax = MAX(rgb.r, MAX(rgb.g, rgb.b));
	uint16_t cmin = MIN(rgb.r, MIN(rgb.g, rgb.b));

	uint16_t hue = 0;
	if (cmin != cmax) {
		int16_t sextant;
		int32_t color_delta;

		if (cmax == rgb.r) {
			sextant = 0;
			color_delta = (int32_t)rgb.g - (int32_t)rgb.b;
		} else if (cmax == rgb.g) {
			sextant = 2;
			color_delta = (int32_t)rgb.b - (int32_t)rgb.r;
		} else {
			sextant = 4;
			color_delta = (int32_t)rgb.r - (int32_t)rgb.g;
		}

		int32_t angle = sextant * HSV_HUE_SEXTANT;
		angle += DIV_ROUND(color_delta * HSV_HUE_SEXTANT, (int32_t)(cmax - cmin));
		hue = angle % HSV_HUE_STEPS;
	}

	uint16_t saturation = 0;
	if (cmax) {
		saturation = DIV_ROUND(((uint32_t)cmax - (uint32_t)cmin) * (uint32_t)HSV_SAT_MAX, (uint32_t)cmax);
	}

	uint16_t value = DIV_ROUND((int32_t)cmax * HSV_VAL_MAX, 65535L);

	out->h = hue;
	out->s = saturation;
	out->v = value;
}

static void color_to_hsv_inplace(color_t *color) {
	if (color->format != COLOR_FORMAT_HSV16) {
		rgb16_to_hsv(&color->hsv, &color->rgb);
		color->format = COLOR_FORMAT_HSV16;
	}
}

static void color_to_rgb_inplace(color_t *color) {
	if (color->format != COLOR_FORMAT_RGB16) {
		color_hsv_t hsv = color->hsv;
		uint16_t r, g, b;
		fast_hsv2rgb_32bit(hsv.h, hsv.s, hsv.v, &r, &g, &b);
		color->rgb.r = r;
		color->rgb.g = g;
		color->rgb.b = b;
		color->format = COLOR_FORMAT_RGB16;
	}
}

color_hsv_t *color_to_hsv(color_t *color) {
	color_to_hsv_inplace(color);
	return &color->hsv;
}

rgb16_t *color_to_rgb(color_t *color) {
	color_to_rgb_inplace(color);
	return &color->rgb;
}

void color_blend_rgb(color_t *result, const rgb16_t *rgb_from, const rgb16_t *rgb_to, uint16_t progress) {
	int32_t delta_r = (int32_t)rgb_to->r - (int32_t)rgb_from->r;
	int32_t delta_g = (int32_t)rgb_to->g - (int32_t)rgb_from->g;
	int32_t delta_b = (int32_t)rgb_to->b - (int32_t)rgb_from->b;

	result->rgb.r = (int32_t)rgb_from->r + DIV_ROUND(delta_r * (int32_t)progress, COLOR_BLEND_MAX);
	result->rgb.g = (int32_t)rgb_from->g + DIV_ROUND(delta_g * (int32_t)progress, COLOR_BLEND_MAX);
	result->rgb.b = (int32_t)rgb_from->b + DIV_ROUND(delta_b * (int32_t)progress, COLOR_BLEND_MAX);
/*
	result->rgb.r = DIV_ROUND((int32_t)rgb_from->r * (COLOR_BLEND_MAX - (int32_t)progress) + (int32_t)rgb_to->r * (int32_t)progress, COLOR_BLEND_MAX);
	result->rgb.g = DIV_ROUND((int32_t)rgb_from->g * (COLOR_BLEND_MAX - (int32_t)progress) + (int32_t)rgb_to->g * (int32_t)progress, COLOR_BLEND_MAX);
	result->rgb.b = DIV_ROUND((int32_t)rgb_from->b * (COLOR_BLEND_MAX - (int32_t)progress) + (int32_t)rgb_to->b * (int32_t)progress, COLOR_BLEND_MAX);
*/
	result->format = COLOR_FORMAT_RGB16;
}

void color_blend(color_t *result, const color_t *from, const color_t *to, uint16_t progress) {
	rgb16_t *rgb_from = color_to_rgb(from);
	rgb16_t *rgb_to = color_to_rgb(to);

	color_blend_rgb(result, rgb_from, rgb_to, progress);
	result->format = COLOR_FORMAT_RGB16;
}

void color_scale_rgb(rgb16_t *rgb, unsigned int max_from, unsigned int max_to) {
	rgb->r = DIV_ROUND((int32_t)rgb->r * (int32_t)max_to, (int32_t)max_from);
	rgb->g = DIV_ROUND((int32_t)rgb->g * (int32_t)max_to, (int32_t)max_from);
	rgb->b = DIV_ROUND((int32_t)rgb->b * (int32_t)max_to, (int32_t)max_from);
}
