#include "rainbow_fade.h"

#include "neighbour.h"

#define HUE_CYCLE_TIME_MS	10000

void rainbow_fade_apply(color_hsv_t *color) {
	int64_t now = neighbour_get_global_clock();
	int64_t cycle_val = now / 1000 * HSV_HUE_STEPS / HUE_CYCLE_TIME_MS;
	uint16_t hue_delta = cycle_val % HSV_HUE_STEPS;

	color->h = (color->h + hue_delta) % HSV_HUE_STEPS;
}
