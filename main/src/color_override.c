#include "color_override.h"

typedef struct color_override {
	bool enabled;
	rgb16_t color;
} color_override_t;

static color_override_t color_override = { 0 };

void color_override_set_enable(bool enable) {
	color_override.enabled = enable;
}

void color_override_set_color(const rgb16_t *rgb) {
	color_override.color = *rgb;
}

void color_override_apply(rgb16_t *rgb) {
	if (color_override.enabled) {
		*rgb = color_override.color;
	}
}
