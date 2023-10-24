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
