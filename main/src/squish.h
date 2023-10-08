#pragma once

#include <stdint.h>

#include <esp_err.h>

#include "color.h"
#include "spl06.h"

typedef struct squish {
	spl06_t *baro;
	int32_t pressure_at_rest;
	unsigned int num_pressure_samples;
	unsigned int squishedness;
	int64_t timestamp_last_update_us;
} squish_t;

void squish_init(squish_t *squish, spl06_t *baro);
esp_err_t squish_update(squish_t *squish);
void squish_apply(const squish_t *squish, color_hsv_t *color);
