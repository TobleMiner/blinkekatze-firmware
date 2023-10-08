#pragma once

#include <stdint.h>

#include <esp_err.h>

#include "i2c_bus.h"

typedef struct lis3dh_sample {
	union {
		struct {
			int16_t x;
			int16_t y;
			int16_t z;
		};
		int16_t xyz[3];
	};
} __attribute__((packed)) lis3dh_sample_t;

static_assert(sizeof(lis3dh_sample_t) == 6);

typedef struct lis3dh {
	i2c_bus_t *bus;
	uint8_t address;
	lis3dh_sample_t fifo_buf[32];
	bool click_detected;
	bool fifo_full;
} lis3dh_t;

esp_err_t lis3dh_init(lis3dh_t *lis, i2c_bus_t *bus, uint8_t address);
esp_err_t lis3dh_update(lis3dh_t *lis);
bool lis3dh_has_click_been_detected(lis3dh_t *lis);
uint16_t lis3dh_get_peak_click_acceleration(lis3dh_t *lis);
int32_t lis3dh_get_click_velocity(lis3dh_t *lis);
