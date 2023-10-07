#pragma once

#include <stdint.h>

#include <esp_err.h>

#include "i2c_bus.h"

typedef struct lis3dh_sample {
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((packed)) lis3dh_sample_t;

typedef struct lis3dh {
	i2c_bus_t *bus;
	uint8_t address;
	lis3dh_sample_t fifo_buf[32];
} lis3dh_t;

esp_err_t lis3dh_init(lis3dh_t *lis, i2c_bus_t *bus, uint8_t address);
esp_err_t lis3dh_update(lis3dh_t *lis);
