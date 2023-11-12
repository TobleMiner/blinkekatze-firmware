#pragma once

#include <stdint.h>

#include <esp_err.h>

#include "i2c_bus.h"

typedef struct ltr_303als {
	i2c_bus_t *bus;
	uint32_t light_level_mlux_white;
	uint32_t light_level_mlux_ir;
} ltr_303als_t;

esp_err_t ltr_303als_init(ltr_303als_t *lis, i2c_bus_t *bus);
esp_err_t ltr_303als_update(ltr_303als_t *lis);
