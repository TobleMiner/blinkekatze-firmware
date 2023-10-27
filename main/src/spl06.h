#pragma once

#include <stdbool.h>

#include <driver/spi_master.h>
#include <esp_err.h>

#include "i2c_bus.h"

typedef struct spl06 {
	bool is_i2c;
	union {
		spi_device_handle_t spi;
		struct {
			i2c_bus_t *bus;
			uint8_t address;
		} i2c;
	};
	int32_t pressure;
} spl06_t;

esp_err_t spl06_init_spi(spl06_t *spl, spi_host_device_t spi_host, int gpio_cs);
esp_err_t spl06_init_i2c(spl06_t *spl, i2c_bus_t *bus, uint8_t address);
esp_err_t spl06_update(spl06_t *spl);
int32_t spl06_get_pressure(spl06_t *spl);
