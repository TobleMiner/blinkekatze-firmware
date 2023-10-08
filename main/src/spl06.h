#pragma once

#include <stdbool.h>

#include <driver/spi_master.h>
#include <esp_err.h>

typedef struct spl06 {
	spi_device_handle_t spi;
	int32_t pressure;
} spl06_t;

esp_err_t spl06_init(spl06_t *spl, spi_host_device_t spi_host, int gpio_cs);
esp_err_t spl06_update(spl06_t *spl);
int32_t spl06_get_pressure(spl06_t *spl);
