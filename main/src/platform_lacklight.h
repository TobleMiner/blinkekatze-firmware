#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <driver/spi_master.h>
#include <esp_err.h>

#include "platform.h"

typedef struct platform_lacklight {
	platform_t base;
	bool transaction_pending;
	spi_device_handle_t dev;
	uint8_t *led_data;
	spi_transaction_t xfer;
} platform_lacklight_t;

esp_err_t platform_lacklight_probe(platform_t **ret);
