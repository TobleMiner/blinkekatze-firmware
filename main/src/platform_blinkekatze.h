#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <driver/spi_master.h>
#include <esp_err.h>

#include "i2c_bus.h"
#include "bq24295.h"
#include "bq27546.h"
#include "lis3dh.h"
#include "ltr_303als.h"
#include "platform.h"
#include "spl06.h"

typedef struct platform_blinkekatze {
	platform_t base;
	i2c_bus_t i2c_bus;
	bq27546_t gauge;
	bq24295_t charger;
	ltr_303als_t als;
	lis3dh_t accelerometer;
	spl06_t barometer;
	bool is_rev2;
	bool transaction_pending;
	spi_device_handle_t dev;
	uint8_t *led_data;
	spi_transaction_t xfer;
} platform_blinkekatze_t;

esp_err_t platform_blinkekatze_probe(platform_t **ret);
