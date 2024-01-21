#pragma once

#include <stdbool.h>

#include <freertos/FreeRTOS.h>

#include <esp_err.h>

#include "i2c_bus.h"

typedef enum bq27546_flash_cmd {
	BQ27546_FLASH_CMD_WRITE,
	BQ27546_FLASH_CMD_COMPARE,
	BQ27546_FLASH_CMD_WAIT
} bq27546_flash_cmd_t;

typedef struct bq27546_flash_op {
	bq27546_flash_cmd_t type;
	union {
		struct {
			uint8_t i2c_address;
			const uint8_t *data;
			size_t len;
		} write;
		struct {
			uint8_t i2c_address;
			uint8_t reg;
			const uint8_t *data;
			size_t len;
		} compare;
		struct {
			unsigned int delay_ms;
		} wait;
	};
} bq27546_flash_op_t;

typedef struct bq27546 {
	i2c_bus_t *i2c_bus;
	SemaphoreHandle_t lock;
	StaticSemaphore_t lock_buffer;
} bq27546_t;

esp_err_t bq27546_init(bq27546_t *bq, i2c_bus_t *bus);
esp_err_t bq27546_write_flash(bq27546_t *bq, const bq27546_flash_op_t *flash_ops, size_t num_ops);
int bq27546_get_voltage_mv(bq27546_t *bq);
esp_err_t bq27546_get_current_ma(bq27546_t *bq, int *current_ma_out);
int bq27546_get_state_of_charge_percent(bq27546_t *bq);
int bq27546_get_state_of_health_percent(bq27546_t *bq);
int bq27546_get_time_to_empty_min(bq27546_t *bq);
int bq27546_get_temperature_0_1k(bq27546_t *bq);
int bq27546_get_full_charge_capacity_mah(bq27546_t *bq);
int bq27546_get_remaining_capacity_mah(bq27546_t *bq);
int bq27546_is_sealed(bq27546_t *bq);
esp_err_t bq27546_seal(bq27546_t *bq);
esp_err_t bq27546_it_enable(bq27546_t *bq);
