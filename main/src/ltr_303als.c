#include "ltr_303als.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

#include "util.h"

#define ADDRESS 0x29

#define REG_CONTR	0x80
#define 	REG_CONTR_SW_RESET		BIT(1)
#define 	REG_CONTR_ALS_GAIN_96X		(0x7 << 2)
#define 	REG_CONTR_ALS_MODE_ACTIVE	BIT(0)
#define REG_PART_ID	0x86
#define REG_MANUFAC_ID	0x87
#define REG_ALS_DATA	0x88
#define REG_ALS_STATUS	0x8c
#define		REG_ALS_STATUS_DATA_STATUS	BIT(2)
#define		REG_ALS_STATUS_DATA_INVALID	BIT(7)

#define PART_ID		0xA0
#define MANUFACTURER_ID	0x05

static const char *TAG = "ltr_303als";

static esp_err_t read_register(ltr_303als_t *lis, uint8_t reg, uint8_t *val) {
	return i2c_bus_read_byte(lis->bus, ADDRESS, reg, val);
}

static esp_err_t read_registers(ltr_303als_t *lis, uint8_t start_reg, uint8_t *values, unsigned int num_values) {
	return i2c_bus_write_then_read(lis->bus, ADDRESS, &start_reg, 1, values, num_values);
}

static esp_err_t write_register(ltr_303als_t *lis, uint8_t reg, uint8_t val) {
	return i2c_bus_write_byte(lis->bus, ADDRESS, reg, val);
}

esp_err_t ltr_303als_init(ltr_303als_t *lis, i2c_bus_t *bus) {
	lis->bus = bus;

	uint8_t device_id[2];
	esp_err_t err = read_registers(lis, REG_PART_ID, device_id, ARRAY_SIZE(device_id));
	if (err) {
		ESP_LOGE(TAG, "Failed to read device id registers: %d", err);
		return err;
	}

	if (device_id[0] != PART_ID ||
	    device_id[1] != MANUFACTURER_ID) {
		ESP_LOGE(TAG, "Unsupported device id: part=0x%02x, manufacturer=%02x", device_id[0], device_id[1]);
		return ESP_ERR_NOT_SUPPORTED;
	}

	ESP_LOGI(TAG, "Found LTR-303ALS @0x%02x", ADDRESS);

	err = write_register(lis, REG_CONTR, REG_CONTR_SW_RESET);
	if (err) {
		ESP_LOGE(TAG, "Failed to perform software reset: %d", err);
		return err;
	}

	vTaskDelay(pdMS_TO_TICKS(100));

	err = write_register(lis, REG_CONTR,
			     REG_CONTR_ALS_GAIN_96X |
			     REG_CONTR_ALS_MODE_ACTIVE);
	if (err) {
		ESP_LOGE(TAG, "Failed to configure sensor to active mode: %d", err);
		return err;
	}

	vTaskDelay(pdMS_TO_TICKS(10));

	return ESP_OK;
}

esp_err_t ltr_303als_update(ltr_303als_t *lis) {
	uint8_t status;
	esp_err_t err = read_register(lis, REG_ALS_STATUS, &status);
	if (err) {
		ESP_LOGW(TAG, "Failed to read status register: %d", err);
		return err;
	}

	if (!(status & REG_ALS_STATUS_DATA_INVALID) || !!(status & REG_ALS_STATUS_DATA_STATUS)) {
		uint8_t als_data[4];
		esp_err_t err = read_registers(lis, REG_ALS_DATA, als_data, ARRAY_SIZE(als_data));
		if (err) {
			ESP_LOGE(TAG, "Failed to read data registers: %d", err);
			return err;
		}

		uint16_t white_raw = (uint16_t)als_data[0] + ((uint16_t)als_data[1] << 8);
		uint16_t ir_raw = (uint16_t)als_data[2] + ((uint16_t)als_data[3] << 8);

		lis->light_level_mlux_white = (uint32_t)white_raw * 600000UL / 65536UL;
		lis->light_level_mlux_ir = (uint32_t)ir_raw * 600000UL / 65536UL;
	}

	return ESP_OK;
}
