#include "lis3dh.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

#define REG_WHO_AM_I			0x0f
#define REG_CTRL_REG1			0x20
#define		CTRL_REG1_ODR_1HZ	(1 << 4)
#define		CTRL_REG1_ODR_10HZ	(2 << 4)
#define		CTRL_REG1_ODR_100HZ	(5 << 4)
#define		CTRL_REG1_ODR_400HZ	(7 << 4)
#define		CTRL_REG1_X_EN		(1 << 0)
#define		CTRL_REG1_Y_EN		(1 << 1)
#define		CTRL_REG1_Z_EN		(1 << 2)
#define REG_CTRL_REG5			0x24
#define 	CTRL_REG5_BOOT		(1 << 7)
#define 	CTRL_REG5_FIFO_EN	(1 << 6)
#define REG_OUT_X_L			0x28
#define REG_OUT_Z_H			0x2d
#define REG_FIFO_CTRL_REG		0x2e
#define 	FIFO_CTRL_REG_FM_FIFO	(1 << 6)
#define 	FIFO_CTRL_REG_FM_STREAM	(1 << 7)
#define 	FIFO_CTRL_REG_FTH(x)	(x)
#define REG_FIFO_SRC_REG		0x2f
#define		FIFO_SRC_REG_WMT	(1 << 7)
#define		FIFO_SRC_REG_OVRN_FIFO	(1 << 6)
#define		FIFO_SRC_REG_EMPTY	(1 << 5)

#define MULTI_BYTE_READ_FLAG		0x80

static const char *TAG = "lis3dh";

static esp_err_t lis_read_reg(lis3dh_t *lis, uint8_t reg, uint8_t *value) {
	return i2c_bus_read_byte(lis->bus, lis->address, reg, value);
}

static esp_err_t lis_write_reg(lis3dh_t *lis, uint8_t reg, uint8_t value) {
	return i2c_bus_write_byte(lis->bus, lis->address, reg, value);
}

esp_err_t lis3dh_init(lis3dh_t *lis, i2c_bus_t *bus, uint8_t address) {
	lis->bus = bus;
	lis->address = address;

	uint8_t device_id;
	esp_err_t err = lis_read_reg(lis, REG_WHO_AM_I, &device_id);
	if (err) {
		ESP_LOGE(TAG, "Failed to communicate with accelerometer: %d", err);
		return err;
	}
	if (device_id != 0x33) {
		ESP_LOGE(TAG, "Invalid device id, expected 0x33 but got %02x", device_id);
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "Found LIS3DH @0x%02x", address);

	err = lis_write_reg(lis, REG_CTRL_REG1, 0);
	if (err) {
		ESP_LOGW(TAG, "Failed to disable data acquisition: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_CTRL_REG5, 0);
	if (err) {
		ESP_LOGW(TAG, "Failed to disable fifo: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_FIFO_CTRL_REG, 0);
	if (err) {
		ESP_LOGW(TAG, "Failed to reset fifo to bypass mode: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_CTRL_REG1, CTRL_REG1_ODR_400HZ |
						CTRL_REG1_X_EN |
						CTRL_REG1_Y_EN |
						CTRL_REG1_Z_EN);
	if (err) {
		ESP_LOGE(TAG, "Datarate setup failed: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_CTRL_REG5, CTRL_REG5_FIFO_EN);
	if (err) {
		ESP_LOGE(TAG, "Failed to enable FIFO: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_FIFO_CTRL_REG, FIFO_CTRL_REG_FM_STREAM |
						    FIFO_CTRL_REG_FTH(16));
	if (err) {
		ESP_LOGE(TAG, "Failed to setup FIFO threshold: %d", err);
		return err;
	}

	return ESP_OK;
}

esp_err_t lis3dh_update(lis3dh_t *lis) {
	uint8_t fifo_src;
	esp_err_t err = lis_read_reg(lis, REG_FIFO_SRC_REG, &fifo_src);
	if (err) {
		ESP_LOGW(TAG, "Failed to read fifo source register: %d", err);
		return err;
	}

	if (fifo_src & FIFO_SRC_REG_WMT) {
		unsigned int fifo_level = fifo_src & 0x1f;
		const uint8_t read_addr = REG_OUT_X_L | MULTI_BYTE_READ_FLAG;
		err = i2c_bus_write_then_read(lis->bus, lis->address, &read_addr, 1,
					      (uint8_t *)lis->fifo_buf, sizeof(lis3dh_sample_t) * fifo_level);
		if (err) {
			ESP_LOGE(TAG, "Failed to read %u samples from fifo: %d", fifo_level, err);
			return err;
		}
		ESP_LOGI(TAG, "Read %u samples", fifo_src & 0x1f);
	}

	if (fifo_src & FIFO_SRC_REG_OVRN_FIFO) {
		ESP_LOGW(TAG, "fifo overrun");
	}

	return ESP_OK;
}
