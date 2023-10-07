#include "lis3dh.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

#include "util.h"

#define REG_WHO_AM_I			0x0f
#define REG_CTRL_REG1			0x20
#define		CTRL_REG1_ODR_1HZ	(1 << 4)
#define		CTRL_REG1_ODR_10HZ	(2 << 4)
#define		CTRL_REG1_ODR_100HZ	(5 << 4)
#define		CTRL_REG1_ODR_400HZ	(7 << 4)
#define		CTRL_REG1_ODR_1344HZ	(9 << 4)
#define		CTRL_REG1_X_EN		(1 << 0)
#define		CTRL_REG1_Y_EN		(1 << 1)
#define		CTRL_REG1_Z_EN		(1 << 2)
#define REG_CTRL_REG2			0x21
#define		CTRL_REG2_HPCLICK	(1 << 2)
#define		CTRL_REG2_FDS		(1 << 3)
#define REG_CTRL_REG3			0x22
#define		CTRL_REG3_I1_CLICK	(1 << 7)
#define REG_CTRL_REG4			0x23
#define		CTRL_REG4_HR		(1 << 3)
#define		CTRL_REG4_FS_2G		(0 << 4)
#define		CTRL_REG4_FS_4G		(1 << 4)
#define		CTRL_REG4_FS_8G		(2 << 4)
#define		CTRL_REG4_FS_16G	(3 << 4)
#define REG_CTRL_REG5			0x24
#define 	CTRL_REG5_BOOT		(1 << 7)
#define 	CTRL_REG5_FIFO_EN	(1 << 6)
#define 	CTRL_REG5_LIR_INT1	(1 << 3)
#define REG_OUT_X_L			0x28
#define REG_OUT_Z_H			0x2d
#define REG_FIFO_CTRL_REG		0x2e
#define 	FIFO_CTRL_REG_FM_FIFO	(1 << 6)
#define 	FIFO_CTRL_REG_FM_STREAM	(2 << 6)
#define 	FIFO_CTRL_REG_FM_STREAM_TO_FIFO	(3 << 6)
#define 	FIFO_CTRL_REG_FTH(x)	(x)
#define REG_FIFO_SRC_REG		0x2f
#define		FIFO_SRC_REG_WMT	(1 << 7)
#define		FIFO_SRC_REG_OVRN_FIFO	(1 << 6)
#define		FIFO_SRC_REG_EMPTY	(1 << 5)
#define REG_INT1_SRC			0x31
#define REG_CLICK_CFG			0x38
#define		CLICK_CFG_XS		(1 << 0)
#define		CLICK_CFG_YS		(1 << 2)
#define		CLICK_CFG_ZS		(1 << 4)
#define REG_CLICK_SRC			0x39
#define		CLICK_SRC_IA		(1 << 6)
#define		CLICK_SRC_SCLICK	(1 << 4)
#define		CLICK_SRC_Z		(1 << 2)
#define		CLICK_SRC_Y		(1 << 1)
#define		CLICK_SRC_X		(1 << 0)
#define REG_CLICK_THS			0x3A
#define 	CLICK_THS_LIR_CLICK	(1 << 7)
#define REG_TIME_LIMIT			0x3B
#define REG_TIME_LATENCY		0x3C
#define REG_TIME_WINDOW			0x3D

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
	lis->click_detected = false;
	lis->fifo_full = false;

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

	err = lis_write_reg(lis, REG_CTRL_REG4, CTRL_REG4_HR | CTRL_REG4_FS_8G);
	if (err) {
		ESP_LOGW(TAG, "Failed to set resolution and range: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_CTRL_REG1, CTRL_REG1_ODR_1344HZ |
						CTRL_REG1_X_EN |
						CTRL_REG1_Y_EN |
						CTRL_REG1_Z_EN);
	if (err) {
		ESP_LOGE(TAG, "Datarate setup failed: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_CTRL_REG5, CTRL_REG5_FIFO_EN | CTRL_REG5_LIR_INT1);
	if (err) {
		ESP_LOGE(TAG, "Failed to enable FIFO: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_CTRL_REG3, CTRL_REG3_I1_CLICK);
	if (err) {
		ESP_LOGE(TAG, "Failed to route click interrupt to INT1: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_FIFO_CTRL_REG, FIFO_CTRL_REG_FM_STREAM_TO_FIFO);

	if (err) {
		ESP_LOGE(TAG, "Failed to setup FIFO mode: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_CLICK_CFG, /*CLICK_CFG_XS | CLICK_CFG_YS | */CLICK_CFG_ZS);
	if (err) {
		ESP_LOGE(TAG, "Failed to enable click detection: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_CLICK_THS, CLICK_THS_LIR_CLICK | 4);
	if (err) {
		ESP_LOGE(TAG, "Failed to enable click interrupt persistence: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_TIME_LIMIT, 10);
	if (err) {
		ESP_LOGE(TAG, "Failed to set click time limit: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_CTRL_REG2, CTRL_REG2_HPCLICK | CTRL_REG2_FDS);
	if (err) {
		ESP_LOGE(TAG, "Failed to enable high pass filter for click detection: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_TIME_LATENCY, 10);
	if (err) {
		ESP_LOGE(TAG, "Failed to setup min double click interval: %d", err);
		return err;
	}

	err = lis_write_reg(lis, REG_TIME_WINDOW, 100);
	if (err) {
		ESP_LOGE(TAG, "Failed to setup max double click interval: %d", err);
		return err;
	}

	return ESP_OK;
}

esp_err_t lis3dh_update(lis3dh_t *lis) {
	lis->fifo_full = false;
	uint8_t fifo_src;
	esp_err_t err = lis_read_reg(lis, REG_FIFO_SRC_REG, &fifo_src);
	if (err) {
		ESP_LOGW(TAG, "Failed to read fifo source register: %d", err);
		return err;
	}

//	ESP_LOGI(TAG, "Fifo level: %u", fifo_src & 0x1f);
	if (fifo_src & /*FIFO_SRC_REG_WMT*/ FIFO_SRC_REG_OVRN_FIFO && lis->click_detected) {
		unsigned int fifo_level = fifo_src & 0x1f;
		const uint8_t read_addr = REG_OUT_X_L | MULTI_BYTE_READ_FLAG;
		err = i2c_bus_write_then_read(lis->bus, lis->address, &read_addr, 1,
					      (uint8_t *)lis->fifo_buf, sizeof(lis3dh_sample_t) * fifo_level);
		if (err) {
			ESP_LOGE(TAG, "Failed to read %u samples from fifo: %d", fifo_level, err);
			return err;
		}

		uint8_t _;
		lis_read_reg(lis, REG_CLICK_SRC, &_);
		lis_read_reg(lis, REG_INT1_SRC, &_);
		lis_read_reg(lis, REG_CLICK_SRC, &_);
		lis_read_reg(lis, REG_INT1_SRC, &_);
/*
		lis_write_reg(lis, REG_CTRL_REG5, 0);
		lis_write_reg(lis, REG_CTRL_REG5, CTRL_REG5_FIFO_EN);
*/
		lis_write_reg(lis, REG_FIFO_CTRL_REG, 0);
		lis_write_reg(lis, REG_FIFO_CTRL_REG, FIFO_CTRL_REG_FM_STREAM_TO_FIFO);

		lis->click_detected = false;
		lis->fifo_full = true;
	}

	if (fifo_src & FIFO_SRC_REG_OVRN_FIFO) {
//		ESP_LOGW(TAG, "fifo overrun");
	}

	if (!lis->click_detected) {
		uint8_t click_src;
		err = lis_read_reg(lis, REG_CLICK_SRC, &click_src);
		if (err) {
			ESP_LOGW(TAG, "Failed to read click source register: %d", err);
			return err;
		}

		if (!!(click_src & (/*CLICK_SRC_X | CLICK_SRC_Y | */CLICK_SRC_Z))) {
			lis->click_detected = true;
		}
	}

	return ESP_OK;
}

bool lis3dh_has_click_been_detected(lis3dh_t *lis) {
	return lis->fifo_full;
}

static int16_t get_offset_channel(unsigned int channel, lis3dh_sample_t *samples, unsigned int num_samples) {
	int32_t accumulator = 0;
	for (unsigned i = 0; i < num_samples; i++) {
		accumulator += samples[i].xyz[channel];
	}
	return accumulator / (int32_t)num_samples;
}

static uint16_t get_click_peak_acceleration_channel(unsigned int channel, lis3dh_sample_t *samples, unsigned int num_samples) {
	uint16_t max_accel = 0;
	int16_t offset = 0;//get_offset_channel(channel, samples, num_samples);
	for (unsigned i = 0; i < num_samples; i++) {
		int16_t sample = samples[i].xyz[channel];
		uint16_t magnitude = ABS(sample - offset);
		max_accel = MAX(max_accel, magnitude);
	}

	return max_accel;
}

uint16_t lis3dh_get_peak_click_acceleration(lis3dh_t *lis) {
	uint16_t peak_accel_x = get_click_peak_acceleration_channel(0, lis->fifo_buf, ARRAY_SIZE(lis->fifo_buf));
	uint16_t peak_accel_y = get_click_peak_acceleration_channel(1, lis->fifo_buf, ARRAY_SIZE(lis->fifo_buf));
	uint16_t peak_accel_z = get_click_peak_acceleration_channel(2, lis->fifo_buf, ARRAY_SIZE(lis->fifo_buf));

	return MAX(peak_accel_x, MAX(peak_accel_y, peak_accel_z));
}

static int32_t get_click_velocity_channel(unsigned int channel, lis3dh_sample_t *samples, unsigned int num_samples) {
	int32_t velocity = 0;
	int16_t offset = 0;//get_offset_channel(channel, samples, num_samples);
	for (unsigned i = 0; i < num_samples; i++) {
		int16_t sample = samples[i].xyz[channel];
//		ESP_LOGI(TAG, "accel: %d", sample);
		int16_t acceleration = sample - offset;
		velocity += acceleration;
	}

	return velocity;
}

uint32_t lis3dh_get_click_velocity(lis3dh_t *lis) {
//	int32_t velocity_x = get_click_velocity_channel(0, lis->fifo_buf, ARRAY_SIZE(lis->fifo_buf));
//	int32_t velocity_y = get_click_velocity_channel(1, lis->fifo_buf, ARRAY_SIZE(lis->fifo_buf));
	int32_t velocity_z = get_click_velocity_channel(2, lis->fifo_buf, ARRAY_SIZE(lis->fifo_buf));

	return /*velocity_x + velocity_y + */velocity_z;
}
