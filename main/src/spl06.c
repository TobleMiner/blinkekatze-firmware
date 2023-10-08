#include "spl06.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

#define REG_PRS_B2		0x00

#define REG_PRS_CFG		0x06
#define		PM_PRC_4	0x02
#define		PM_RATE_128	(7 << 4)

#define REG_MEAS_CFG		0x08
#define		COEF_RDY	(1 << 7)
#define		SENSOR_RDY	(1 << 6)
#define		PRS_RDY		(1 << 4)
#define		MEAS_CTRL_PRS	(1 << 0)
#define		MEAS_CTRL_PRS_CONT (5 << 0)

#define REG_RESET_FIFO_FLUSH	0x0c
#define		SOFT_RST	0x09

#define REG_PRODUCT_ID		0x0D

static const char *TAG = "spl06";

static esp_err_t write_reg(spl06_t *spl, uint8_t reg, uint8_t val) {
	spi_transaction_t xfer = {
		.flags = SPI_TRANS_USE_TXDATA,
		.cmd = reg,
		.length = 8,
		.rxlength = 0,
		.tx_data = { val }
	};
	return spi_device_transmit(spl->spi, &xfer);
}

static esp_err_t read_reg(spl06_t *spl, uint8_t reg, uint8_t *val) {
	spi_transaction_t xfer = {
		.flags = SPI_TRANS_USE_TXDATA,
		.cmd = reg | 0x80,
		.length = 8,
		.rxlength = 8,
		.tx_data = { 0xff },
		.rx_buffer = val
	};
	return spi_device_transmit(spl->spi, &xfer);
}

static esp_err_t read_24bit_signed(spl06_t *spl, uint8_t reg, int32_t *val) {
	spi_transaction_t xfer = {
		.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
		.cmd = reg | 0x80,
		.length = 24,
		.rxlength = 24,
		.tx_data = { 0xff, 0xff, 0xff }
	};
	esp_err_t err = spi_device_transmit(spl->spi, &xfer);
	if (err) {
		return err;
	}
/*
	int32_t value = (int32_t)xfer.rx_data[0] << 24 |
			(int32_t)xfer.rx_data[1] << 16 |
			(int32_t)xfer.rx_data[2] << 8;
	if ((uint32_t)value & (1 << 31)) {
		value |= 0xff;
	}
	value /= 256;
*/
	int32_t value = (int32_t)xfer.rx_data[0] << 16 |
			(int32_t)xfer.rx_data[1] << 8 |
			(int32_t)xfer.rx_data[2] << 0;
	*val = value;
	return ESP_OK;
}

esp_err_t spl06_init(spl06_t *spl, spi_host_device_t spi_host, int gpio_cs) {
	spi_device_interface_config_t dev_cfg = {
		.command_bits = 8,
		.address_bits = 0,
		.dummy_bits = 0,
		.mode = 3,
		.duty_cycle_pos = 0,
		.cs_ena_pretrans = 0,
		.cs_ena_posttrans = 0,
		.clock_speed_hz = 3000000,
		.input_delay_ns = 0,
		.spics_io_num = gpio_cs,
		.queue_size = 1,
	};
	esp_err_t err = spi_bus_add_device(spi_host, &dev_cfg, &spl->spi);
	if (err) {
		ESP_LOGE(TAG, "Failed to register SPI device: %d", err);
		return err;
	}

	uint8_t prod_rev;
	err = read_reg(spl, REG_PRODUCT_ID, &prod_rev);
	if (err) {
		ESP_LOGE(TAG, "Failed to read product id: %d", err);
		return err;
	}
	ESP_LOGI(TAG, "Found SPL06-001 prod: %u, rev: %u", prod_rev >> 4, prod_rev & 0x0f);

	err = write_reg(spl, REG_RESET_FIFO_FLUSH, SOFT_RST);
	if (err) {
		ESP_LOGE(TAG, "Failed to reset sensor: %d", err);
		return err;
	}

	vTaskDelay(pdMS_TO_TICKS(100));
	uint8_t status;
	err = read_reg(spl, REG_MEAS_CFG, &status);
	if (err) {
		ESP_LOGE(TAG, "Failed to read sensor status: %d", err);
		return err;
	}

	if ((status & (SENSOR_RDY | COEF_RDY)) != (SENSOR_RDY | COEF_RDY)) {
		ESP_LOGE(TAG, "Sensor did not become ready");
		return ESP_ERR_TIMEOUT;
	}

	err = write_reg(spl, REG_PRS_CFG, PM_PRC_4 | PM_RATE_128);
	if (err) {
		ESP_LOGE(TAG, "Failed to setup pressure measurement settings: %d", err);
		return err;
	}

	err = write_reg(spl, REG_MEAS_CFG, MEAS_CTRL_PRS_CONT);
	if (err) {
		ESP_LOGE(TAG, "Failed to start pressure measurement: %d", err);
		return err;
	}
	spl->pressure = 0;
	return ESP_OK;
}

esp_err_t spl06_update(spl06_t *spl) {
	uint8_t status;
	esp_err_t err = read_reg(spl, REG_MEAS_CFG, &status);
	if (err) {
		ESP_LOGE(TAG, "Failed to read sensor status: %d", err);
		return err;
	}

	if (status & PRS_RDY) {
		int32_t pressure;
		err = read_24bit_signed(spl, REG_PRS_B2, &pressure);
		if (err) {
			ESP_LOGE(TAG, "Failed to read pressure: %d", err);
			return err;
		}
		spl->pressure = pressure;
	}

	return ESP_OK;
}

int32_t spl06_get_pressure(spl06_t *spl) {
	return spl->pressure;
}
