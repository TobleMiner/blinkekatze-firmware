#include <stdlib.h>
#include <string.h>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>

#include "i2c_bus.h"
#include "util.h"

#define I2C_UNSTICK_BITS 32

static const char *TAG = "I2C_BUS";

static esp_err_t i2c_bus_init_(i2c_bus_t* bus) {
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = bus->gpio_sda,
		.scl_io_num = bus->gpio_scl,
		.master.clk_speed = bus->speed_hz,
	};

	esp_err_t err = i2c_param_config(bus->i2c_port, &i2c_config);
	if(err) {
		return err;
	}
	return i2c_driver_install(bus->i2c_port, I2C_MODE_MASTER, 0, 0, 0);
}

static esp_err_t i2c_bus_deinit_(i2c_bus_t* bus) {
	return i2c_driver_delete(bus->i2c_port);
}

esp_err_t i2c_bus_deinit(i2c_bus_t* bus) {
	return i2c_bus_deinit_(bus);
}

esp_err_t i2c_bus_init(i2c_bus_t* bus, i2c_port_t i2c_port, unsigned int gpio_sda, unsigned int gpio_scl, uint32_t speed_hz) {
	memset(bus, 0, sizeof(*bus));
	bus->i2c_port = i2c_port;
	bus->gpio_sda = gpio_sda;
	bus->gpio_scl = gpio_scl;
	bus->speed_hz = speed_hz;

	esp_err_t err = i2c_bus_init_(bus);

	bus->lock = xSemaphoreCreateMutexStatic(&bus->lock_buffer);
	return err;
}

static void i2c_unstick_bus(i2c_bus_t* bus) {
	ESP_ERROR_CHECK(i2c_bus_deinit_(bus));

	ESP_ERROR_CHECK(gpio_reset_pin(bus->gpio_sda));
	ESP_ERROR_CHECK(gpio_set_direction(bus->gpio_sda, GPIO_MODE_INPUT));
	ESP_ERROR_CHECK(gpio_reset_pin(bus->gpio_scl));
	ESP_ERROR_CHECK(gpio_set_direction(bus->gpio_scl, GPIO_MODE_OUTPUT));
	for (int i = 0; i < I2C_UNSTICK_BITS; i++) {
		ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 0));
		ets_delay_us(DIV_ROUND_UP(1000000UL, bus->speed_hz));
		ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 1));
		ets_delay_us(DIV_ROUND_UP(1000000UL, bus->speed_hz));
	}

	ESP_ERROR_CHECK(i2c_bus_init_(bus));
}

static esp_err_t i2c_bus_cmd_begin_unlocked(i2c_bus_t* bus, i2c_cmd_handle_t handle, TickType_t timeout) {
	esp_err_t err = i2c_master_cmd_begin(bus->i2c_port, handle, timeout);

	if (err == ESP_ERR_TIMEOUT) {
		ESP_LOGE(TAG, "I2C bus timeout, trying to unstick bus");
		i2c_unstick_bus(bus);
	}
	return err;
}

esp_err_t i2c_bus_cmd_begin(i2c_bus_t* bus, i2c_cmd_handle_t handle, TickType_t timeout) {
	xSemaphoreTake(bus->lock, portMAX_DELAY);
	esp_err_t err = i2c_bus_cmd_begin_unlocked(bus, handle, timeout);
	xSemaphoreGive(bus->lock);

	return err;
}

esp_err_t i2c_bus_write_then_read(i2c_bus_t *bus, uint8_t address,
					 const uint8_t *data_write, unsigned int write_len,
					 uint8_t *data_read, unsigned int read_len) {
	xSemaphoreTake(bus->lock, portMAX_DELAY);
	esp_err_t err;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(bus->cmd_buf, sizeof(bus->cmd_buf));
	if(!cmd) {
		err = ESP_ERR_NO_MEM;
		goto fail;
	}
	if((err = i2c_master_start(cmd))) {
		goto fail_link;
	}
	if((err = i2c_master_write_byte(cmd, address << 1, 1))) {
		goto fail_link;
	}
	if((err = i2c_master_write(cmd, data_write, write_len, true))) {
		goto fail_link;
	}
	if (read_len) {
		if((err = i2c_master_start(cmd))) {
			goto fail_link;
		}
		if((err = i2c_master_write_byte(cmd, (address << 1) | 1, 1))) {
			goto fail_link;
		}
		if((err = i2c_master_read(cmd, data_read, read_len, I2C_MASTER_LAST_NACK))) {
			goto fail_link;
		}
	}
	if((err = i2c_master_stop(cmd))) {
		goto fail_link;
	}
	err = i2c_bus_cmd_begin_unlocked(bus, cmd, pdMS_TO_TICKS(100));
fail_link:
	i2c_cmd_link_delete_static(cmd);
fail:
	xSemaphoreGive(bus->lock);
	return err;
}

esp_err_t i2c_bus_read_byte(i2c_bus_t *bus, uint8_t address, uint8_t reg, uint8_t *res) {
	return i2c_bus_write_then_read(bus, address, &reg, 1, res, 1);
}

esp_err_t i2c_bus_write_byte(i2c_bus_t *bus, uint8_t address, uint8_t reg, uint8_t val) {
	uint8_t data[] = { reg, val };
	return i2c_bus_write_then_read(bus, address, data, 2, NULL, 0);
}

esp_err_t i2c_bus_scan(i2c_bus_t* bus, i2c_address_set_t addr) {
	esp_err_t err = ESP_OK;
	size_t link_buf_size = I2C_LINK_RECOMMENDED_SIZE(1);
	void *link_buf = malloc(link_buf_size);
	for (uint8_t i = 0; i < 128; i++) {
		I2C_ADDRESS_SET_CLEAR(addr, i);
		i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(link_buf, link_buf_size);
		if(!cmd) {
			err = ESP_ERR_NO_MEM;
			continue;
		}
		if((err = i2c_master_start(cmd))) {
			goto fail_link;
		}
		if((err = i2c_master_write_byte(cmd, (i << 1), true))) {
			goto fail_link;
		}
		if((err = i2c_master_stop(cmd))) {
			goto fail_link;
		}
		esp_err_t nacked = i2c_bus_cmd_begin(bus, cmd, pdMS_TO_TICKS(100));
		if(!nacked) {
			I2C_ADDRESS_SET_SET(addr, i);
		}
		i2c_cmd_link_delete_static(cmd);
		continue;
fail_link:
		i2c_cmd_link_delete_static(cmd);
		break;
	}
	free(link_buf);
	return err;
}

void i2c_detect(i2c_bus_t* bus) {
	ESP_LOGI(TAG, "Scanning i2c bus %d for devices", bus->i2c_port);
	I2C_ADDRESS_SET(devices);
	esp_err_t err = i2c_bus_scan(bus, devices);
	if (err) {
		ESP_LOGE(TAG, "Failed to scan bus %d: %d", bus->i2c_port, err);
	} else {
		ESP_LOGI(TAG, "=== Detected devices ===");
		for (uint8_t i = 0; i < 128; i++) {
			if (I2C_ADDRESS_SET_CONTAINS(devices, i)) {
				ESP_LOGI(TAG, "  0x%02x", i);
			}
		}
		ESP_LOGI(TAG, "========================");
	}
}

void i2c_bus_enter_soft_exclusive(i2c_bus_t *bus) {
	xSemaphoreTake(bus->lock, portMAX_DELAY);
	ESP_ERROR_CHECK(i2c_bus_deinit_(bus));
	ESP_ERROR_CHECK(gpio_reset_pin(bus->gpio_sda));
	ESP_ERROR_CHECK(gpio_set_direction(bus->gpio_sda, GPIO_MODE_INPUT));
	ESP_ERROR_CHECK(gpio_reset_pin(bus->gpio_scl));
	ESP_ERROR_CHECK(gpio_set_direction(bus->gpio_scl, GPIO_MODE_INPUT));
}

void i2c_bus_leave_soft_exclusive(i2c_bus_t *bus) {
	ESP_ERROR_CHECK(i2c_bus_init_(bus));
	xSemaphoreGive(bus->lock);
}

static esp_err_t i2c_wait_bus_idle(i2c_bus_t *bus, unsigned int timeout_ms) {
	int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000LL;
	int level_sda, level_scl;
	do {
		level_sda = gpio_get_level(bus->gpio_sda);
		level_scl = gpio_get_level(bus->gpio_scl);
		if (!level_sda || !level_scl) {
			int64_t now = esp_timer_get_time();
			if (now > deadline) {
				return ESP_ERR_TIMEOUT;
			}
			vTaskDelay(1);
		}
	} while (!level_sda || !level_scl);

	return ESP_OK;
}

static esp_err_t i2c_wait_clock_idle(i2c_bus_t *bus, unsigned int timeout_ms) {
	int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000LL;
	int level_scl;
	do {
		level_scl = gpio_get_level(bus->gpio_scl);
		if (!level_scl) {
			int64_t now = esp_timer_get_time();
			if (now > deadline) {
				return ESP_ERR_TIMEOUT;
			}
			vTaskDelay(1);
		}
	} while (!level_scl);

	return ESP_OK;
}

static esp_err_t i2c_bus_clock_out_byte(i2c_bus_t *bus, uint8_t datum, unsigned int clock_stretch_timeout_ms) {
	/* Clock out bits */
	for (int i = 7; i >= 0; i--) {
		unsigned int bit = datum & (1 << i);
		ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 0));
		ets_delay_us(DIV_ROUND_UP(1000000UL / 4UL, bus->speed_hz));
		ESP_ERROR_CHECK(gpio_set_level(bus->gpio_sda, bit ? 1 : 0));
		ets_delay_us(DIV_ROUND_UP(1000000UL / 4UL, bus->speed_hz));
		ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 1));
		ets_delay_us(DIV_ROUND_UP(1000000UL / 2UL, bus->speed_hz));
	}

	/* Release SDA for ack */
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 0));
	ets_delay_us(DIV_ROUND_UP(1000000UL / 4UL, bus->speed_hz));
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_sda, 1));
	ets_delay_us(DIV_ROUND_UP(1000000UL / 4UL, bus->speed_hz));
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 1));

	/* Wait for clock stretching */
	esp_err_t err = i2c_wait_clock_idle(bus, clock_stretch_timeout_ms);

	/* Return SDA to idle */
	ets_delay_us(DIV_ROUND_UP(1000000UL / 2UL, bus->speed_hz));
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 0));
	ets_delay_us(DIV_ROUND_UP(1000000UL / 4UL, bus->speed_hz));
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_sda, 1));
	return err;
}

static void i2c_bus_clock_in_byte(i2c_bus_t *bus, uint8_t *datum, bool ack) {
	/* Clock out bits */
	uint8_t byt = 0;
	for (int i = 7; i >= 0; i--) {
		ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 0));
		ets_delay_us(DIV_ROUND_UP(1000000UL / 4UL, bus->speed_hz));
		ESP_ERROR_CHECK(gpio_set_level(bus->gpio_sda, 1));
		ets_delay_us(DIV_ROUND_UP(1000000UL / 4UL, bus->speed_hz));
		ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 1));
		ets_delay_us(DIV_ROUND_UP(1000000UL / 4UL, bus->speed_hz));
		int bit = gpio_get_level(bus->gpio_sda);
		byt |= (bit ? 1 : 0) << i;
		ets_delay_us(DIV_ROUND_UP(1000000UL / 4UL, bus->speed_hz));
	}
	*datum = byt;

	/* Send ack / nak */
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 0));
	ets_delay_us(DIV_ROUND_UP(1000000UL / 4UL, bus->speed_hz));
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_sda, ack ? 0 : 1));
	ets_delay_us(DIV_ROUND_UP(1000000UL / 4UL, bus->speed_hz));
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 1));

	/* SDA back to idle */
	ets_delay_us(DIV_ROUND_UP(1000000UL / 2UL, bus->speed_hz));
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 0));
	ets_delay_us(DIV_ROUND_UP(1000000UL / 4UL, bus->speed_hz));
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_sda, 1));
}

esp_err_t i2c_bus_soft_write_then_read(i2c_bus_t *bus, uint8_t address,
                                       const uint8_t *data_write, unsigned int write_len,
                                       uint8_t *data_read, unsigned int read_len,
				       unsigned int idle_timeout_ms) {

	/* Wait for bus to become idle */
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_sda, 1));
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 1));
	ESP_ERROR_CHECK(gpio_set_direction(bus->gpio_sda, GPIO_MODE_INPUT_OUTPUT_OD));
	ESP_ERROR_CHECK(gpio_set_direction(bus->gpio_scl, GPIO_MODE_INPUT_OUTPUT_OD));
	esp_err_t err = i2c_wait_bus_idle(bus, idle_timeout_ms);
	if (err) {
		ESP_LOGE(TAG, "Bus did not become idle");
		return err;
	}

	/* Send start condition */
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_sda, 0));
	ets_delay_us(DIV_ROUND_UP(1000000UL, bus->speed_hz));

	/* Send address (W) */
	err = i2c_bus_clock_out_byte(bus, address << 1, idle_timeout_ms);
	if (err) {
		ESP_LOGE(TAG, "Clock stretch timeout after address (W)");
		return err;
	}

	/* Send data */
	for (unsigned int i = 0; i < write_len; i++) {
		err = i2c_bus_clock_out_byte(bus, data_write[i], idle_timeout_ms);
		if (err) {
			ESP_LOGE(TAG, "Clock stretch timeout on byte %u", i);
			return err;
		}
	}

	if (read_len) {
		/* Send repeated start */
		ESP_ERROR_CHECK(gpio_set_level(bus->gpio_sda, 1));
		ets_delay_us(DIV_ROUND_UP(1000000UL / 2UL, bus->speed_hz));
		ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 1));
		ets_delay_us(DIV_ROUND_UP(1000000UL / 2UL, bus->speed_hz));
		ESP_ERROR_CHECK(gpio_set_level(bus->gpio_sda, 0));
		ets_delay_us(DIV_ROUND_UP(1000000UL / 2UL, bus->speed_hz));

		/* Send address (R) */
		err = i2c_bus_clock_out_byte(bus, (address << 1) | 1, idle_timeout_ms);
		if (err) {
			ESP_LOGE(TAG, "Clock stretch timeout after address (R)");
			return err;
		}

		/* Receive data */
		for (unsigned int i = 0; i < write_len; i++) {
			i2c_bus_clock_in_byte(bus, &data_read[i], i == write_len - 1);
		}
	}

	/* Send stop condition */
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_sda, 0));
	ets_delay_us(DIV_ROUND_UP(1000000UL / 2UL, bus->speed_hz));
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_scl, 1));
	ets_delay_us(DIV_ROUND_UP(1000000UL / 2UL, bus->speed_hz));
	ESP_ERROR_CHECK(gpio_set_level(bus->gpio_sda, 1));
	ets_delay_us(DIV_ROUND_UP(1000000UL / 2UL, bus->speed_hz));
	return ESP_OK;
}
