#pragma once

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <esp_err.h>
#include <driver/i2c.h>

typedef struct i2c_bus {
	i2c_port_t i2c_port;
	unsigned int gpio_sda;
	unsigned int gpio_scl;
	uint32_t speed_hz;
	SemaphoreHandle_t lock;
	StaticSemaphore_t lock_buffer;
	uint8_t cmd_buf[I2C_LINK_RECOMMENDED_SIZE(3)];
} i2c_bus_t;

#define I2C_ADDRESS_SET(name) uint8_t name[16] = { 0 }

#define I2C_ADDRESS_SET_CONTAINS(set, id) \
	(!!((set)[(id) / 8] & (1 << (id % 8))))

#define I2C_ADDRESS_SET_SET(set, id) \
	(set)[(id) / 8] |= (1 << (id % 8))

#define I2C_ADDRESS_SET_CLEAR(set, id) \
	(set)[(id) / 8] &= ~(1 << (id % 8))

typedef uint8_t* i2c_address_set_t;

esp_err_t i2c_bus_init(i2c_bus_t* bus, i2c_port_t i2c_port, unsigned int gpio_sda, unsigned int gpio_scl, uint32_t speed_hz);
esp_err_t i2c_bus_deinit(i2c_bus_t* bus);
esp_err_t i2c_bus_cmd_begin(i2c_bus_t* bus, i2c_cmd_handle_t handle, TickType_t timeout);
esp_err_t i2c_bus_write_then_read(i2c_bus_t *bus, uint8_t address,
					 const uint8_t *data_write, unsigned int write_len,
					 uint8_t *data_read, unsigned int read_len);
esp_err_t i2c_bus_read_byte(i2c_bus_t *bus, uint8_t address, uint8_t reg, uint8_t *res);
esp_err_t i2c_bus_write_byte(i2c_bus_t *bus, uint8_t address, uint8_t reg, uint8_t val);

esp_err_t i2c_bus_scan(i2c_bus_t* bus, i2c_address_set_t addr);
void i2c_detect(i2c_bus_t* i2c_bus);

static inline esp_err_t i2c_bus_write(i2c_bus_t *bus, uint8_t address,
				      const uint8_t *data_write, unsigned int write_len) {
	return i2c_bus_write_then_read(bus, address, data_write, write_len, NULL, 0);
}
