#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "bq24295.h"
#include "fast_hsv2rgb.h"
#include "i2c_bus.h"
#include "util.h"

static const char *TAG = "main";

#define GPIO_LED1	20
#define GPIO_LED2	21
#define GPIO_POWER_ON	10

#define NUM_LEDS	16
#define BITS_PER_SYMBOL	4
#define SYMBOL_ZERO	0b0001
#define SYMBOL_ONE	0b0111

#define BYTES_DATA	((NUM_LEDS) * 24 * (BITS_PER_SYMBOL)) / 8
#define BYTES_RESET	(250 / 8)

static uint8_t *led_set_color_component(uint8_t *data, uint8_t val) {
	for (int i = 0; i < 8; i++) {
		bool bit = !!(val & (1 << (7 - i)));
		if (i % 2 == 0) {
			*data &= ~0xf;
			*data |= bit ? SYMBOL_ONE : SYMBOL_ZERO;
		} else {
			*data &= ~0xf0;
			*data |= (bit ? SYMBOL_ONE : SYMBOL_ZERO) << 4;
			data++;
		}
	}
	return data;
}

static uint8_t *led_set_color(uint8_t *data, uint32_t rgb_) {
	data = led_set_color_component(data, (rgb_ & 0x00ff00) >> 8);
	data = led_set_color_component(data, (rgb_ & 0x0000ff) >> 0);
	data = led_set_color_component(data, (rgb_ & 0xff0000) >> 16);
	return data;
}

static void leds_set_color(uint8_t *data, uint32_t rgb_) {
	for (int i = 0; i < NUM_LEDS; i++) {
		data = led_set_color(data, rgb_);
	}
}

void app_main(void) {
	gpio_reset_pin(GPIO_LED1);
	gpio_reset_pin(GPIO_LED2);
	gpio_reset_pin(GPIO_POWER_ON);
	gpio_set_direction(GPIO_LED1, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_LED2, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_POWER_ON, GPIO_MODE_INPUT);
	gpio_set_pull_mode(GPIO_POWER_ON, GPIO_PULLDOWN_ONLY);

	vTaskDelay(pdMS_TO_TICKS(2000));

	i2c_bus_t i2c_bus;
	i2c_bus_init(&i2c_bus, I2C_NUM_0, 0, 2, 100000);
	i2c_detect(&i2c_bus);

	bq24295_t charger;
	// Reset charger to default settings
	bq24295_init(&charger, &i2c_bus);
	ESP_ERROR_CHECK(bq24295_reset(&charger));
	vTaskDelay(pdMS_TO_TICKS(10));

	// Setup chargers settings
	// Min system voltage 3.0V
	ESP_ERROR_CHECK(bq24295_set_min_system_voltage(&charger, 3000));
	// Boost voltage 4.55V
	ESP_ERROR_CHECK(bq24295_set_boost_voltage(&charger, 4550));

	spi_bus_config_t spi_bus_cfg = {
		.mosi_io_num = 3,
		.miso_io_num = -1,
		.sclk_io_num = -1,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 4092,
		.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
		.intr_flags = 0
	};
	ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));
	spi_device_interface_config_t dev_cfg = {
		.command_bits = 0,
		.address_bits = 0,
		.dummy_bits = 0,
		.mode = 0,
		.duty_cycle_pos = 0,
		.cs_ena_pretrans = 0,
		.cs_ena_posttrans = 0,
		.clock_speed_hz = 3000000,
		.input_delay_ns = 0,
		.spics_io_num = -1,
		.flags = SPI_DEVICE_BIT_LSBFIRST,
		.queue_size = 1,
	};
	spi_device_handle_t dev;
	ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &dev));

	size_t dma_buf_len = ALIGN_UP(BYTES_RESET + BYTES_DATA + BYTES_RESET, 4);
	ESP_LOGI(TAG, "Allocating %zu bytes of DMA memory", dma_buf_len);
	uint8_t *led_data = heap_caps_malloc(dma_buf_len, MALLOC_CAP_DMA);
	ESP_ERROR_CHECK(!led_data);
	memset(led_data, 0, dma_buf_len);

	bool level = true;
	uint8_t bright = 0;
	unsigned int offset = 0;
	bool shutdown = false;
	while (1) {
		ESP_LOGI(TAG, "Loop!");
		gpio_set_level(GPIO_LED1, level);
		gpio_set_level(GPIO_LED2, !level);
		level = !level;

		if (!gpio_get_level(GPIO_POWER_ON)) {
			if (shutdown) {
				// Disable watchdog and other timers
				ESP_ERROR_CHECK(bq24295_set_watchdog_timeout(&charger, BQ24295_WATCHDOG_TIMEOUT_DISABLED));
				// Disable BATFET
				ESP_ERROR_CHECK(bq24295_set_shutdown(&charger, true));
			}
			ESP_LOGI(TAG, "Shutdown requested");
			shutdown = true;
		} else {
			shutdown = false;
		}

		uint8_t r, g, b;
		fast_hsv2rgb_32bit(offset % HSV_HUE_STEPS, HSV_SAT_MAX, HSV_VAL_MAX, &r, &g, &b);
//		leds_set_color(led_data + BYTES_RESET, (uint32_t)b << 16 | (uint32_t)g << 8 | r);

		leds_set_color(led_data + BYTES_RESET, level ? 0xffffff : 0);

		spi_transaction_t xfer = {
			.length = dma_buf_len * 8,
			.rxlength = 0,
			.tx_buffer = led_data,
			.rx_buffer = NULL
		};
		bright += 10;
		ESP_ERROR_CHECK(spi_device_transmit(dev, &xfer));

		// Charger watchdog reset
		ESP_ERROR_CHECK(bq24295_watchdog_reset(&charger));
		ESP_ERROR_CHECK(i2c_bus_write_byte(&i2c_bus, 0x6b, 0x01, 0x30 | (1 << 6)));

		offset++;
		offset %= HSV_HUE_STEPS;
		vTaskDelay(pdMS_TO_TICKS(3000));
	}
}
