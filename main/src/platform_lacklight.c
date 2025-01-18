#include "platform_lacklight.h"

#include <stdlib.h>

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_rom_gpio.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <hal/gpio_hal.h>
#include <hal/gpio_ll.h>
#include <sdkconfig.h>
#include <soc/gpio_sig_map.h>
#include <soc/io_mux_reg.h>

#include "color.h"
#include "util.h"

#define NUM_LEDS	104
#define BITS_PER_SYMBOL	4
#define SYMBOL_ZERO	0b0001
#define SYMBOL_ONE	0b0111

#define BYTES_DATA	((NUM_LEDS) * 24 * (BITS_PER_SYMBOL)) / 8
#define BYTES_RESET	(250 / 8)

static const char *TAG = "lacklight";
static const size_t dma_buf_len = ALIGN_UP(BYTES_RESET + BYTES_DATA + BYTES_RESET, 4);

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

static uint8_t *led_set_color(uint8_t *data, uint8_t r, uint8_t g, uint8_t b) {
	data = led_set_color_component(data, g);
	data = led_set_color_component(data, r);
	data = led_set_color_component(data, b);
	return data;
}

#define GLOBAL_BRIGHT(comp) ((comp) > NUM_LEDS ? (comp) >> 4 : 0)
#define LOCAL_BRIGHT(comp, i_) (GLOBAL_BRIGHT(comp) + (((i_ < (comp - (GLOBAL_BRIGHT(comp) << 4)))) ? 1 : 0))

static void leds_set_color(uint8_t *data, uint16_t r, uint16_t g, uint16_t b) {
	rgb16_t color_in = { r, g, b };

	uint16_t r_corrected = color_in.r;
	uint16_t g_corrected = color_in.g;
	uint16_t b_corrected = color_in.b;
	for (int i = 0; i < NUM_LEDS; i++) {
		uint8_t local_r = MIN(r_corrected, 255);
		uint8_t local_g = MIN(g_corrected, 255);
		uint8_t local_b = MIN(b_corrected, 255);
		data = led_set_color(data, local_r, local_g, local_b);
		r_corrected -= local_r;
		g_corrected -= local_g;
		b_corrected -= local_b;
	}
}

static void pre_schedule(platform_t *platform) {
	platform_lacklight_t *plat = container_of(platform, platform_lacklight_t, base);

	spi_transaction_t *xfer_;
	if (plat->transaction_pending) {
		spi_device_get_trans_result(plat->dev, &xfer_, portMAX_DELAY);
		plat->transaction_pending = false;
	}
}

static void set_rdb_led_color(platform_t *platform, uint16_t r, uint16_t g, uint16_t b) {
	platform_lacklight_t *plat = container_of(platform, platform_lacklight_t, base);

	spi_transaction_t *xfer_;
	if (plat->transaction_pending) {
		spi_device_get_trans_result(plat->dev, &xfer_, portMAX_DELAY);
		plat->transaction_pending = false;
	}

	leds_set_color(plat->led_data + BYTES_RESET, r, g, b);

	plat->xfer.length = dma_buf_len * 8;
	plat->xfer.rxlength = 0;
	plat->xfer.tx_buffer = plat->led_data;
	plat->xfer.rx_buffer = NULL;
	ESP_ERROR_CHECK(spi_device_queue_trans(plat->dev, &plat->xfer, 0));
	plat->transaction_pending = true;
}

static const platform_ops_t lacklight_ops = {
	.pre_schedule = pre_schedule,
	.set_rgb_led_color = set_rdb_led_color
};

esp_err_t platform_lacklight_probe(platform_t **ret) {
	platform_lacklight_t *katze = calloc(1, sizeof(platform_lacklight_t));
	if (!katze) {
		return ESP_ERR_NO_MEM;
	}

	spi_bus_config_t spi_bus_cfg = {
		.mosi_io_num = 4,
		.miso_io_num = -1,
		.sclk_io_num = -1,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 4092,
		.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
		.intr_flags = ESP_INTR_FLAG_IRAM
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
	ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &katze->dev));

	ESP_LOGI(TAG, "Allocating %lu bytes of DMA memory", (unsigned long)dma_buf_len);
	katze->led_data = heap_caps_malloc(dma_buf_len, MALLOC_CAP_DMA);
	ESP_ERROR_CHECK(!katze->led_data);
	memset(katze->led_data, 0, dma_buf_len);

	leds_set_color(katze->led_data + BYTES_RESET, 0x00, 0x00, 0x00);

	katze->xfer.length = dma_buf_len * 8;
	katze->xfer.rxlength = 0;
	katze->xfer.tx_buffer = katze->led_data;
	katze->xfer.rx_buffer = NULL;
	ESP_ERROR_CHECK(spi_device_transmit(katze->dev, &katze->xfer));

	platform_init(&katze->base, &lacklight_ops);
	*ret = &katze->base;

	return 0;
}
