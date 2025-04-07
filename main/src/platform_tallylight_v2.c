#include "platform_tallylight_v2.h"

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
#include "embedded_files.h"
#include "util.h"

#define REV_ID_GPIO	4
#define MAC_CS_GPIO	2
#define MAC_SI_GPIO	8
#define MAC_RST_GPIO	10

#define NUM_LEDS	10
#define BITS_PER_SYMBOL	4
#define SYMBOL_ZERO	0b0001
#define SYMBOL_ONE	0b0011

#define BYTES_DATA	((NUM_LEDS) * 24 * (BITS_PER_SYMBOL)) / 8
#define BYTES_RESET	(250 / 8)

static const char *TAG = "TallyLight v2";
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

static const rgb16_t *colorcal_table = (const rgb16_t *)EMBEDDED_FILE_PTR(colorcal_16x16x16_9bit_bin);

#define COLOR_TABLE_SIZE	16UL
#define LOOKUP_DIV		((1 << 16) / COLOR_TABLE_SIZE)

static void lookup_color(const rgb16_t *in, rgb16_t *out) {
	uint32_t idx = ((in->g * COLOR_TABLE_SIZE) + in->b) * COLOR_TABLE_SIZE + in->r;
	*out = colorcal_table[idx];
}

static void apply_color_correction_per_channel(const rgb16_t *in, rgb16_t *out) {
	rgb16_t color_min_lookup = { in->r / LOOKUP_DIV, in->g / LOOKUP_DIV, in->b / LOOKUP_DIV };
	rgb16_t color_ref_lookup = {
		MIN(in->r / LOOKUP_DIV + DIV_ROUND(in->r % LOOKUP_DIV, LOOKUP_DIV), COLOR_TABLE_SIZE - 1),
		MIN(in->g / LOOKUP_DIV + DIV_ROUND(in->g % LOOKUP_DIV, LOOKUP_DIV), COLOR_TABLE_SIZE - 1),
		MIN(in->b / LOOKUP_DIV + DIV_ROUND(in->b % LOOKUP_DIV, LOOKUP_DIV), COLOR_TABLE_SIZE - 1),
	};
	rgb16_t color_lookup_r = {
		MIN(MAX((int)color_ref_lookup.r + (color_ref_lookup.r == color_min_lookup.r ? 1 : -1), 0), COLOR_TABLE_SIZE - 1),
		color_ref_lookup.g,
		color_ref_lookup.b
	};
	rgb16_t color_lookup_g = {
		color_ref_lookup.r,
		MIN(MAX((int)color_ref_lookup.g + (color_ref_lookup.g == color_min_lookup.g ? 1 : -1), 0), COLOR_TABLE_SIZE - 1),
		color_ref_lookup.b
	};
	rgb16_t color_lookup_b = {
		color_ref_lookup.r,
		color_ref_lookup.g,
		MIN(MAX((int)color_ref_lookup.b + (color_ref_lookup.b == color_min_lookup.b ? 1 : -1), 0), COLOR_TABLE_SIZE - 1)
	};
	rgb16_t color_ref = {
		color_ref_lookup.r * LOOKUP_DIV,
		color_ref_lookup.g * LOOKUP_DIV,
		color_ref_lookup.b * LOOKUP_DIV
	};
	rgb16_t color_max = {
		color_lookup_r.r * LOOKUP_DIV,
		color_lookup_g.g * LOOKUP_DIV,
		color_lookup_b.b * LOOKUP_DIV
	};
	rgb16_t color_corrected_ref;
	rgb16_t color_corrected_r;
	rgb16_t color_corrected_g;
	rgb16_t color_corrected_b;
	lookup_color(&color_ref_lookup, &color_corrected_ref);
	lookup_color(&color_lookup_r, &color_corrected_r);
	lookup_color(&color_lookup_g, &color_corrected_g);
	lookup_color(&color_lookup_b, &color_corrected_b);

	int32_t delta_in_r = (int32_t)in->r - (int32_t)color_ref.r;
	int32_t delta_in_g = (int32_t)in->g - (int32_t)color_ref.g;
	int32_t delta_in_b = (int32_t)in->b - (int32_t)color_ref.b;
	int32_t delta_ref_r = (int32_t)color_max.r - (int32_t)color_ref.r;
	int32_t delta_ref_g = (int32_t)color_max.g - (int32_t)color_ref.g;
	int32_t delta_ref_b = (int32_t)color_max.b - (int32_t)color_ref.b;
	rgb16_t color_out = color_corrected_ref;

	if (delta_ref_r) {
		color_out.r += DIV_ROUND(((int32_t)color_corrected_r.r - (int32_t)color_corrected_ref.r) * delta_in_r, delta_ref_r);
		color_out.g += DIV_ROUND(((int32_t)color_corrected_r.g - (int32_t)color_corrected_ref.g) * delta_in_r, delta_ref_r);
		color_out.b += DIV_ROUND(((int32_t)color_corrected_r.b - (int32_t)color_corrected_ref.b) * delta_in_r, delta_ref_r);
	}
	if (delta_ref_g) {
		color_out.r += DIV_ROUND(((int32_t)color_corrected_g.r - (int32_t)color_corrected_ref.r) * delta_in_g, delta_ref_g);
		color_out.g += DIV_ROUND(((int32_t)color_corrected_g.g - (int32_t)color_corrected_ref.g) * delta_in_g, delta_ref_g);
		color_out.b += DIV_ROUND(((int32_t)color_corrected_g.b - (int32_t)color_corrected_ref.b) * delta_in_g, delta_ref_g);
	}
	if (delta_ref_b) {
		color_out.r += DIV_ROUND(((int32_t)color_corrected_b.r - (int32_t)color_corrected_ref.r) * delta_in_b, delta_ref_b);
		color_out.g += DIV_ROUND(((int32_t)color_corrected_b.g - (int32_t)color_corrected_ref.g) * delta_in_b, delta_ref_b);
		color_out.b += DIV_ROUND(((int32_t)color_corrected_b.b - (int32_t)color_corrected_ref.b) * delta_in_b, delta_ref_b);
	}

	*out = color_out;
}

#define GLOBAL_BRIGHT(comp) ((comp) > NUM_LEDS ? (comp) >> 4 : 0)
#define LOCAL_BRIGHT(comp, i_) (GLOBAL_BRIGHT(comp) + (((i_ < (comp - (GLOBAL_BRIGHT(comp) << 4)))) ? 1 : 0))

static void leds_set_color(uint8_t *data, uint16_t r, uint16_t g, uint16_t b) {
	rgb16_t color_in = { r, g, b };

	rgb16_t color_out;
	apply_color_correction_per_channel(&color_in, &color_out);

	uint16_t r_corrected = color_out.r;
	uint16_t g_corrected = color_out.g;
	uint16_t b_corrected = color_out.b;
	uint8_t local_r1, local_g1, local_b1;
	uint8_t local_r2, local_g2, local_b2;

	local_r1 = MIN(r_corrected, 255);
	local_g1 = MIN(g_corrected, 255);
	local_b1 = MIN(b_corrected, 255);
	r_corrected -= (uint16_t)local_r1;
	g_corrected -= (uint16_t)local_g1;
	b_corrected -= (uint16_t)local_b1;

	local_r2 = MIN(r_corrected, 255);
	local_g2 = MIN(g_corrected, 255);
	local_b2 = MIN(b_corrected, 255);
	r_corrected -= (uint16_t)local_r2;
	g_corrected -= (uint16_t)local_g2;
	b_corrected -= (uint16_t)local_b2;

	for (int i = 0; i < NUM_LEDS; i++) {
		if (i % 2) {
			data = led_set_color(data, local_r1, local_g1, local_b1);
		} else {
			data = led_set_color(data, local_r2, local_g2, local_b2);
		}
	}
}

static void pre_schedule(platform_t *platform) {
	platform_tallylight_v2_t *plat = container_of(platform, platform_tallylight_v2_t, base);

	spi_transaction_t *xfer_;
	if (plat->transaction_pending) {
		spi_device_get_trans_result(plat->dev, &xfer_, portMAX_DELAY);
		plat->transaction_pending = false;
	}
}

static void set_rdb_led_color(platform_t *platform, uint16_t r, uint16_t g, uint16_t b) {
	platform_tallylight_v2_t *plat = container_of(platform, platform_tallylight_v2_t, base);

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

esp_err_t platform_tallylight_v2_probe(platform_t **ret) {
	platform_tallylight_v2_t *tally = calloc(1, sizeof(platform_tallylight_v2_t));
	if (!tally) {
		return ESP_ERR_NO_MEM;
	}

	if (gpio_get_level(REV_ID_GPIO) != 0) {
		return ESP_FAIL;
	}
	if (gpio_get_level(MAC_CS_GPIO) != 1) {
		return ESP_FAIL;
	}
	if (gpio_get_level(MAC_SI_GPIO) != 1) {
		return ESP_FAIL;
	}
	if (gpio_get_level(MAC_RST_GPIO) != 1) {
		return ESP_FAIL;
	}

	gpio_set_pull_mode(REV_ID_GPIO, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(MAC_CS_GPIO, GPIO_PULLDOWN_ONLY);
	gpio_set_pull_mode(MAC_SI_GPIO, GPIO_PULLDOWN_ONLY);
	gpio_set_pull_mode(MAC_RST_GPIO, GPIO_PULLDOWN_ONLY);
	vTaskDelay(pdMS_TO_TICKS(100));

	if (gpio_get_level(REV_ID_GPIO) != 0) {
		return ESP_FAIL;
	}
	if (gpio_get_level(MAC_CS_GPIO) != 1) {
		return ESP_FAIL;
	}
	if (gpio_get_level(MAC_SI_GPIO) != 1) {
		return ESP_FAIL;
	}
	if (gpio_get_level(MAC_RST_GPIO) != 1) {
		return ESP_FAIL;
	}

	spi_bus_config_t spi_bus_cfg = {
		.mosi_io_num = 6,
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
	ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &tally->dev));

	ESP_LOGI(TAG, "Allocating %lu bytes of DMA memory", (unsigned long)dma_buf_len);
	tally->led_data = heap_caps_malloc(dma_buf_len, MALLOC_CAP_DMA);
	ESP_ERROR_CHECK(!tally->led_data);
	memset(tally->led_data, 0, dma_buf_len);

	leds_set_color(tally->led_data + BYTES_RESET, 0x00, 0x00, 0x00);

	tally->xfer.length = dma_buf_len * 8;
	tally->xfer.rxlength = 0;
	tally->xfer.tx_buffer = tally->led_data;
	tally->xfer.rx_buffer = NULL;
	ESP_ERROR_CHECK(spi_device_transmit(tally->dev, &tally->xfer));

	platform_init(&tally->base, &lacklight_ops, "TallyLight v2");
	*ret = &tally->base;

	return 0;
}
