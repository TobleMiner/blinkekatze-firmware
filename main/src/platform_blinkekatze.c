#include "platform_blinkekatze.h"

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

#include "bq27546_dataflash.h"
#include "color.h"
#include "embedded_files.h"
#include "fast_hsv2rgb.h"
#include "util.h"

#define NUM_LEDS	16
#define BITS_PER_SYMBOL	4
#define SYMBOL_ZERO	0b0001
#define SYMBOL_ONE	0b0111

#define BYTES_DATA	((NUM_LEDS) * 24 * (BITS_PER_SYMBOL)) / 8
#define BYTES_RESET	(250 / 8)

static const char *TAG = "blinkekatze";
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

static const rgb16_t *colorcal_table = (const rgb16_t *)EMBEDDED_FILE_PTR(colorcal_16x16x16_12bit_bin);
//static const rgb16_t *colorcal_table = (const rgb16_t *)EMBEDDED_FILE_PTR(colorcal_32x32x32_12bit_bin);

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
//	int led_map[NUM_LEDS] = { 0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15 };
	int led_map[NUM_LEDS] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
/*
	for (int i = 0; i < NUM_LEDS; i++) {
		uint8_t local_r = MIN(LOCAL_BRIGHT(color_out.r, led_map[i]), 255);
		uint8_t local_g = MIN(LOCAL_BRIGHT(color_out.g, led_map[i]), 255);
		uint8_t local_b = MIN(LOCAL_BRIGHT(color_out.b, led_map[i]), 255);
		data = led_set_color(data, local_r, local_g, local_b);
	}
*/

	uint16_t r_corrected = color_out.r;
	uint16_t g_corrected = color_out.g;
	uint16_t b_corrected = color_out.b;
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
	platform_blinkekatze_t *plat = container_of(platform, platform_blinkekatze_t, base);

	spi_transaction_t *xfer_;
	if (plat->transaction_pending) {
		spi_device_get_trans_result(plat->dev, &xfer_, portMAX_DELAY);
		plat->transaction_pending = false;
	}
	if (!plat->is_rev2) {
		gpio_set_direction(3, GPIO_MODE_OUTPUT);
		gpio_set_level(3, 0);
		gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[3], PIN_FUNC_GPIO);
	}

}

static void set_rdb_led_color(platform_t *platform, uint16_t r, uint16_t g, uint16_t b) {
	platform_blinkekatze_t *plat = container_of(platform, platform_blinkekatze_t, base);

	spi_transaction_t *xfer_;
	if (plat->transaction_pending) {
		spi_device_get_trans_result(plat->dev, &xfer_, portMAX_DELAY);
		plat->transaction_pending = false;
	}
	if (!plat->is_rev2) {
		gpio_set_direction(3, GPIO_MODE_OUTPUT);
		gpio_set_level(3, 0);
		gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[3], PIN_FUNC_GPIO);
	}

	leds_set_color(plat->led_data + BYTES_RESET, r, g, b);

	plat->xfer.length = dma_buf_len * 8;
	plat->xfer.rxlength = 0;
	plat->xfer.tx_buffer = plat->led_data;
	plat->xfer.rx_buffer = NULL;
	if (!plat->is_rev2) {
		esp_rom_gpio_connect_out_signal(3, FSPID_OUT_IDX, false, false);
	}
	ESP_ERROR_CHECK(spi_device_queue_trans(plat->dev, &plat->xfer, 0));
	plat->transaction_pending = true;
}

static const platform_ops_t blinkekatze_ops = {
	.probe = platform_blinkekatze_probe,
	.pre_schedule = pre_schedule,
	.set_rgb_led_color = set_rdb_led_color
};

platform_def_t platform_blinkekatze = {
	.ops = &blinkekatze_ops,
	.name = "blinkekatze"
};

esp_err_t platform_blinkekatze_probe(platform_t **ret) {
	platform_blinkekatze_t *katze = calloc(1, sizeof(platform_blinkekatze_t));
	if (!katze) {
		return ESP_ERR_NO_MEM;
	}

	i2c_bus_init(&katze->i2c_bus, I2C_NUM_0, 0, 2, 100000);
	esp_err_t err =	bq27546_init(&katze->gauge, &katze->i2c_bus);
	if (err) {
		i2c_bus_deinit(&katze->i2c_bus);
		i2c_bus_init(&katze->i2c_bus, I2C_NUM_0, 8, 2, 100000);
		err = bq27546_init(&katze->gauge, &katze->i2c_bus);
		if (err) {
			*ret = NULL;
			return 0;
		}
		katze->is_rev2 = true;
	}

#ifdef CONFIG_BK_GAUGE_DF_PROG
	err = bq27546_is_sealed(&katze->gauge);
	if (err < 0) {
		ESP_LOGE(TAG, "Failed to get sealing status: %d", err);
		ESP_ERROR_CHECK(ESP_FAIL);
	} else {
		bool sealed = !!err;
		if (sealed) {
			ESP_LOGI(TAG, "Gauge sealed, skipping data flash programming");
		} else {
			ESP_LOGI(TAG, "Gauge not sealed, programming data flash...");
			err = bq27546_write_flash(&katze->gauge, bq27546_inr18650_dataflash, bq27546_inr18650_dataflash_num_ops);
			if (err) {
				ESP_LOGE(TAG, "Programming failed: %d", err);
				ESP_ERROR_CHECK(ESP_FAIL);
			} else {
				ESP_LOGI(TAG, "Programming successful, enabling gauging...");
				vTaskDelay(pdMS_TO_TICKS(10000));
				ESP_ERROR_CHECK(bq27546_it_enable(&katze->gauge));
				ESP_LOGI(TAG, "Sealing gauge...");
				vTaskDelay(pdMS_TO_TICKS(1000));
				bq27546_seal(&katze->gauge);
				ESP_LOGI(TAG, "Done. Restarting.");
				esp_restart();
			}
		}
	}
#endif
	ESP_LOGI(TAG, "Battery voltage: %dmV", bq27546_get_voltage_mv(&katze->gauge));
	int current_ma;
	ESP_ERROR_CHECK(bq27546_get_current_ma(&katze->gauge, &current_ma));
	ESP_LOGI(TAG, "Battery current: %dmA", current_ma);

	spi_bus_config_t spi_bus_cfg = {
		.mosi_io_num = katze->is_rev2 ? 3 : 7,
		.miso_io_num = katze->is_rev2 ? -1 : 6,
		.sclk_io_num = katze->is_rev2 ? -1 : 8,
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

//	i2c_detect(&katze->i2c_bus);

	ESP_ERROR_CHECK(bq24295_init(&katze->charger, &katze->i2c_bus));

	ESP_ERROR_CHECK(lis3dh_init(&katze->accelerometer, &katze->i2c_bus, 0x18));

	if (katze->is_rev2) {
		ESP_ERROR_CHECK(spl06_init_i2c(&katze->barometer, &katze->i2c_bus, 0x77));
	} else {
		ESP_ERROR_CHECK(spl06_init_spi(&katze->barometer, SPI2_HOST, 9));
	}

	ESP_ERROR_CHECK(ltr_303als_init(&katze->als, &katze->i2c_bus));

	platform_init(&katze->base, &platform_blinkekatze);
	katze->base.gauge = &katze->gauge;
	katze->base.charger = &katze->charger;
	katze->base.als = &katze->als;
	katze->base.accelerometer = &katze->accelerometer;
	katze->base.barometer = &katze->barometer;
	*ret = &katze->base;

	return 0;
}
