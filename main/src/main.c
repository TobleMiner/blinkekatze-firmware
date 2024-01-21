#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
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

#include "bonk.h"
#include "bq24295.h"
#include "bq27546.h"
#include "bq27546_dataflash.h"
#include "color_override.h"
#include "default_color.h"
#include "embedded_files.h"
#include "fast_hsv2rgb.h"
#include "i2c_bus.h"
#include "lis3dh.h"
#include "ltr_303als.h"
#include "neighbour.h"
#include "neighbour_rssi_delay_model.h"
#include "neighbour_static_info.h"
#include "neighbour_status.h"
#include "main.h"
#include "node_info.h"
#include "ota.h"
#include "power_control.h"
#include "rainbow_fade.h"
#include "scheduler.h"
#include "settings.h"
#include "shell.h"
#include "spl06.h"
#include "squish.h"
#include "state_of_charge.h"
#include "status_leds.h"
#include "strutil.h"
#include "uid.h"
#include "usb.h"
#include "util.h"
#include "wireless.h"

static const char *TAG = "main";

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

static squish_t squish;
static bonk_t bonk;
static bool is_rev2 = false;
static lis3dh_t accelerometer;
static SemaphoreHandle_t main_lock;
static StaticSemaphore_t main_lock_buffer;
static StaticEventGroup_t main_event_group_buffer;
static EventGroupHandle_t main_event_group;

static scheduler_task_t led_upate_task;

static void led_update(void *arg);
static void led_update(void *arg) {
	post_event(EVENT_LED);
	scheduler_schedule_task_relative(&led_upate_task, led_update, NULL, MS_TO_US(10));
}

void app_main(void) {
	gpio_reset_pin(0);
	gpio_reset_pin(2);

	settings_init();

	main_lock = xSemaphoreCreateMutexStatic(&main_lock_buffer);

	main_event_group = xEventGroupCreateStatic(&main_event_group_buffer);
	scheduler_init();
	usb_init();

	i2c_bus_t i2c_bus;
	i2c_bus_init(&i2c_bus, I2C_NUM_0, 0, 2, 100000);
	bq27546_t gauge;
	esp_err_t err =	bq27546_init(&gauge, &i2c_bus);
	if (err) {
		i2c_bus_deinit(&i2c_bus);
		i2c_bus_init(&i2c_bus, I2C_NUM_0, 8, 2, 100000);
		ESP_ERROR_CHECK(bq27546_init(&gauge, &i2c_bus));
		is_rev2 = true;
	}

#ifdef CONFIG_BK_GAUGE_DF_PROG
	err = bq27546_is_sealed(&gauge);
	if (err < 0) {
		ESP_LOGE(TAG, "Failed to get sealing status: %d", err);
		ESP_ERROR_CHECK(ESP_FAIL);
	} else {
		bool sealed = !!err;
		if (sealed) {
			ESP_LOGI(TAG, "Gauge sealed, skipping data flash programming");
		} else {
			ESP_LOGI(TAG, "Gauge not sealed, programming data flash...");
			err = bq27546_write_flash(&gauge, bq27546_inr18650_dataflash, bq27546_inr18650_dataflash_num_ops);
			if (err) {
				ESP_LOGE(TAG, "Programming failed: %d", err);
				ESP_ERROR_CHECK(ESP_FAIL);
			} else {
				ESP_LOGI(TAG, "Programming successful, enabling gauging...");
				vTaskDelay(pdMS_TO_TICKS(10000));
				ESP_ERROR_CHECK(bq27546_it_enable(&gauge));
				ESP_LOGI(TAG, "Sealing gauge...");
				vTaskDelay(pdMS_TO_TICKS(1000));
				bq27546_seal(&gauge);
				ESP_LOGI(TAG, "Done. Restarting.");
				esp_restart();
			}
		}
	}
#endif
	ESP_LOGI(TAG, "Battery voltage: %dmV", bq27546_get_voltage_mv(&gauge));
	int current_ma;
	ESP_ERROR_CHECK(bq27546_get_current_ma(&gauge, &current_ma));
	ESP_LOGI(TAG, "Battery current: %dmA", current_ma);

	spi_bus_config_t spi_bus_cfg = {
		.mosi_io_num = is_rev2 ? 3 : 7,
		.miso_io_num = is_rev2 ? -1 : 6,
		.sclk_io_num = is_rev2 ? -1 : 8,
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
	spi_device_handle_t dev;
	ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &dev));

	size_t dma_buf_len = ALIGN_UP(BYTES_RESET + BYTES_DATA + BYTES_RESET, 4);
	ESP_LOGI(TAG, "Allocating %lu bytes of DMA memory", (unsigned long)dma_buf_len);
	uint8_t *led_data = heap_caps_malloc(dma_buf_len, MALLOC_CAP_DMA);
	ESP_ERROR_CHECK(!led_data);
	memset(led_data, 0, dma_buf_len);

	leds_set_color(led_data + BYTES_RESET, 0x00, 0x00, 0x00);

	spi_transaction_t xfer = {
		.length = dma_buf_len * 8,
		.rxlength = 0,
		.tx_buffer = led_data,
		.rx_buffer = NULL
	};
	ESP_ERROR_CHECK(spi_device_transmit(dev, &xfer));

//	i2c_detect(&i2c_bus);

	bq24295_t charger;
	ESP_ERROR_CHECK(bq24295_init(&charger, &i2c_bus));

	ESP_ERROR_CHECK(power_control_init(&charger));

	ESP_ERROR_CHECK(wireless_init());

	neighbour_status_init(&gauge);

	neighbour_init();

	ESP_ERROR_CHECK(lis3dh_init(&accelerometer, &i2c_bus, 0x18));
	bonk_init(&bonk, &accelerometer);

	spl06_t barometer;
	if (is_rev2) {
		ESP_ERROR_CHECK(spl06_init_i2c(&barometer, &i2c_bus, 0x77));
	} else {
		ESP_ERROR_CHECK(spl06_init_spi(&barometer, SPI2_HOST, 9));
	}
	squish_init(&squish, &barometer);

	ltr_303als_t als;
	ESP_ERROR_CHECK(ltr_303als_init(&als, &i2c_bus));

	status_leds_init();
	status_led_set_strobe(STATUS_LED_RED, 20);

	ESP_ERROR_CHECK(ota_init());

	neighbour_static_info_init();
	node_info_init(&gauge);

	rainbow_fade_init();

	default_color_init();

	state_of_charge_init(&gauge);

	shell_init(&bonk);

	unsigned loop_interval_ms = 20;
	bool transaction_pending = false;
	uint64_t loops = 0;
	scheduler_task_init(&led_upate_task);
	scheduler_schedule_task_relative(&led_upate_task, led_update, NULL, MS_TO_US(10));
	while (1) {
		EventBits_t events = xEventGroupWaitBits(main_event_group, EVENTS, pdTRUE, pdFALSE, portMAX_DELAY);
		int64_t time_loop_start_us = esp_timer_get_time();
		xSemaphoreTake(main_lock, portMAX_DELAY);

		if (events & EVENT_WIRELESS) {
			wireless_packet_t packet;
			while (xQueueReceive(wireless_get_rx_queue(), &packet, 0)) {
				ESP_LOGD(TAG, "Dequeued packet, size: %u bytes", packet.len);
				if (packet.len >= 1) {
					const neighbour_t *neigh = neighbour_find_by_address(packet.src_addr);
					uint8_t packet_type = packet.data[0];
					status_led_strobe(STATUS_LED_RED);
					switch (packet_type) {
					case WIRELESS_PACKET_TYPE_BONK:
						bonk_rx(&bonk, &packet, neigh);
						break;
					case WIRELESS_PACKET_TYPE_NEIGHBOUR_ADVERTISEMENT:
						neighbour_rx(&packet);
						break;
					case WIRELESS_PACKET_TYPE_NEIGHBOUR_STATUS:
						neighbour_status_rx(&packet, neigh);
						break;
					case WIRELESS_PACKET_TYPE_NEIGHBOUR_STATIC_INFO:
						neighbour_static_info_rx(&packet, neigh);
						break;
					case WIRELESS_PACKET_TYPE_OTA:
						ota_rx(&packet, neigh);
						break;
					case WIRELESS_PACKET_TYPE_UID:
						uid_rx(&packet);
						break;
					case WIRELESS_PACKET_TYPE_SQUISH:
						squish_rx(&squish, &packet, neigh);
						break;
					case WIRELESS_PACKET_TYPE_RAINBOW_FADE:
						rainbow_fade_rx(&packet);
						break;
					case WIRELESS_PACKET_TYPE_COLOR_OVERRIDE:
						color_override_rx(&packet);
						break;
					case WIRELESS_PACKET_TYPE_POWER_CONTROL:
						power_control_rx(&packet);
						break;
					case WIRELESS_PACKET_TYPE_DEFAULT_COLOR:
						default_color_rx(&packet);
						break;
					case WIRELESS_PACKET_TYPE_STATE_OF_CHARGE:
						state_of_charge_rx(&packet);
						break;
					case WIRELESS_PACKET_TYPE_USB_CONFIG:
						usb_config_rx(&packet);
						break;
					default:
						ESP_LOGD(TAG, "Unknown packet type 0x%02x", packet_type);
					}
				}
			}
		}

		if (events & EVENT_SCHEDULER) {
			spi_transaction_t *xfer_;
			if (transaction_pending) {
				spi_device_get_trans_result(dev, &xfer_, portMAX_DELAY);
				transaction_pending = false;
			}
			if (!is_rev2) {
				gpio_set_direction(3, GPIO_MODE_OUTPUT);
				gpio_set_level(3, 0);
				gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[3], PIN_FUNC_GPIO);
			}

			scheduler_run();
		}

		if (events & EVENT_LED) {
			color_hsv_t hsv = { 0, HSV_SAT_MAX, HSV_VAL_MAX / 2 };
			default_color_apply(&hsv);
			rainbow_fade_apply(&hsv);
			bonk_apply(&bonk, &hsv);
			squish_apply(&squish, &hsv);
			state_of_charge_apply(&hsv);
			ota_indicate_update(&hsv);
			uid_apply(&hsv);

			uint16_t r, g, b;
			fast_hsv2rgb_32bit(hsv.h, hsv.s, hsv.v, &r, &g, &b);
			rgb16_t color_rgb = { r, g, b};
			color_override_apply(&color_rgb);
			if (power_control_is_powered_off()) {
				color_rgb.r = 0;
				color_rgb.g = 0;
				color_rgb.b = 0;
			}
			spi_transaction_t *xfer_;
			if (transaction_pending) {
				spi_device_get_trans_result(dev, &xfer_, portMAX_DELAY);
				transaction_pending = false;
			}
			if (!is_rev2) {
				gpio_set_direction(3, GPIO_MODE_OUTPUT);
				gpio_set_level(3, 0);
				gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[3], PIN_FUNC_GPIO);
			}

			leds_set_color(led_data + BYTES_RESET, color_rgb.r, color_rgb.g, color_rgb.b);

			xfer.length = dma_buf_len * 8;
			xfer.rxlength = 0;
			xfer.tx_buffer = led_data;
			xfer.rx_buffer = NULL;
			if (!is_rev2) {
				esp_rom_gpio_connect_out_signal(3, FSPID_OUT_IDX, false, false);
			}
			ESP_ERROR_CHECK(spi_device_queue_trans(dev, &xfer, 0));
			transaction_pending = true;

			status_leds_update();
		}

		if (wireless_is_scan_done()) {
			unsigned int num_results = wireless_get_num_scan_results();
			ESP_LOGD(TAG, "Scan complete, found %u APs", num_results);
			wifi_ap_record_t *scan_results = calloc(num_results, sizeof(wifi_ap_record_t));
			if (scan_results) {
				esp_err_t err = wireless_get_scan_results(scan_results, &num_results);
				if (!err) {
					for (int i = 0; i < num_results; i++) {
						wifi_ap_record_t *scan_result = &scan_results[i];
						if (!str_starts_with((const char *)scan_result->ssid, "blinkekatze_")) {
							continue;
						}

						neighbour_update_rssi(scan_result->bssid, scan_result->rssi);
					}
				}
				free(scan_results);
			}
			wireless_clear_scan_results();
		}
		xSemaphoreGive(main_lock);

		if (loops % 500 == 250) {
			wireless_scan_aps();
		}

		int64_t time_loop_end_us = esp_timer_get_time();
		int dt_ms = DIV_ROUND(time_loop_end_us - time_loop_start_us, 1000);
		if (dt_ms > loop_interval_ms) {
			ESP_LOGW(TAG, "Can't keep up, update took %d ms", dt_ms);
		}
		loops++;
	}
}

void main_loop_lock() {
	xSemaphoreTake(main_lock, portMAX_DELAY);
}

void main_loop_unlock() {
	xSemaphoreGive(main_lock);
}

void post_event(EventBits_t bits) {
	xEventGroupSetBits(main_event_group, bits);
}
