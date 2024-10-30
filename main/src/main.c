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
#include "platform.h"
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

static squish_t squish;
static bonk_t bonk;
static SemaphoreHandle_t main_lock;
static StaticSemaphore_t main_lock_buffer;
static StaticEventGroup_t main_event_group_buffer;
static EventGroupHandle_t main_event_group;

static scheduler_task_t led_update_task;

static void led_update(void *arg);
static void led_update(void *arg) {
	post_event(EVENT_LED);
	scheduler_schedule_task_relative(&led_update_task, led_update, NULL, MS_TO_US(10));
}

void app_main(void) {
	gpio_reset_pin(0);
	gpio_reset_pin(2);

	settings_init();

	main_lock = xSemaphoreCreateMutexStatic(&main_lock_buffer);

	main_event_group = xEventGroupCreateStatic(&main_event_group_buffer);
	scheduler_init();
	usb_init();

	platform_t *platform;
	ESP_ERROR_CHECK(platform_probe(&platform));

	ESP_ERROR_CHECK(power_control_init(platform->charger, platform->gauge));

	ESP_ERROR_CHECK(wireless_init());

	neighbour_status_init(platform->gauge);

	neighbour_init();

	bonk_init(&bonk, platform->accelerometer);

	squish_init(&squish, platform->barometer);

	status_leds_init();
	status_led_set_strobe(STATUS_LED_RED, 20);

	ESP_ERROR_CHECK(ota_init());

	neighbour_static_info_init();
	node_info_init(platform->gauge);

	rainbow_fade_init();

	default_color_init();

	state_of_charge_init(platform->gauge);

	shell_init(&bonk);

	unsigned loop_interval_ms = 20;
	uint64_t loops = 0;
	scheduler_task_init(&led_update_task);
	scheduler_schedule_task_relative(&led_update_task, led_update, NULL, MS_TO_US(10));
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
					case WIRELESS_PACKET_TYPE_NEIGHBOUR_RSSI_REPORT:
						neighbour_rx_rssi_info(&packet);
						break;
					default:
						ESP_LOGD(TAG, "Unknown packet type 0x%02x", packet_type);
					}
				}
			}
		}

		if (events & EVENT_SCHEDULER) {
			platform_pre_schedule(platform);
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
			platform_set_rgb_led_color(platform, color_rgb.r, color_rgb.g, color_rgb.b);

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
