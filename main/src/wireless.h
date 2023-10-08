#pragma once

#include <stddef.h>
#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <esp_err.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>

#define WIRELESS_MAX_PACKET_SIZE	64

typedef enum wireless_packet_type {
	WIRELESS_PACKET_TYPE_BONK = 0,
} wireless_packet_type_t;

typedef struct wireless_packet {
	int64_t rx_timestamp;
	uint8_t src_addr[ESP_NOW_ETH_ALEN];
	unsigned int len;
	uint8_t data[WIRELESS_MAX_PACKET_SIZE];
} wireless_packet_t;

esp_err_t wireless_init();
esp_err_t wireless_broadcast(const uint8_t *data, size_t len);
QueueHandle_t wireless_get_rx_queue(void);

esp_err_t wireless_scan_aps(void);
bool wireless_is_scan_done(void);
unsigned int wireless_get_num_scan_results(void);
esp_err_t wireless_get_scan_results(wifi_ap_record_t *ap_records, unsigned int *num_records);
void wireless_clear_scan_results(void);
