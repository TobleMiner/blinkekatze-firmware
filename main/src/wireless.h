#pragma once

#include <stddef.h>
#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <esp_err.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>

#define WIRELESS_MAX_PACKET_SIZE	128
#define WIRELESS_AP_PASSWORD_LENGTH	 16

typedef enum wireless_packet_type {
	WIRELESS_PACKET_TYPE_BONK = 0,
	WIRELESS_PACKET_TYPE_NEIGHBOUR_ADVERTISEMENT = 1,
	WIRELESS_PACKET_TYPE_NEIGHBOUR_STATUS = 2,
	WIRELESS_PACKET_TYPE_NEIGHBOUR_STATIC_INFO = 3,
	WIRELESS_PACKET_TYPE_OTA = 4
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
const char *wireless_get_ap_password(void);
esp_err_t wireless_connect_to_ap(wifi_config_t *sta_cfg);
esp_err_t wireless_disconnect_from_ap(void);
int wireless_get_ap_ifindex(void);
int wireless_get_sta_ifindex(void);
bool wireless_is_sta_connected(void);
const uint8_t *wireless_get_mac_address(void);
