#include "wireless.h"

#include <string.h>

#include <esp_log.h>
#include <esp_mac.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <nvs_flash.h>

#define WIRELESS_RX_QUEUE_SIZE		 8


static const char *TAG = "wireless";

const uint8_t wireless_broadcast_address[ESP_NOW_ETH_ALEN] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

static QueueHandle_t rx_queue;
static bool scan_done = false;

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int data_len) {
	int64_t rx_timestamp = esp_timer_get_time();

	ESP_LOGD(TAG, "Received %d bytes", data_len);
	if (data_len <= WIRELESS_MAX_PACKET_SIZE) {
		wireless_packet_t packet = {
			.rx_timestamp = rx_timestamp,
			.len = data_len
		};
		memcpy(packet.src_addr, info->src_addr, sizeof(packet.src_addr));
		memcpy(packet.data, data, data_len);

		if (xQueueSend(rx_queue, &packet, 0) != pdTRUE) {
			ESP_LOGW(TAG, "RX queue overflow. Dropping packet");
		} else {
			ESP_LOGD(TAG, "Packet queued, %u bytes", packet.len);
		}
	}
}

static void sta_event_handler(void *arg, esp_event_base_t event_base,
			      int32_t event_id, void *event_data) {
	scan_done = true;
}

static void generate_psk(char *dst, size_t len) {
	static const char *non_confusable_characters = "23467abcdefjkprtxyz";

	while (len--) {
	        uint32_t rnd = esp_random();

	        rnd %= strlen(non_confusable_characters);
	        *dst++ = non_confusable_characters[rnd];
	}
}

esp_err_t wireless_init() {
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		nvs_flash_erase();
		err = nvs_flash_init();
	}
	if (err) {
		return err;
	}

	err = esp_netif_init();
	if (err) {
		return err;
	}

	err = esp_event_loop_create_default();
	if (err) {
		return err;
	}

	wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
	err = esp_wifi_init(&wifi_cfg);
	if (err) {
		return err;
	}

	err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
	if (err) {
		return err;
	}

	err = esp_wifi_set_mode(WIFI_MODE_APSTA);
	if (err) {
		return err;
	}

	wifi_country_t country = {
		.cc = { 'J', 'P', 0 },
		.schan = 1,
		.nchan = 14,
		.policy = WIFI_COUNTRY_POLICY_MANUAL
	};
	err = esp_wifi_set_country(&country);
	if (err) {
		return err;
	}

	uint8_t ap_mac_address[6];
	err = esp_wifi_get_mac(WIFI_IF_AP, ap_mac_address);
	if (err) {
		return err;
	}

	wifi_config_t ap_cfg = {
		.ap = {
			.ssid = { 0 },
			.password = { 0 },
			.channel = 14,
			.authmode = WIFI_AUTH_WPA2_PSK,
			.max_connection = 1,
			.beacon_interval = 500
		}
	};
	snprintf((char *)ap_cfg.ap.ssid, 32, "blinkekatze_"MACSTR, MAC2STR(ap_mac_address));
	generate_psk((char *)ap_cfg.ap.password, 32);
	err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
	if (err) {
		return err;
	}

	err = esp_wifi_start();
	if (err) {
		return err;
	}

	err = esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);
	if (err) {
		return err;
	}

	err = esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE,
						  &sta_event_handler, NULL, NULL);
	if (err) {
		return err;
	}

	err = esp_now_init();
	if (err) {
		ESP_LOGE(TAG, "Failed to initialize ESP-NOW: %d", err);
		return err;
	}

	esp_now_peer_info_t bcast_peer = {
		.channel = 14,
		.ifidx = WIFI_IF_AP,
		.encrypt = false,
	};
	memcpy(bcast_peer.peer_addr, wireless_broadcast_address, ESP_NOW_ETH_ALEN);
	err = esp_now_add_peer(&bcast_peer);
	if (err) {
		ESP_LOGE(TAG, "Failed to add broadcast peer: %d", err);
		return err;
	}

	rx_queue = xQueueCreate(WIRELESS_RX_QUEUE_SIZE, sizeof(wireless_packet_t));
	if (!rx_queue) {
		return ESP_ERR_NO_MEM;
	}

	return esp_now_register_recv_cb(recv_cb);
}

esp_err_t wireless_broadcast(const uint8_t *data, size_t len) {
	return esp_now_send(wireless_broadcast_address, data, len);
}

QueueHandle_t wireless_get_rx_queue() {
	return rx_queue;
}

esp_err_t wireless_scan_aps(void) {
	wifi_scan_config_t scan_cfg = {
		.channel = 14,
		.scan_type = WIFI_SCAN_TYPE_PASSIVE
	};
	esp_err_t err = esp_wifi_scan_start(&scan_cfg, false);
	if (!err) {
		scan_done = false;
	}
	return err;
}

bool wireless_is_scan_done(void) {
	return scan_done;
}

unsigned int wireless_get_num_scan_results(void) {
	uint16_t num_ap;
	esp_err_t err = esp_wifi_scan_get_ap_num(&num_ap);
	if (err) {
		return 0;
	}
	return num_ap;
}

esp_err_t wireless_get_scan_results(wifi_ap_record_t *ap_records, unsigned int *num_records) {
	uint16_t num_records_u16 = *num_records;
	esp_err_t err = esp_wifi_scan_get_ap_records(&num_records_u16, ap_records);
	*num_records = num_records_u16;
	return err;
}

void wireless_clear_scan_results(void) {
	esp_wifi_clear_ap_list();
	scan_done = false;
}
