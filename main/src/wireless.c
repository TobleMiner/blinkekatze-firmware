#include "wireless.h"

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <esp_log.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <mbedtls/md.h>
#include <nvs_flash.h>
#include <sdkconfig.h>

#include "embedded_files.h"
#include "main.h"
#include "neighbour.h"
#include "network.h"
#include "util.h"

#define WIRELESS_RX_QUEUE_SIZE		8
#define WIRELESS_HMAC_CTX_CNT		3
#define WIRELESS_REPLAY_AGE_LIMIT_MS	100
#define WIRELESS_REPLAY_BUFFER_SIZE	100

typedef struct wireless_packet_hdr {
	union {
		struct {
			uint32_t timestamp;
			uint32_t packet_cnt;
			uint32_t random_id;
		};
		uint8_t data[12];
	} nonce;
	uint8_t short_hmac[8];
} wireless_packet_hdr_t;

typedef struct wireless_hmac_entry {
	int64_t timestamp_ms;
	uint8_t short_hmac[8];
} wireless_hmac_entry_t;

static const char *TAG = "wireless";

static const uint8_t wireless_broadcast_address[ESP_NOW_ETH_ALEN] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

static uint8_t wireless_encryption_key[WIRELESS_ENCRYPTION_KEY_SIZE];
static uint32_t wireless_random_id;
static uint32_t wireless_packet_tx_cnt = 0;

static bool wireless_encryption_enabled = true;
static bool replay_protection_enabled = true;
static mbedtls_md_context_t wireless_hmac_ctx[WIRELESS_HMAC_CTX_CNT];
static bool wireless_hmac_map[WIRELESS_HMAC_CTX_CNT] = { 0 };
static SemaphoreHandle_t wireless_hmac_lock;
static StaticSemaphore_t wireless_hmac_lock_buffer;
static portMUX_TYPE wireless_hmac_spinlock = portMUX_INITIALIZER_UNLOCKED;
static size_t replay_buffer_read = 0;
static size_t replay_buffer_write = 0;
static wireless_hmac_entry_t wireless_replay_buffer[WIRELESS_REPLAY_BUFFER_SIZE] = { 0 };

static char ap_password[WIRELESS_AP_PASSWORD_LENGTH + 1] = { 0 };

static QueueHandle_t rx_queue;
static bool scan_done = false;
static bool sta_connected = false;
static esp_netif_t *ap_netif = NULL;
static esp_netif_t *sta_netif = NULL;
static uint8_t ap_mac_address[ESP_NOW_ETH_ALEN];

static int hmac_take(void) {
	int idx = -1;
	xSemaphoreTake(wireless_hmac_lock, portMAX_DELAY);
	taskENTER_CRITICAL(&wireless_hmac_spinlock);
	for (int i = 0; i < WIRELESS_HMAC_CTX_CNT; i++) {
		if (!wireless_hmac_map[i]) {
			wireless_hmac_map[i] = true;
			idx = i;
			break;
		}
	}
	taskEXIT_CRITICAL(&wireless_hmac_spinlock);
	return idx;
}

static void hmac_put(int hmac_idx) {
	taskENTER_CRITICAL(&wireless_hmac_spinlock);
	wireless_hmac_map[hmac_idx] = false;
	taskEXIT_CRITICAL(&wireless_hmac_spinlock);
	xSemaphoreGive(wireless_hmac_lock);
}

static void hmac_ctx_reinit(void) {
	for (int i = 0; i < WIRELESS_HMAC_CTX_CNT; i++) {
		xSemaphoreTake(wireless_hmac_lock, portMAX_DELAY);
	}
	for (int i = 0; i < WIRELESS_HMAC_CTX_CNT; i++) {
		mbedtls_md_context_t *hmac = &wireless_hmac_ctx[i];
		mbedtls_md_init(hmac);
		mbedtls_md_setup(hmac, mbedtls_md_info_from_type(MBEDTLS_MD_SHA1), 1);
		mbedtls_md_hmac_starts(hmac, wireless_encryption_key, WIRELESS_ENCRYPTION_KEY_SIZE);
	}
	for (int i = 0; i < WIRELESS_HMAC_CTX_CNT; i++) {
		xSemaphoreGive(wireless_hmac_lock);
	}
}

static void hmac_ctx_init(void) {
	wireless_hmac_lock = xSemaphoreCreateCountingStatic(WIRELESS_HMAC_CTX_CNT, WIRELESS_HMAC_CTX_CNT, &wireless_hmac_lock_buffer);
	hmac_ctx_reinit();
}

static void packet_generate_hmac(uint8_t *dst, const uint8_t *src, size_t len) {
	int idx = hmac_take();
	ESP_ERROR_CHECK(idx < 0);
	mbedtls_md_context_t *hmac = &wireless_hmac_ctx[idx];
	mbedtls_md_hmac_update(hmac, ap_mac_address, sizeof(ap_mac_address));
	mbedtls_md_hmac_update(hmac, src, len);
	mbedtls_md_hmac_finish(hmac, dst);
	mbedtls_md_hmac_reset(hmac);
	hmac_put(idx);
}

static bool packet_validate_timestamp(const wireless_packet_hdr_t *hdr) {
	if (!replay_protection_enabled || !neighbour_has_neighbours()) {
		return true;
	}
	uint32_t now_ms_ish = (uint64_t)neighbour_get_global_clock() >> 10;
	return hdr->nonce.timestamp + WIRELESS_REPLAY_AGE_LIMIT_MS > now_ms_ish;
}

static bool packet_check_replay(const wireless_packet_hdr_t *hdr) {
	if (!replay_protection_enabled) {
		return true;
	}

	int64_t now_ms = esp_timer_get_time() / 1000LL;
	size_t idx = replay_buffer_read;
	bool replay_detected = false;
	while (idx != replay_buffer_write) {
		wireless_hmac_entry_t *entry = &wireless_replay_buffer[idx];
		idx++;
		idx %= ARRAY_SIZE(wireless_replay_buffer);

		int64_t age_ms = now_ms - entry->timestamp_ms;
		if (age_ms > WIRELESS_REPLAY_AGE_LIMIT_MS) {
			replay_buffer_read = idx;
		}

		if (!memcmp(entry->short_hmac, hdr->short_hmac, sizeof(hdr->short_hmac))) {
			replay_detected = true;
		}
	}

	return !replay_detected;
}

static void push_hmac_to_replay_buffer(const wireless_packet_hdr_t *hdr) {
	int64_t now_ms = esp_timer_get_time() / 1000LL;
	wireless_hmac_entry_t entry = {
		.timestamp_ms = now_ms
	};
	memcpy(entry.short_hmac, hdr->short_hmac, sizeof(entry.short_hmac));
	wireless_replay_buffer[replay_buffer_write] = entry;
	replay_buffer_write++;
	replay_buffer_write %= ARRAY_SIZE(wireless_replay_buffer);
	if (replay_buffer_write == replay_buffer_read) {
		replay_buffer_read++;
		replay_buffer_read %= ARRAY_SIZE(wireless_replay_buffer);
	}
}

static bool packet_validate_hmac(const uint8_t *data, size_t data_len, const uint8_t *peer_address, const wireless_packet_hdr_t *hdr) {
	int idx = hmac_take();
	ESP_ERROR_CHECK(idx < 0);
	mbedtls_md_context_t *hmac = &wireless_hmac_ctx[idx];
	mbedtls_md_hmac_update(hmac, peer_address, ESP_NOW_ETH_ALEN);
	mbedtls_md_hmac_update(hmac, data, data_len);
	uint8_t digest[20];
	mbedtls_md_hmac_finish(hmac, digest);
	mbedtls_md_hmac_reset(hmac);
	hmac_put(idx);
	return !memcmp(digest, hdr->short_hmac, sizeof(hdr->short_hmac));
}

static void packet_crypt(uint8_t *dst, const uint8_t *src, size_t len, const wireless_packet_hdr_t *hdr) {
	chacha20_ctx_t chacha20;
	chacha20_init(&chacha20, wireless_encryption_key, hdr->nonce.data, 0);
	chacha20_xor(&chacha20, dst, src, len);
}

static bool rx_packet_encrypted(wireless_packet_t *packet, const esp_now_recv_info_t *info, const uint8_t *data, int data_len) {
	if (data_len >= sizeof(wireless_packet_hdr_t) &&
	    data_len <= WIRELESS_MAX_PACKET_SIZE - sizeof(wireless_packet_hdr_t)) {
		wireless_packet_hdr_t hdr;
		memcpy(&hdr, data, sizeof(wireless_packet_hdr_t));
		const uint8_t *payload = data + sizeof(wireless_packet_hdr_t);
		size_t payload_len = data_len - sizeof(wireless_packet_hdr_t);
		packet_crypt(packet->data, payload, payload_len, &hdr);
		bool is_valid = packet_validate_timestamp(&hdr) &&
				packet_check_replay(&hdr) &&
				packet_validate_hmac(packet->data, payload_len, info->src_addr, &hdr);
		if (is_valid) {
			push_hmac_to_replay_buffer(&hdr);
			packet->len = payload_len;
			return true;
		}
	}

	return false;
}

static bool rx_packet_plain(wireless_packet_t *packet, const uint8_t *data, int data_len) {
	if (data_len > 0 && data_len <= WIRELESS_MAX_PACKET_SIZE) {
		memcpy(packet->data, data, data_len);
		packet->len = data_len;
		return true;
	}

	return false;
}

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int data_len) {
	int64_t rx_timestamp = esp_timer_get_time();

	ESP_LOGD(TAG, "Received %d bytes", data_len);
	wireless_packet_t packet = {
		.rx_timestamp = rx_timestamp
	};

	bool packet_valid;
	if (wireless_encryption_enabled) {
		packet_valid = rx_packet_encrypted(&packet, info, data, data_len);
	} else {
		packet_valid = rx_packet_plain(&packet, data, data_len);
	}

	if (packet_valid) {
		memcpy(packet.src_addr, info->src_addr, sizeof(packet.src_addr));
		if (xQueueSend(rx_queue, &packet, 0) != pdTRUE) {
			ESP_LOGW(TAG, "RX queue overflow. Dropping packet");
		} else {
			ESP_LOGD(TAG, "Packet queued, %u bytes", packet.len);
			post_event(EVENT_WIRELESS);
		}
	}
}

static void sta_event_handler(void *arg, esp_event_base_t event_base,
			      int32_t event_id, void *event_data) {
	switch (event_id) {
	case WIFI_EVENT_SCAN_DONE:
		scan_done = true;
		break;
	case  WIFI_EVENT_STA_DISCONNECTED:
		sta_connected = false;
		break;
	case WIFI_EVENT_STA_CONNECTED:
		sta_connected = true;
		break;
	}
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
	esp_err_t err = network_init();
	if (err) {
		return err;
	}

	ap_netif = esp_netif_create_default_wifi_ap();
	if (!ap_netif) {
		return ESP_FAIL;
	}

	sta_netif = esp_netif_create_default_wifi_sta();
	if (!sta_netif) {
		return ESP_FAIL;
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

#ifdef CONFIG_BK_WLAN_REG
	wifi_country_t country = {
		.cc = { 0 },
		.schan = 1,
		.nchan = 14,
		.policy = WIFI_COUNTRY_POLICY_MANUAL
	};
	strncpy(country.cc, CONFIG_BK_WLAN_REG_CODE, sizeof(country.cc));
	err = esp_wifi_set_country(&country);
	if (err) {
		return err;
	}
#endif

	err = esp_wifi_get_mac(WIFI_IF_AP, ap_mac_address);
	if (err) {
		return err;
	}

	wifi_config_t ap_cfg = {
		.ap = {
			.ssid = { 0 },
			.password = { 0 },
			.channel = CONFIG_BK_WLAN_CHANNEL,
			.authmode = WIFI_AUTH_WPA2_PSK,
			.max_connection = 8,
			.beacon_interval = 500
		}
	};
	snprintf((char *)ap_cfg.ap.ssid, 32, "blinkekatze_"MACSTR, MAC2STR(ap_mac_address));
	generate_psk(ap_password, WIRELESS_AP_PASSWORD_LENGTH);
	memcpy(ap_cfg.ap.password, ap_password, WIRELESS_AP_PASSWORD_LENGTH);
	err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
	if (err) {
		return err;
	}

	err = esp_wifi_start();
	if (err) {
		return err;
	}

	err = esp_wifi_set_channel(CONFIG_BK_WLAN_CHANNEL, WIFI_SECOND_CHAN_NONE);
	if (err) {
		return err;
	}

	err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
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
		.channel = CONFIG_BK_WLAN_CHANNEL,
		.ifidx = WIFI_IF_AP,
		.encrypt = false,
	};
	memcpy(bcast_peer.peer_addr, wireless_broadcast_address, ESP_NOW_ETH_ALEN);
	err = esp_now_add_peer(&bcast_peer);
	if (err) {
		ESP_LOGE(TAG, "Failed to add broadcast peer: %d", err);
		return err;
	}

	const uint8_t *default_wireless_encryption_key = EMBEDDED_FILE_PTR(wireless_key);
	memcpy(wireless_encryption_key, default_wireless_encryption_key, WIRELESS_ENCRYPTION_KEY_SIZE);
	wireless_random_id = esp_random();
	hmac_ctx_init();

	rx_queue = xQueueCreate(WIRELESS_RX_QUEUE_SIZE, sizeof(wireless_packet_t));
	if (!rx_queue) {
		return ESP_ERR_NO_MEM;
	}

	return esp_now_register_recv_cb(recv_cb);
}

static esp_err_t broadcast_encrypted(const uint8_t *data, size_t len) {
	union {
		struct {
			wireless_packet_hdr_t hdr;
			uint8_t payload[WIRELESS_MAX_PACKET_SIZE];
		};
		uint8_t data[sizeof(wireless_packet_hdr_t) + WIRELESS_MAX_PACKET_SIZE];
	} packet;
	packet.hdr.nonce.timestamp = (uint64_t)neighbour_get_global_clock() >> 10;
	packet.hdr.nonce.packet_cnt = wireless_packet_tx_cnt++;
	packet.hdr.nonce.random_id = wireless_random_id;

	packet_crypt(packet.payload, data, len, &packet.hdr);

	uint8_t hmac[20];
	packet_generate_hmac(hmac, data, len);
	memcpy(packet.hdr.short_hmac, hmac, sizeof(packet.hdr.short_hmac));

	return esp_now_send(wireless_broadcast_address, packet.data, sizeof(wireless_packet_hdr_t) + len);
}

esp_err_t wireless_broadcast(const uint8_t *data, size_t len) {
	if (wireless_encryption_enabled) {
		return broadcast_encrypted(data, len);
	} else {
		return esp_now_send(wireless_broadcast_address, data, len);
	}
}

QueueHandle_t wireless_get_rx_queue() {
	return rx_queue;
}

esp_err_t wireless_scan_aps(void) {
	wifi_scan_config_t scan_cfg = {
		.channel = CONFIG_BK_WLAN_CHANNEL,
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

const char *wireless_get_ap_password() {
	return ap_password;
};

esp_err_t wireless_connect_to_ap(wifi_config_t *sta_cfg) {
	sta_cfg->sta.scan_method = WIFI_FAST_SCAN;
	sta_cfg->sta.bssid_set = false;
	sta_cfg->sta.channel = CONFIG_BK_WLAN_CHANNEL;

	esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, sta_cfg);
	if (err) {
		ESP_LOGE(TAG, "Failed to configure station interface: %d", err);
		return err;
	}

	wireless_disconnect_from_ap();
	return esp_wifi_connect();
}

esp_err_t wireless_disconnect_from_ap() {
	esp_err_t err = esp_wifi_disconnect();
	sta_connected = false;
	return err;
}

int wireless_get_ap_ifindex() {
	return esp_netif_get_netif_impl_index(ap_netif);
}

int wireless_get_sta_ifindex() {
	return esp_netif_get_netif_impl_index(sta_netif);
}

bool wireless_is_sta_connected() {
	return sta_connected;
}

const uint8_t *wireless_get_mac_address() {
	return ap_mac_address;
}

const uint8_t *wireless_get_broadcast_address() {
	return wireless_broadcast_address;
}

bool wireless_is_broadcast_address(const uint8_t *addr) {
	return !memcmp(addr, wireless_broadcast_address, sizeof(wireless_broadcast_address));
}

bool wireless_is_local_address(const uint8_t *addr) {
	return !memcmp(addr, ap_mac_address, sizeof(ap_mac_address));
}

void wireless_set_encryption_enable(bool enable) {
	wireless_encryption_enabled = enable;
}

void wireless_set_replay_protection_enable(bool enable) {
	replay_protection_enabled = enable;
}

esp_err_t wireless_set_encryption_key(const uint8_t *key, unsigned int len) {
	if (len != WIRELESS_ENCRYPTION_KEY_SIZE) {
		return ESP_ERR_INVALID_ARG;
	}
	memcpy(wireless_encryption_key, key, len);
	hmac_ctx_reinit();
	return ESP_OK;
}
