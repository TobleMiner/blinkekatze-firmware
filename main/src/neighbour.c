#include "neighbour.h"

#include <stdlib.h>
#include <stdio.h>

#include <esp_log.h>
#include <esp_mac.h>
#include <esp_timer.h>

#include "ota.h"
#include "status_leds.h"
#include "util.h"
#include "wireless.h"

#define NEIGHBOUR_TTL_US		10000000UL
#define NEIGHBOUR_ADV_INTERVAL_US	 3000000UL

static const char *TAG = "neighbour";

typedef struct neighbours {
	int64_t last_adv_timestamp;
	int64_t local_to_global_time_offset;
	list_head_t neighbours;
} neighbours_t;

static neighbours_t neighbours;

void neighbour_init() {
	neighbours.last_adv_timestamp = 0;
	neighbours.local_to_global_time_offset = 0;
	INIT_LIST_HEAD(neighbours.neighbours);
}

static neighbour_t *find_neighbour(const uint8_t *address) {
	neighbour_t *neigh;
	LIST_FOR_EACH_ENTRY(neigh, &neighbours.neighbours, list) {
		if (!memcmp(address, neigh->address, sizeof(neigh->address))) {
			return neigh;
		}
	}

	return NULL;
}

const neighbour_t *neighbour_find_by_address(const uint8_t *address) {
	return find_neighbour(address);
}

static int64_t get_uptime_us(const neighbour_t *neigh, int64_t now) {
	int64_t offset = now - neigh->last_local_adv_rx_timestamp_us;

	return neigh->last_advertisement.uptime_us + offset;
}

static int64_t get_global_clock_us(const neighbour_t *neigh, int64_t now) {
	int64_t offset = now - neigh->last_local_adv_rx_timestamp_us;

	return neigh->last_advertisement.global_clock_us + offset;
}

static int64_t get_global_clock(int64_t now, neighbour_t **src) {
	int64_t max_uptime = now;
	int64_t global_clock = now + neighbours.local_to_global_time_offset;
	neighbour_t *neigh_src = NULL;

	neighbour_t *neigh;
	LIST_FOR_EACH_ENTRY(neigh, &neighbours.neighbours, list) {
		int64_t uptime = get_uptime_us(neigh, now);
		if (uptime > max_uptime) {
			global_clock = get_global_clock_us(neigh, now);
			max_uptime = uptime;
			neigh_src = neigh;
		}
	}

	if (src) {
		*src = neigh_src;
	}
	return global_clock;
}

esp_err_t neighbour_update(const uint8_t *address, int64_t timestamp_us, const neighbour_advertisement_t *adv) {
	neighbour_t *neigh = find_neighbour(address);
	if (!neigh) {
		ESP_LOGI(TAG, "New neighbour "MACSTR, MAC2STR(address));
		neigh = calloc(1, sizeof(neighbour_t));
		if (!neigh) {
			return ESP_ERR_NO_MEM;
		}
		INIT_LIST_HEAD(neigh->list);
		memcpy(neigh->address, address, sizeof(neigh->address));
		neigh->local_to_remote_time_offset = 0;
		LIST_APPEND(&neigh->list, &neighbours.neighbours);
	}

	neigh->last_local_adv_rx_timestamp_us = timestamp_us;
	neigh->last_advertisement = *adv;

	return ESP_OK;
}

esp_err_t neighbour_rx(const wireless_packet_t *packet) {
	neighbour_advertisement_t adv;
	if (packet->len < sizeof(adv)) {
		ESP_LOGD(TAG, "Got short advertisement packet, expected %u bytes but got only %u bytes",
			 sizeof(adv), packet->len);
		return ESP_ERR_INVALID_ARG;
	}
	memcpy(&adv, packet->data, sizeof(adv));

	return neighbour_update(packet->src_addr, packet->rx_timestamp, &adv);
}

void neighbour_housekeeping() {
	int64_t now = esp_timer_get_time();
	int64_t global_clock = get_global_clock(now, NULL);
	neighbour_t *neigh;
	list_head_t *next;
	neighbours.local_to_global_time_offset = global_clock - now;
	LIST_FOR_EACH_ENTRY_SAFE(neigh, next, &neighbours.neighbours, list) {
		int64_t age = now - neigh->last_local_adv_rx_timestamp_us;
		if (age > NEIGHBOUR_TTL_US) {
			ESP_LOGI(TAG, "Neighbour "MACSTR" TTL exceeded, deleting", MAC2STR(neigh->address));
			LIST_DELETE(&neigh->list);
			free(neigh);
		} else {
			neigh->local_to_remote_time_offset = now - get_uptime_us(neigh, now);
		}
	}

	if (now - neighbours.last_adv_timestamp >= NEIGHBOUR_ADV_INTERVAL_US || !neighbours.last_adv_timestamp) {
		now = esp_timer_get_time();
		global_clock = get_global_clock(now, NULL);
		neighbour_advertisement_t adv = {
			WIRELESS_PACKET_TYPE_NEIGHBOUR_ADVERTISEMENT,
			now,
			global_clock
		};

		wireless_broadcast((uint8_t *)&adv, sizeof(adv));
		neighbours.last_adv_timestamp = now;
	}

	if (neighbour_has_neighbours()) {
		status_led_set_mode(STATUS_LED_GREEN, STATUS_LED_MODE_ON);
	} else {
		status_led_set_blink(STATUS_LED_GREEN, 1000);
	}
}

int64_t neighbour_get_global_clock_and_source(neighbour_t **src) {
	int64_t now = esp_timer_get_time();
	return get_global_clock(now, src);
}

int64_t neighbour_get_global_clock() {
	return neighbour_get_global_clock_and_source(NULL);
}

esp_err_t neighbour_update_rssi(const uint8_t *address, int rssi) {
	neighbour_t *neigh = find_neighbour(address);
	if (!neigh) {
		return ESP_ERR_NOT_FOUND;
	}

	ESP_LOGD(TAG, "Updating RSSI of neighbour "MACSTR" to %d", MAC2STR(neigh->address), rssi);
	neigh->rssi = rssi;
	return ESP_OK;
}

int64_t neighbour_remote_to_local_time(const neighbour_t *neigh, int64_t remote_timestamp) {
	if (!neigh) {
		return remote_timestamp;
	}

	return remote_timestamp + neigh->local_to_remote_time_offset;
}

int64_t neighbour_get_uptime(const neighbour_t *neigh) {
	return get_uptime_us(neigh, esp_timer_get_time());
}

bool neighbour_has_neighbours(void) {
	return !LIST_IS_EMPTY(&neighbours.neighbours);
}

void neighbour_print_list(void) {
	printf("Address             Uptime       Age      RSSI     SoC    Time to empty   Firmware hash   Firmware version   OTA status\r\n");
	printf("=======================================================================================================================\r\n");
	//      aa:bb:cc:dd:ee:ff   xxxxxxxxms   xxxxms   -90dBm   100%   65535min        <short hash>    <firmware version>
	neighbour_t *neigh;
	int64_t now = esp_timer_get_time();
	unsigned int num_neighbours = 0;
	LIST_FOR_EACH_ENTRY(neigh, &neighbours.neighbours, list) {
		num_neighbours++;
		bool neigh_status_valid = !!neigh->last_status.packet_type;
		uint64_t age_ms = now - neigh->last_local_adv_rx_timestamp_us;
		bool firmware_str_valid = !!neigh->last_static_info.packet_type;
		const char *firmware_str = neigh->last_static_info.firmware_version;
		char ota_status[32];
		ota_neighbour_info_to_string(&neigh->last_ota_info, ota_status, sizeof(ota_status));
		if (neigh_status_valid) {
			char firmware_hash_str[12 + 1] = { 0 };
			hex_encode(neigh->last_static_info.firmware_sha256_hash, sizeof(neigh->last_static_info.firmware_sha256_hash),
				   firmware_hash_str, sizeof(firmware_hash_str) - 1);
			printf(MACSTR"   %8lums   %4lums   %2ddBm   %3d%%   %5dmin        %-13s   %-16.*s   %s\r\n",
			       MAC2STR(neigh->address),
			       (unsigned long)(get_uptime_us(neigh, now) / 1000ULL),
			       (unsigned long)(age_ms / 1000ULL),
			       neigh->rssi,
			       neigh->last_status.battery_soc_percent,
			       neigh->last_status.battery_time_to_empty_min,
			       firmware_hash_str,
			       firmware_str_valid ? strnlen(firmware_str, sizeof(neigh->last_static_info.firmware_version)) : 3,
			       firmware_str_valid ? firmware_str : "???",
			       ota_status);
		} else {
			printf(MACSTR"   %8lums   %4lums   %2ddBm   ???       ???         ???            %-16.*s   %s\r\n",
			       MAC2STR(neigh->address),
			       (unsigned long)(get_uptime_us(neigh, now) / 1000ULL),
			       (unsigned long)(age_ms / 1000ULL),
			       neigh->rssi,
			       firmware_str_valid ? strnlen(firmware_str, sizeof(neigh->last_static_info.firmware_version)) : 3,
			       firmware_str_valid ? firmware_str : "???",
			       ota_status);
		}
	}
	printf("Have %u neigbours\r\n", num_neighbours);
}

void neighbour_update_status(const neighbour_t *neigh, const neighbour_status_packet_t *status) {
	neighbour_t *neigh_ = (neighbour_t *)neigh;
	neigh_->last_status = *status;
}

void neighbour_update_static_info(const neighbour_t *neigh, const neighbour_static_info_packet_t *static_info) {
	neighbour_t *neigh_ = (neighbour_t *)neigh;
	neigh_->last_static_info = *static_info;
}

void neighbour_update_ota_info(const neighbour_t *neigh, const neighbour_ota_info_t *ota_info) {
	neighbour_t *neigh_ = (neighbour_t *)neigh;
	neigh_->last_ota_info = *ota_info;
}

int8_t neighbour_get_rssi(const neighbour_t *neigh) {
	return neigh->rssi;
}
