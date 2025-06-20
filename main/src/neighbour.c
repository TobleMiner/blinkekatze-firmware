#include "neighbour.h"

#include <stdlib.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_mac.h>
#include <esp_timer.h>

#include "ota.h"
#include "scheduler.h"
#include "status_leds.h"
#include "util.h"
#include "wireless.h"

#define NEIGHBOUR_TTL_US			30000000UL
#define NEIGHBOUR_ADV_INTERVAL_US		 3000000UL
#define NEIGHBOUR_RSSI_REPORT_INTERVAL_US	10000000UL
#define NEIGHBOUR_MAX_RSSI_REPORTS			64
#define NEIGHBOUR_RSSI_REPORT_ALLOCATION_BLOCK_SIZE	 8
#define NEIGHBOUR_MAX_REPORTS_PER_PACKET		 8

static const char *TAG = "neighbour";

typedef struct neighbours {
	int64_t last_adv_timestamp;
	int64_t local_to_global_time_offset;
	list_head_t neighbours;
	scheduler_task_t housekeeping_task;
	neighbour_t *clock_source;
	portMUX_TYPE lock;
	StaticSemaphore_t rssi_report_lock_buffer;
	SemaphoreHandle_t rssi_report_lock;
	unsigned int rssi_report_index;
	int64_t last_rssi_report_timestamp;
} neighbours_t;

static neighbours_t neighbours;

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
	int64_t global_clock = now + neighbours.local_to_global_time_offset;

	if (neighbours.clock_source) {
		global_clock = get_global_clock_us(neighbours.clock_source, now);
	}
	if (src) {
		*src = neighbours.clock_source;
	}
	return global_clock;
}

static void update_clock_source(void) {
	int64_t now = esp_timer_get_time();
	int64_t max_uptime = now;
	neighbour_t *neigh;
	neighbour_t *neigh_src = NULL;
	LIST_FOR_EACH_ENTRY(neigh, &neighbours.neighbours, list) {
		int64_t uptime = get_uptime_us(neigh, now);
		if (uptime > max_uptime) {
			max_uptime = uptime;
			neigh_src = neigh;
		}
	}
	neighbours.clock_source = neigh_src;
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
		taskENTER_CRITICAL(&neighbours.lock);
		LIST_APPEND(&neigh->list, &neighbours.neighbours);
		taskEXIT_CRITICAL(&neighbours.lock);
	}

	neigh->last_local_adv_rx_timestamp_us = timestamp_us;
	neigh->last_advertisement = *adv;
	update_clock_source();

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

static neighbour_rssi_info_t *neighbour_find_rssi_report(const neighbour_t *neigh, const uint8_t *address) {
	for (unsigned i = 0; i < neigh->num_rssi_reports; i++) {
		neighbour_rssi_info_t *report = &neigh->neighbour_rssi_reports[i];
		if (!memcmp(address, report->address, sizeof(report->address))) {
			return report;
		}
	}

	return NULL;
}

esp_err_t neighbour_add_rssi_report(neighbour_t *neigh, const neighbour_rssi_info_t *new_report) {
	const uint8_t *bcast_addr = wireless_get_broadcast_address();

	// Try finding an entry that is not used anymore
	for (unsigned i = 0; i < neigh->num_rssi_reports; i++) {
		neighbour_rssi_info_t *report = &neigh->neighbour_rssi_reports[i];

		if (wireless_is_broadcast_address(report->address)) {
			ESP_LOGD(TAG, "Storing new RSSI report in preallocated entry");
			memcpy(report->address, new_report->address, sizeof(report->address));
			report->rssi = new_report->rssi;
			return ESP_OK;
		}
	}

	// No free entry in list, need to expand allocation
	if (neigh->num_rssi_reports >= NEIGHBOUR_MAX_RSSI_REPORTS) {
		return ESP_ERR_NO_MEM;
	}

	unsigned int new_num_rssi_reports = neigh->num_rssi_reports +
					    NEIGHBOUR_RSSI_REPORT_ALLOCATION_BLOCK_SIZE;
	// Limit number of stored RSSI reports to a reasonable maximum
	if (new_num_rssi_reports > NEIGHBOUR_MAX_RSSI_REPORTS) {
		new_num_rssi_reports = NEIGHBOUR_MAX_RSSI_REPORTS;
	}

	// Try allocating a new set of rssi reports
	neighbour_rssi_info_t *reallocated_reports = realloc(neigh->neighbour_rssi_reports, sizeof(neighbour_rssi_info_t) * (size_t)new_num_rssi_reports);
	if (!reallocated_reports) {
		return ESP_ERR_NO_MEM;
	}
	neigh->neighbour_rssi_reports = reallocated_reports;

	// Store new report
	ESP_LOGD(TAG, "Storing new RSSI report in reallocated array");
	neighbour_rssi_info_t *report = &neigh->neighbour_rssi_reports[neigh->num_rssi_reports];
	memcpy(report->address, new_report->address, sizeof(report->address));
	report->rssi = new_report->rssi;

	// Initialize trailing reports
	for (unsigned int i = neigh->num_rssi_reports + 1; i < new_num_rssi_reports; i++) {
		neighbour_rssi_info_t *report = &neigh->neighbour_rssi_reports[i];
		memcpy(report->address, bcast_addr, sizeof(report->address));
	}
	neigh->num_rssi_reports = new_num_rssi_reports;

	return ESP_OK;
}

esp_err_t neighbour_update_rssi_reports(const uint8_t *address, const neighbour_rssi_info_packet_t *info, const void *payload) {
	const neighbour_rssi_info_t *reports = payload;
	neighbour_t *neigh = find_neighbour(address);

	if (!neigh) {
		return ESP_OK;
	}

	for (unsigned int i = 0; i < info->num_rssi_reports; i++) {
		const neighbour_rssi_info_t *report = &reports[i];
		if (wireless_is_broadcast_address(report->address)) {
			continue;
		}

		neighbour_rssi_info_t *old_report = neighbour_find_rssi_report(neigh, report->address);
		if (old_report) {
			ESP_LOGD(TAG, "Updating old RSSI report");
			old_report->rssi = report->rssi;
		} else if (find_neighbour(report->address) || wireless_is_local_address(report->address)) {
			esp_err_t err = neighbour_add_rssi_report(neigh, report);
			if (err) {
				return err;
			}
		}
	}

	return ESP_OK;
}

esp_err_t neighbour_rx_rssi_info(const wireless_packet_t *packet) {
	neighbour_rssi_info_packet_t info;
	if (packet->len < sizeof(info)) {
		ESP_LOGD(TAG, "Got short advertisement rssi, expected %u bytes but got only %u bytes",
			 sizeof(info), packet->len);
		return ESP_ERR_INVALID_ARG;
	}
	memcpy(&info, packet->data, sizeof(info));

	size_t min_size = sizeof(info) + (size_t)info.num_rssi_reports * sizeof(neighbour_rssi_info_t);
	if (packet->len < min_size) {
		ESP_LOGD(TAG, "Got internally inconsistent RSSI packet, expected %u bytes but got only %u bytes",
			 min_size, packet->len);
		return ESP_ERR_INVALID_ARG;
	}

	ESP_LOGD(TAG, "Received RSSI report with %u RSSI entries", info.num_rssi_reports);
	if (xSemaphoreTake(neighbours.rssi_report_lock, 1)) {
		esp_err_t err = neighbour_update_rssi_reports(packet->src_addr, &info, packet->data + sizeof(info));
		xSemaphoreGive(neighbours.rssi_report_lock);
		return err;
	}

	return ESP_OK;
}

static esp_err_t send_next_rssi_report(void) {
	unsigned int reports_in_packet = 0;
	struct {
		neighbour_rssi_info_packet_t info;
		neighbour_rssi_info_t rssi_reports[NEIGHBOUR_MAX_REPORTS_PER_PACKET];
	} __attribute__((packed)) full_rssi_report_packet;
	neighbour_t *neigh;
	unsigned int i = 0;
	unsigned int num_neighbours = 0;
	LIST_FOR_EACH_ENTRY(neigh, &neighbours.neighbours, list) {
		if (i++ >= neighbours.rssi_report_index &&
		    reports_in_packet < NEIGHBOUR_MAX_REPORTS_PER_PACKET) {
			neighbour_rssi_info_t *report = &full_rssi_report_packet.rssi_reports[reports_in_packet];
			memcpy(report->address, neigh->address, sizeof(report->address));
			report->rssi = neigh->rssi;
			reports_in_packet++;
		}
		num_neighbours++;
	}
	// Loop around, filling any empty spots
	LIST_FOR_EACH_ENTRY(neigh, &neighbours.neighbours, list) {
		if (reports_in_packet >= NEIGHBOUR_MAX_REPORTS_PER_PACKET) {
			break;
		}
		neighbour_rssi_info_t *report = &full_rssi_report_packet.rssi_reports[reports_in_packet];
		memcpy(report->address, neigh->address, sizeof(report->address));
		report->rssi = neigh->rssi;
		reports_in_packet++;
	}

	ESP_LOGD(TAG, "Sending RSSI report with %u RSSI entries", num_neighbours);

	// Fill in packet metadata
	full_rssi_report_packet.info.packet_type = WIRELESS_PACKET_TYPE_NEIGHBOUR_RSSI_REPORT;
	full_rssi_report_packet.info.num_rssi_reports = num_neighbours;

	// Send the report
	esp_err_t err = wireless_broadcast((const uint8_t *)&full_rssi_report_packet,
					   sizeof(full_rssi_report_packet.info) +
					   sizeof(neighbour_rssi_info_t) * (size_t)reports_in_packet);
	if (err) {
		return err;
	}

	neighbours.rssi_report_index += reports_in_packet;
	neighbours.rssi_report_index %= num_neighbours;

	return ESP_OK;
}

static void neighbour_housekeeping(void *priv) {
	int64_t now = esp_timer_get_time();
	int64_t global_clock = get_global_clock(now, NULL);
	neighbour_t *neigh;
	list_head_t *next;
	neighbours.local_to_global_time_offset = global_clock - now;
	LIST_FOR_EACH_ENTRY_SAFE(neigh, next, &neighbours.neighbours, list) {
		int64_t age = now - neigh->last_local_adv_rx_timestamp_us;
		if (age > NEIGHBOUR_TTL_US) {
			ESP_LOGI(TAG, "Neighbour "MACSTR" TTL exceeded, deleting", MAC2STR(neigh->address));
			taskENTER_CRITICAL(&neighbours.lock);
			LIST_DELETE(&neigh->list);
			if (neigh == neighbours.clock_source) {
				neighbours.clock_source = NULL;
				update_clock_source();
			}
			taskEXIT_CRITICAL(&neighbours.lock);
			free(neigh->neighbour_rssi_reports);
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

	if (now - neighbours.last_rssi_report_timestamp >= NEIGHBOUR_RSSI_REPORT_INTERVAL_US ||
	    !neighbours.last_rssi_report_timestamp) {
		esp_err_t err = send_next_rssi_report();
		if (err) {
			ESP_LOGW(TAG, "Failed to send rssi report: %d", err);
		}
		neighbours.last_rssi_report_timestamp = esp_timer_get_time();
	}

	if (neighbour_has_neighbours()) {
		status_led_set_mode(STATUS_LED_GREEN, STATUS_LED_MODE_ON);
	} else {
		status_led_set_blink(STATUS_LED_GREEN, 1000);
	}

	scheduler_schedule_task_relative(&neighbours.housekeeping_task, neighbour_housekeeping, NULL, MS_TO_US(2000));
}

void neighbour_init() {
	neighbours.last_adv_timestamp = 0;
	neighbours.local_to_global_time_offset = 0;
	neighbours.lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
	neighbours.rssi_report_lock = xSemaphoreCreateMutexStatic(&neighbours.rssi_report_lock_buffer);
	INIT_LIST_HEAD(neighbours.neighbours);
	scheduler_task_init(&neighbours.housekeeping_task);
	scheduler_schedule_task_relative(&neighbours.housekeeping_task, neighbour_housekeeping, NULL, MS_TO_US(0));
	neighbours.rssi_report_index = 0;
}

int64_t neighbour_get_global_clock_and_source(neighbour_t **src) {
	int64_t now = esp_timer_get_time();
	return get_global_clock(now, src);
}

int64_t neighbour_get_global_clock() {
	taskENTER_CRITICAL(&neighbours.lock);
	int64_t clock = neighbour_get_global_clock_and_source(NULL);
	taskEXIT_CRITICAL(&neighbours.lock);
	return clock;
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
	printf("Address             Platform         Uptime        Age       RSSI    SoC     Time to empty   Firmware hash   Firmware version   OTA status\r\n");
	printf("===========================================================================================================================================\r\n");
	//      aa:bb:cc:dd:ee:ff   tallylight_v2    xxxxxxxxms    xxxxxms  -90dBm   100%   65535min        <short hash>    <firmware version>
	neighbour_t *neigh;
	int64_t now = esp_timer_get_time();
	unsigned int num_neighbours = 0;
	LIST_FOR_EACH_ENTRY(neigh, &neighbours.neighbours, list) {
		num_neighbours++;
		bool neigh_status_valid = !!neigh->last_status.packet_type;
		uint64_t age_ms = now - neigh->last_local_adv_rx_timestamp_us;
		bool static_strs_valid = !!neigh->last_static_info.packet_type;
		const char *firmware_str = neigh->last_static_info.firmware_version;
		const char *platform_name = neigh->last_static_info.platform_name;
		char ota_status[32];
		ota_neighbour_info_to_string(&neigh->last_ota_info, ota_status, sizeof(ota_status));
		if (neigh_status_valid) {
			char firmware_hash_str[12 + 1] = { 0 };
			hex_encode(neigh->last_static_info.firmware_sha256_hash, sizeof(neigh->last_static_info.firmware_sha256_hash),
				   firmware_hash_str, sizeof(firmware_hash_str) - 1);
			if (neigh->rssi == 0) {
				printf(MACSTR"   %-13.*s   %9lums   %5lums     0dBm   %3d%%   %5dmin         %-13s   %-16.*s   %s\r\n",
				       MAC2STR(neigh->address),
				       static_strs_valid ? strnlen(platform_name, sizeof(neigh->last_static_info.platform_name)) : 3,
				       static_strs_valid ? platform_name : "???",
				       (unsigned long)(get_uptime_us(neigh, now) / 1000ULL),
				       (unsigned long)(age_ms / 1000ULL),
				       neigh->last_status.battery_soc_percent,
				       neigh->last_status.battery_time_to_empty_min,
				       firmware_hash_str,
				       static_strs_valid ? strnlen(firmware_str, sizeof(neigh->last_static_info.firmware_version)) : 3,
				       static_strs_valid ? firmware_str : "???",
				       ota_status);
			} else {
				printf(MACSTR"   %-13.*s   %9lums   %5lums   %2ddBm   %3d%%   %5dmin         %-13s   %-16.*s   %s\r\n",
				       MAC2STR(neigh->address),
				       static_strs_valid ? strnlen(platform_name, sizeof(neigh->last_static_info.platform_name)) : 3,
				       static_strs_valid ? platform_name : "???",
				       (unsigned long)(get_uptime_us(neigh, now) / 1000ULL),
				       (unsigned long)(age_ms / 1000ULL),
				       neigh->rssi,
				       neigh->last_status.battery_soc_percent,
				       neigh->last_status.battery_time_to_empty_min,
				       firmware_hash_str,
				       static_strs_valid ? strnlen(firmware_str, sizeof(neigh->last_static_info.firmware_version)) : 3,
				       static_strs_valid ? firmware_str : "???",
				       ota_status);
			}
		} else {
			if (neigh->rssi == 0) {
				printf(MACSTR"   %-13.*s   %9lums   %5lums     0dBm   ???       ???           ???             %-16.*s   %s\r\n",
				       MAC2STR(neigh->address),
				       static_strs_valid ? strnlen(platform_name, sizeof(neigh->last_static_info.platform_name)) : 3,
				       static_strs_valid ? platform_name : "???",
				       (unsigned long)(get_uptime_us(neigh, now) / 1000ULL),
				       (unsigned long)(age_ms / 1000ULL),
				       static_strs_valid ? strnlen(firmware_str, sizeof(neigh->last_static_info.firmware_version)) : 3,
				       static_strs_valid ? firmware_str : "???",
				       ota_status);
			} else {
				printf(MACSTR"   %-13.*s   %9lums   %5lums   %2ddBm   ???       ???           ???             %-16.*s   %s\r\n",
				       MAC2STR(neigh->address),
				       static_strs_valid ? strnlen(platform_name, sizeof(neigh->last_static_info.platform_name)) : 3,
				       static_strs_valid ? platform_name : "???",
				       (unsigned long)(get_uptime_us(neigh, now) / 1000ULL),
				       (unsigned long)(age_ms / 1000ULL),
				       neigh->rssi,
				       static_strs_valid ? strnlen(firmware_str, sizeof(neigh->last_static_info.firmware_version)) : 3,
				       static_strs_valid ? firmware_str : "???",
				       ota_status);

			}
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

unsigned int neighbour_take_rssi_reports(const neighbour_t *neigh, neighbour_rssi_info_t **rssi_reports) {
	xSemaphoreTake(neighbours.rssi_report_lock, portMAX_DELAY);
	*rssi_reports = neigh->neighbour_rssi_reports;
	return neigh->num_rssi_reports;
}

void neighbour_put_rssi_reports(void) {
	xSemaphoreGive(neighbours.rssi_report_lock);
}
