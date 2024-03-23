#include "neighbour.h"

#include <stdlib.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_mac.h>
#include <esp_random.h>
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
#define NEIGHBOUR_NODE_ID_TLB_ALLOCATION_BLOCK_SIZE	 8

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
	unsigned int local_id_tlb_size;
	neighbour_t **local_id_tlb;
} neighbours_t;

static neighbours_t neighbours;

#define LOCAL_NODE_ID_SELF 1
#define LOCAL_NODE_ID_FIRST 2
#define LOCAL_NODE_ID_TO_TLB_INDEX(x_) ((x_) - LOCAL_NODE_ID_FIRST)
#define TLB_INDEX_TO_LOCAL_NODE_ID(x_) ((x_) + LOCAL_NODE_ID_FIRST)

static inline neighbour_t *get_tlb_entry_by_local_node_id(unsigned int id) {
	return neighbours.local_id_tlb[LOCAL_NODE_ID_TO_TLB_INDEX(id)];
}

static inline void set_tlb_entry_by_local_node_id(unsigned int id, neighbour_t *neigh) {
	neighbours.local_id_tlb[LOCAL_NODE_ID_TO_TLB_INDEX(id)] = neigh;
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

static unsigned int find_next_local_node_id(void) {
	unsigned int tlb_idx = 0;
	for (; tlb_idx < neighbours.local_id_tlb_size; tlb_idx++) {
		if (!neighbours.local_id_tlb[tlb_idx]) {
			break;
		}
	}
	return TLB_INDEX_TO_LOCAL_NODE_ID(tlb_idx);
}

static esp_err_t allocate_local_node_id_tlb_entry(unsigned int id, neighbour_t *neigh) {
	unsigned int tlb_idx = LOCAL_NODE_ID_TO_TLB_INDEX(id);
	if (tlb_idx >= neighbours.local_id_tlb_size) {
		unsigned int new_local_tlb_size = ALIGN_UP(tlb_idx + 1, NEIGHBOUR_NODE_ID_TLB_ALLOCATION_BLOCK_SIZE);
		neighbour_t **new_tlb = realloc(neighbours.local_id_tlb, new_local_tlb_size * sizeof(neighbour_t *));
		if (!new_tlb) {
			return ESP_ERR_NO_MEM;
		}
		for (unsigned int i = neighbours.local_id_tlb_size; i < new_local_tlb_size; i++) {
			new_tlb[i] = NULL;
		}
		neighbours.local_id_tlb = new_tlb;
		neighbours.local_id_tlb_size = new_local_tlb_size;
	}
	ESP_LOGV(TAG, "Storing neighbour at TLB index %u", tlb_idx);
	neighbours.local_id_tlb[tlb_idx] = neigh;
	return ESP_OK;
}

static esp_err_t add_new_neighbour(neighbour_t **neigh_ret, const uint8_t *address) {
	neighbour_t *neigh = calloc(1, sizeof(neighbour_t));
	if (!neigh) {
		return ESP_ERR_NO_MEM;
	}
	INIT_LIST_HEAD(neigh->list);
	memcpy(neigh->address, address, sizeof(neigh->address));
	neigh->location.x = esp_random() >> (32 - 1 - FIXEDPOINT_BASE);
	neigh->location.y = esp_random() >> (32 - 1 - FIXEDPOINT_BASE);
	neigh->location.z = esp_random() >> (32 - 1 - FIXEDPOINT_BASE);
	unsigned int local_node_id = find_next_local_node_id();
	ESP_LOGD(TAG, "Assigning local node id %u to "MACSTR, local_node_id, MAC2STR(address));
	esp_err_t err = allocate_local_node_id_tlb_entry(local_node_id, neigh);
	if (err) {
		free(neigh);
		return err;
	}
	neigh->local_node_id = local_node_id;
	taskENTER_CRITICAL(&neighbours.lock);
	LIST_APPEND(&neigh->list, &neighbours.neighbours);
	taskEXIT_CRITICAL(&neighbours.lock);

	*neigh_ret = neigh;
	return ESP_OK;
}

esp_err_t neighbour_update(const uint8_t *address, int64_t timestamp_us, const neighbour_advertisement_t *adv) {
	neighbour_t *neigh = find_neighbour(address);
	if (!neigh) {
		ESP_LOGI(TAG, "New neighbour "MACSTR, MAC2STR(address));
		esp_err_t err = add_new_neighbour(&neigh, address);
		if (err) {
			ESP_LOGW(TAG, "Failed to add neighbour: %d", err);
			return err;
		}
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
	bool is_local_address = wireless_is_local_address(address);

	ESP_LOGV(TAG, "Looking up RSSI report for "MACSTR" on "MACSTR, MAC2STR(address), MAC2STR(neigh->address));

	for (unsigned i = 0; i < neigh->num_rssi_reports; i++) {
		neighbour_rssi_info_t *report = &neigh->neighbour_rssi_reports[i];
		ESP_LOGV(TAG, "Report %u has local node id %u", i, report->local_node_id);
		if (report->local_node_id) {
			if (report->local_node_id == LOCAL_NODE_ID_SELF) {
				if (is_local_address) {
					return report;
				}
			} else {
				neighbour_t *report_neigh = get_tlb_entry_by_local_node_id(report->local_node_id);
				if (report_neigh) {
					ESP_LOGV(TAG, "Report %u has address "MACSTR, i, MAC2STR(report_neigh->address));
					if (!memcmp(address, report_neigh->address, sizeof(report_neigh->address))) {
						return report;
					}
				} else {
					ESP_LOGW(TAG, "Neighbour for local node id %u not found. This should not happen.", report->local_node_id);
				}
			}
		}
	}

	return NULL;
}

esp_err_t neighbour_add_rssi_report(neighbour_t *neigh, neighbour_t *report_neigh, int rssi) {
	unsigned int local_node_id = LOCAL_NODE_ID_SELF;
	if (report_neigh) {
		local_node_id = report_neigh->local_node_id;
		ESP_LOGD(TAG, "Storing RSSI report from "MACSTR" for "MACSTR" (%u)", MAC2STR(neigh->address), MAC2STR(report_neigh->address), local_node_id);
	} else {
		ESP_LOGD(TAG, "Storing RSSI report from "MACSTR" for self (%u)", MAC2STR(neigh->address), local_node_id);
	}

	// Try finding an entry that is not used
	for (unsigned i = 0; i < neigh->num_rssi_reports; i++) {
		neighbour_rssi_info_t *report = &neigh->neighbour_rssi_reports[i];

		if (!report->local_node_id) {
			ESP_LOGV(TAG, "Storing new RSSI report in preallocated entry");
			report->local_node_id = local_node_id;
			report->rssi = rssi;
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
	report->local_node_id = local_node_id;
	report->rssi = rssi;

	// Initialize trailing reports
	for (unsigned int i = neigh->num_rssi_reports + 1; i < new_num_rssi_reports; i++) {
		neighbour_rssi_info_t *report = &neigh->neighbour_rssi_reports[i];
		report->local_node_id = 0;
	}
	neigh->num_rssi_reports = new_num_rssi_reports;

	return ESP_OK;
}

esp_err_t neighbour_update_rssi_reports(const uint8_t *address, const neighbour_rssi_info_packet_t *info, const void *payload) {
	const neighbour_rssi_packet_info_t *reports = payload;
	neighbour_t *neigh = find_neighbour(address);

	if (!neigh) {
		return ESP_OK;
	}

	for (unsigned int i = 0; i < info->num_rssi_reports; i++) {
		const neighbour_rssi_packet_info_t *report = &reports[i];
		if (wireless_is_broadcast_address(report->address)) {
			continue;
		}

		neighbour_rssi_info_t *old_report = neighbour_find_rssi_report(neigh, report->address);
		if (old_report) {
			ESP_LOGV(TAG, "Updating old RSSI report");
			old_report->rssi = report->rssi;
		} else {
			neighbour_t *report_neigh = find_neighbour(report->address);
			if (report_neigh || wireless_is_local_address(report->address)) {
				esp_err_t err = neighbour_add_rssi_report(neigh, report_neigh, report->rssi);
				if (err) {
					return err;
				}
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
		neighbour_rssi_packet_info_t rssi_reports[NEIGHBOUR_MAX_REPORTS_PER_PACKET];
	} __attribute__((packed)) full_rssi_report_packet;
	neighbour_t *neigh;
	unsigned int i = 0;
	unsigned int num_neighbours = 0;
	LIST_FOR_EACH_ENTRY(neigh, &neighbours.neighbours, list) {
		if (i++ >= neighbours.rssi_report_index &&
		    reports_in_packet < NEIGHBOUR_MAX_REPORTS_PER_PACKET) {
			neighbour_rssi_packet_info_t *report = &full_rssi_report_packet.rssi_reports[reports_in_packet];
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
		neighbour_rssi_packet_info_t *report = &full_rssi_report_packet.rssi_reports[reports_in_packet];
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
					   sizeof(neighbour_rssi_packet_info_t) * (size_t)reports_in_packet);
	if (err) {
		return err;
	}

	neighbours.rssi_report_index += reports_in_packet;
	neighbours.rssi_report_index %= num_neighbours;

	return ESP_OK;
}

static fp_t rssi_distance_metric(int rssi) {
	if (rssi >= 0) {
		return 0;
	}
	fp_t rssi_fp = int_to_fp(-rssi);
	fp_t rssi_scaled = fp_div(rssi_fp, int_to_fp(10));
	return fp_exp(rssi_scaled);
}

static void calculate_force(fp_vec3_t *force_out, const fp_vec3_t *pos_a, const fp_vec3_t *pos_b, fp_t target_distance) {
	// Figure out how far neighbour should be away from us
	fp_vec3_t delta_pos = *pos_b;
	fp_vec3_sub(&delta_pos, pos_a);
	fp_t current_dist = fp_vec3_mag(&delta_pos);
	fp_t delta_dist = target_distance - current_dist;

	// Get normalized direction vector from pos_a to pos_b
	fp_vec3_t dir_a_to_b_norm = delta_pos;
	fp_vec3_div(&dir_a_to_b_norm, fp_vec3_mag(&dir_a_to_b_norm));

	// Turn normalized vector into desired location
	fp_vec3_t ideal_location = dir_a_to_b_norm;
	fp_vec3_mul(&ideal_location, target_distance);
	fp_vec3_add(&ideal_location, pos_a);

	// Calculate force magnitude
	fp_t clamped_dist = delta_dist;
	if (fp_abs(clamped_dist) > int_to_fp(5)) {
		clamped_dist = int_to_fp(5);
	}
	fp_t force_attract = fp_mul(clamped_dist, clamped_dist);

	// Get normalized direction vector from neighbour to ideal location
	fp_vec3_t dir_to_ideal = ideal_location;
	fp_vec3_sub(&dir_to_ideal, pos_b);
	fp_vec3_t dir_to_ideal_norm = dir_to_ideal;
	fp_vec3_div(&dir_to_ideal_norm, fp_vec3_mag(&dir_to_ideal_norm));

	// Scale normalized vector to ideal to obtain force
	fp_vec3_t force = dir_to_ideal_norm;
	fp_vec3_mul(&force, force_attract);

	*force_out = force;
}

static void simulate_links(void) {
	xSemaphoreTake(neighbours.rssi_report_lock, portMAX_DELAY);
	// Simulate direct neighbours
	neighbour_t *neigh;
//	ESP_LOGI(TAG, "====================NEIGH========================");
	LIST_FOR_EACH_ENTRY(neigh, &neighbours.neighbours, list) {
		int rssi = neigh->rssi;
		const neighbour_rssi_info_t *rssi_report = neighbour_find_rssi_report(neigh, wireless_get_mac_address());
		if (rssi_report) {
			rssi = DIV_ROUND(rssi + rssi_report->rssi, 2);
		}

		// Figure out how far neighbour should be away from us
		fp_t ideal_dist = rssi_distance_metric(rssi);
/*
		fp_t current_dist = fp_vec3_mag(&neigh->location);
		fp_t delta_dist = ideal_dist - current_dist;

		// Get normalized direction vector from origin to neighbour
		fp_vec3_t dir_origin_to_neigh = neigh->location;
		fp_vec3_t dir_origin_to_neigh_norm = dir_origin_to_neigh;
		fp_vec3_div(&dir_origin_to_neigh_norm, fp_vec3_mag(&dir_origin_to_neigh_norm));

		// Turn normalized vector into desired location
		fp_vec3_t ideal_location = dir_origin_to_neigh_norm;
		fp_vec3_mul(&ideal_location, ideal_dist);

		// Calculate force magnitude
		fp_t clamped_dist = delta_dist;
		if (clamped_dist > int_to_fp(5)) {
			clamped_dist = int_to_fp(5);
		}
		fp_t force_attract = fp_mul(clamped_dist, clamped_dist);

		// Get normalized direction vector from neighbour to ideal location
		fp_vec3_t dir_to_origin = ideal_location;
		fp_vec3_sub(&dir_to_origin, &neigh->location);
		fp_vec3_t dir_to_origin_norm = dir_to_origin;
		fp_vec3_div(&dir_to_origin_norm, fp_vec3_mag(&dir_to_origin_norm));

		// Scale normalized vector to origin to obtain force
		fp_vec3_t force = dir_to_origin_norm;
		fp_vec3_mul(&force, force_attract);

		ESP_LOGI(TAG, MACSTR" force "FPVEC3STR, MAC2STR(neigh->address), FPVEC32STR(force));
*/
		fp_vec3_t origin = { 0, 0, 0 };
		fp_vec3_t force;
		calculate_force(&force, &origin, &neigh->location, ideal_dist);

//		ESP_LOGI(TAG, MACSTR" force "FPVEC3STR, MAC2STR(neigh->address), FPVEC32STR(force));

		for (unsigned i = 0; i < neigh->num_rssi_reports; i++) {
			neighbour_rssi_info_t *report = &neigh->neighbour_rssi_reports[i];

			if (report->local_node_id && report->local_node_id != LOCAL_NODE_ID_SELF) {
				neighbour_t *neigh_neigh = get_tlb_entry_by_local_node_id(report->local_node_id);
				if (neigh_neigh) {
					fp_t ideal_dist = rssi_distance_metric(report->rssi);

					fp_vec3_t force_neigh;
					calculate_force(&force_neigh, &neigh_neigh->location, &neigh->location, ideal_dist);
//					ESP_LOGI(TAG, MACSTR" force "FPVEC3STR, MAC2STR(neigh->address), FPVEC32STR(force_neigh));
					fp_vec3_t dist_vec = neigh->location;
					fp_vec3_sub(&dist_vec, &neigh_neigh->location);
//					ESP_LOGI(TAG, MACSTR" loc "FPVEC3STR", vel "FPVEC3STR", ideal dist "FPSTR", current dist "FPSTR, MAC2STR(neigh->address),
//					         FPVEC32STR(neigh->location), FPVEC32STR(neigh->velocity), FP2STR(ideal_dist), FP2STR(fp_vec3_mag(&dist_vec)));
					fp_vec3_add(&force, &force_neigh);
				}
			}
		}

		// Add applied force as acceleration to velocity (mass = 1)
		fp_vec3_add(&neigh->velocity, &force);

		// Limit velocity to ensure it remains manageable within the confines of the simulation
		fp_t velocity_mag = fp_vec3_mag(&neigh->velocity);
		if (velocity_mag > int_to_fp(10)) {
			fp_vec3_div(&neigh->velocity, velocity_mag);
			fp_vec3_mul(&neigh->velocity, int_to_fp(10));
		}

		fp_vec3_add(&neigh->location, &neigh->velocity);

//		ESP_LOGI(TAG, MACSTR" loc "FPVEC3STR", vel "FPVEC3STR", ideal dist "FPSTR", current dist "FPSTR, MAC2STR(neigh->address),
//			 FPVEC32STR(neigh->location), FPVEC32STR(neigh->velocity), FP2STR(ideal_dist), FP2STR(fp_vec3_mag(&neigh->location)));
		printf(MACSTR", "FPSTR", "FPSTR", "FPSTR"\n", MAC2STR(neigh->address), FP2STR(neigh->location.x), FP2STR(neigh->location.y), FP2STR(neigh->location.z));
	}

	xSemaphoreGive(neighbours.rssi_report_lock);
}

static void delete_neighbour(neighbour_t *neigh) {
	neighbour_t *neigh_rssi;
	list_head_t *next;

	taskENTER_CRITICAL(&neighbours.lock);
	// Clear TLB entry for this node
	set_tlb_entry_by_local_node_id(neigh->local_node_id, NULL);
	LIST_DELETE(&neigh->list);
	if (neigh == neighbours.clock_source) {
		neighbours.clock_source = NULL;
		update_clock_source();
	}
	taskEXIT_CRITICAL(&neighbours.lock);

	// Remove all references to TLB entry from RSSI reports
	LIST_FOR_EACH_ENTRY_SAFE(neigh_rssi, next, &neighbours.neighbours, list) {
		for (unsigned i = 0; i < neigh_rssi->num_rssi_reports; i++) {
			neighbour_rssi_info_t *report = &neigh_rssi->neighbour_rssi_reports[i];
			if (report->local_node_id == neigh->local_node_id) {
				report->local_node_id = 0;
			}
		}
	}
	free(neigh->neighbour_rssi_reports);
	free(neigh);
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
			delete_neighbour(neigh);
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

	simulate_links();

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
	neighbours.last_rssi_report_timestamp = 0;
	neighbours.local_id_tlb_size = 0;
	neighbours.local_id_tlb = NULL;
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
	printf("Address             Uptime       Age      RSSI     SoC    Time to empty   Firmware hash   Firmware version   OTA status\r\n");
	printf("========================================================================================================================\r\n");
	//      aa:bb:cc:dd:ee:ff   xxxxxxxxms   xxxxxms  -90dBm   100%   65535min        <short hash>    <firmware version>
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
			printf(MACSTR"   %8lums   %5lums   %2ddBm   %3d%%   %5dmin        %-13s   %-16.*s   %s\r\n",
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
			printf(MACSTR"   %8lums   %5lums   %2ddBm   ???       ???         ???            %-16.*s   %s\r\n",
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

unsigned int neighbour_take_rssi_reports(const neighbour_t *neigh, neighbour_rssi_info_t **rssi_reports) {
	xSemaphoreTake(neighbours.rssi_report_lock, portMAX_DELAY);
	*rssi_reports = neigh->neighbour_rssi_reports;
	return neigh->num_rssi_reports;
}

const uint8_t *neighbour_get_address_from_rssi_report(const neighbour_rssi_info_t *report) {
	unsigned int local_node_id = report->local_node_id;
	if (!local_node_id) {
		return NULL;
	}

	if (local_node_id == LOCAL_NODE_ID_SELF) {
		return wireless_get_mac_address();
	}

	neighbour_t *neigh = get_tlb_entry_by_local_node_id(local_node_id);
	if (!neigh) {
		return NULL;
	}

	return neigh->address;
}

void neighbour_put_rssi_reports(void) {
	xSemaphoreGive(neighbours.rssi_report_lock);
}
