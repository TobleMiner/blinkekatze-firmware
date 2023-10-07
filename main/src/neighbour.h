#pragma once

#include <stdint.h>

#include <esp_err.h>
#include <esp_now.h>

#include "list.h"

typedef struct neighbour_advertisement {
	int64_t uptime_us;
	int64_t global_clock_us;
} neighbour_advertisement_t;

typedef struct neighbour {
	list_head_t list;
	uint8_t address[ESP_NOW_ETH_ALEN];
	int64_t last_local_adv_rx_timestamp_us;
	int64_t local_to_global_time_offset;
	neighbour_advertisement_t last_advertisement;
	int rssi;
} neighbour_t;

void neighbour_init(void);
esp_err_t neighbour_update(const uint8_t *address, int64_t timestamp_us, const neighbour_advertisement_t *adv);
void neighbour_housekeeping();
int64_t neighbour_get_global_clock();
int64_t neighbour_get_global_clock_and_source(neighbour_t **src);
esp_err_t neighbour_update_rssi(const uint8_t *address, int rssi);
