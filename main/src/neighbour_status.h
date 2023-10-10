#pragma once

#include <stdint.h>

#include "bq27546.h"
#include "neighbour.h"
#include "wireless.h"

typedef struct neighbour_status_packet {
	uint8_t packet_type;
	int8_t battery_soc_percent;
	int16_t battery_voltage_mv;
	int16_t battery_current_ma;
	int16_t battery_temperature_0_1k;
	int16_t battery_time_to_empty_min;
	int16_t battery_full_charge_capacity_mah;
	int8_t battery_soh_percent;
} __attribute__((packed)) neighbour_status_packet_t;

void neighbour_status_init(bq27546_t *battery_gauge);
void neighbour_status_rx(const wireless_packet_t *packet, const neighbour_t *neigh);
esp_err_t neighbour_status_update(void);
