#include "node_info.h"

#include <stdio.h>

#include <esp_app_desc.h>
#include <esp_mac.h>

#include "power_control.h"
#include "util.h"
#include "wireless.h"

typedef struct node_info {
	bq27546_t *gauge;
} node_info_t;

static node_info_t node_info = { 0 };

void node_info_init(bq27546_t *gauge) {
	node_info.gauge = gauge;
}

void node_info_print_local() {
	const uint8_t *address = wireless_get_mac_address();
	printf("Node address:  "MACSTR"\r\n", MAC2STR(address));
	printf("Time to empty: %dmin\r\n", (int)MAX(bq27546_get_time_to_empty_min(node_info.gauge), -1));
	printf("Firmware:\r\n");
	const esp_app_desc_t *app_desc = esp_app_get_description();
	printf("  Version:     %s\r\n", app_desc->version);
	char firmware_hash_str[32 * 2 + 1] = { 0 };
	hex_encode(app_desc->app_elf_sha256, sizeof(app_desc->app_elf_sha256), firmware_hash_str, sizeof(firmware_hash_str) - 1);
	printf("  SHA256:      %s\r\n", firmware_hash_str);
	printf("Battery:\r\n");
	int current_ma = -32768;
	bq27546_get_current_ma(node_info.gauge, &current_ma);
	printf("  SoC:         %d%%\r\n", (int)MAX(bq27546_get_state_of_charge_percent(node_info.gauge), -1));
	printf("  Voltage:     %dmV\r\n", (int)MAX(bq27546_get_voltage_mv(node_info.gauge), -1));
	printf("  Current:     %dmA\r\n", current_ma);
	printf("  Temperature: %d°C\r\n", DIV_ROUND((int)bq27546_get_temperature_0_1k(node_info.gauge) - 2732, 10));
	printf("  Capacity:    %dmAh\r\n", (int)MAX(bq27546_get_full_charge_capacity_mah(node_info.gauge), -1));
	printf("  SoH:         %d%%\r\n", (int)MAX(bq27546_get_state_of_health_percent(node_info.gauge), -1));
	if (power_control_is_battery_storage_mode_enabled()) {
		printf("  Battery storage mode is enabled!\r\n");
	}
}

void node_info_print_remote(const neighbour_t *neigh) {
	const uint8_t *address = neigh->address;
	printf("Node address:  "MACSTR"\r\n", MAC2STR(address));
	if (neigh->last_status.packet_type) {
		const neighbour_status_packet_t *status = &neigh->last_status;
		printf("Time to empty: %dmin\r\n", (int)status->battery_time_to_empty_min);
	} else {
		printf("Time to empty: ??""?\r\n");
	}
	printf("Firmware:\r\n");
	if (neigh->last_static_info.packet_type) {
		const neighbour_static_info_packet_t *info = &neigh->last_static_info;
		printf("  Version:     %.*s\r\n",
		       sizeof(info->firmware_version),
		       info->firmware_version);
		char firmware_hash_str[32 * 2 + 1] = { 0 };
		hex_encode(info->firmware_sha256_hash, sizeof(info->firmware_sha256_hash),
			   firmware_hash_str, sizeof(firmware_hash_str) - 1);
		printf("  SHA256:      %s\r\n", firmware_hash_str);
	} else {
		printf("  Version:     ??""?\r\n");
		printf("  SHA256:      ??""?\r\n");
	}
	printf("Battery:\r\n");
	if (neigh->last_status.packet_type) {
		const neighbour_status_packet_t *status = &neigh->last_status;
		printf("  SoC:         %d%%\r\n", (int)status->battery_soc_percent);
		printf("  Voltage:     %dmV\r\n", (int)status->battery_voltage_mv);
		printf("  Current:     %dmA\r\n", (int)status->battery_current_ma);
		printf("  Temperature: %d°C\r\n", DIV_ROUND((int)status->battery_temperature_0_1k - 2732, 10));
		printf("  Capacity:    %dmAh\r\n", (int)status->battery_full_charge_capacity_mah);
		printf("  SoH:         %d%%\r\n", (int)status->battery_soh_percent);
	} else {
		printf("  SoC:         ??""?\r\n");
		printf("  Voltage:     ??""?\r\n");
		printf("  Current:     ??""?\r\n");
		printf("  Temperature: ??""?\r\n");
		printf("  Capacity:    ??""?\r\n");
		printf("  SoH:         ??""?\r\n");
	}

	printf("Neighbours:\r\n");
	neighbour_rssi_info_t *rssi_reports;
	unsigned int num_reports = neighbour_take_rssi_reports(neigh, &rssi_reports);
	if (num_reports) {
		for (unsigned int i = 0; i < num_reports; i++) {
			neighbour_rssi_info_t *report = &rssi_reports[i];
			if (!wireless_is_broadcast_address(report->address)) {
				printf("  "MACSTR": %d dBm\r\n", MAC2STR(report->address), report->rssi);
			}
		}
	}
	neighbour_put_rssi_reports();
}
