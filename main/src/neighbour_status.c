#include "neighbour_status.h"

#include <string.h>

#include <esp_log.h>
#include <esp_mac.h>
#include <esp_timer.h>

#include "util.h"

#define NEIGHBOUR_STATUS_INTERVAL_MS	10000

typedef struct neighbour_status {
	bq27546_t *gauge;
	int64_t	timestamp_last_status_tx_us;
} neighbour_status_t;

static const char *TAG = "node_status";

neighbour_status_t neighbour_status = { 0 };

void neighbour_status_init(bq27546_t *battery_gauge) {
	neighbour_status.gauge = battery_gauge;
}

void neighbour_status_rx(const wireless_packet_t *packet, const neighbour_t *neigh) {
	if (packet->len < sizeof(neighbour_status_packet_t)) {
		ESP_LOGI(TAG, "Short packet received, expected %u bytes but got %u bytes", sizeof(neighbour_status_packet_t), packet->len);
	}
	neighbour_status_packet_t status;
	memcpy(&status, packet->data, sizeof(neighbour_status_packet_t));
	if (neigh) {
		uint32_t uptime_ms = neighbour_get_uptime(neigh) / 1000;
		ESP_LOGI(TAG, "<"MACSTR"> Uptime <%lums>, Battery <%d%%, %dmV, %dmA, %d°C>",
			 MAC2STR(packet->src_addr), uptime_ms, (int)status.battery_soc_percent,
			 (int)status.battery_voltage_mv, (int)status.battery_current_ma,
			 DIV_ROUND((int)status.battery_temperature_0_1k - 2732, 10));
	} else {
		ESP_LOGI(TAG, "<"MACSTR"> Uptime <??""?>, Battery <%d%%, %dmV, %dmA, %d°C>",
			 MAC2STR(packet->src_addr), (int)status.battery_soc_percent,
			 (int)status.battery_voltage_mv, (int)status.battery_current_ma,
			 DIV_ROUND((int)status.battery_temperature_0_1k - 2732, 10));
	}
}

esp_err_t neighbour_status_update(void) {
	int64_t now = esp_timer_get_time();
	esp_err_t err = ESP_OK;
	if (now > neighbour_status.timestamp_last_status_tx_us + MS_TO_US(NEIGHBOUR_STATUS_INTERVAL_MS)) {
		neighbour_status_packet_t status = { .packet_type = WIRELESS_PACKET_TYPE_NEIGHBOUR_STATUS };
		status.battery_soc_percent = MAX(bq27546_get_state_of_charge_percent(neighbour_status.gauge), -1);
		status.battery_voltage_mv = MAX(bq27546_get_voltage_mv(neighbour_status.gauge), -1);
		int current_ma = -32768;
		err = bq27546_get_current_ma(neighbour_status.gauge, &current_ma);
		status.battery_current_ma = current_ma;
		status.battery_temperature_0_1k = bq27546_get_temperature_0_1k(neighbour_status.gauge);
		wireless_broadcast((const uint8_t *)&status, sizeof(status));
		neighbour_status.timestamp_last_status_tx_us = now;
	}

	return err;
}
