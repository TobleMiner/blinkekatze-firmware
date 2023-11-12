#include "state_of_charge.h"

#include <stdbool.h>
#include <stdint.h>

#include <esp_log.h>
#include <esp_timer.h>

#include "scheduler.h"
#include "shared_config.h"
#include "util.h"

#define SOC_DISPLAY_DURATION_MS	3000

typedef struct state_of_charge {
	int64_t timestamp_init;
	shared_config_t shared_cfg;
	scheduler_task_t update_task;
	int soc;
	bool enable;
} state_of_charge_t;

#define FLAG_ENABLE_SOC_DISPLAY	BIT(0)

typedef struct state_of_charge_config_packet {
	uint8_t packet_type;
	uint8_t flags;
	shared_config_hdr_t shared_cfg_hdr;
} __attribute__((packed)) state_of_charge_config_packet_t;

static const char *TAG = "state_of_charge";

static state_of_charge_t state_of_charge;

static void state_of_charge_tx(void) {
	state_of_charge_config_packet_t packet = {
		.packet_type = WIRELESS_PACKET_TYPE_STATE_OF_CHARGE,
		.flags = state_of_charge.enable ? FLAG_ENABLE_SOC_DISPLAY : 0
	};
	shared_config_hdr_init(&state_of_charge.shared_cfg, &packet.shared_cfg_hdr);
	wireless_broadcast((const uint8_t *)&packet, sizeof(packet));
	shared_config_tx_done(&state_of_charge.shared_cfg);
}

static void config_changed(void) {
	shared_config_update_local(&state_of_charge.shared_cfg);

	for (int i = 0; i < SHARED_CONFIG_TX_TIMES; i++) {
		state_of_charge_tx();
	}
}

static void state_of_charge_update(void *priv);
static void state_of_charge_update(void *priv) {
	if (shared_config_should_tx(&state_of_charge.shared_cfg)) {
		state_of_charge_tx();
	}
	scheduler_schedule_task_relative(&state_of_charge.update_task, state_of_charge_update, NULL, MS_TO_US(1000));
}

void state_of_charge_init(bq27546_t *gauge) {
	state_of_charge.soc = bq27546_get_state_of_charge_percent(gauge);
	state_of_charge.timestamp_init = esp_timer_get_time();
	state_of_charge.enable = false;
	scheduler_task_init(&state_of_charge.update_task);
	scheduler_schedule_task_relative(&state_of_charge.update_task, state_of_charge_update, NULL, MS_TO_US(1000));
}

void state_of_charge_rx(const wireless_packet_t *packet) {
	state_of_charge_config_packet_t config_packet;
	if (packet->len < sizeof(config_packet)) {
		ESP_LOGD(TAG, "Received short packet, expected %u bytes but got only %u bytes",
		         sizeof(config_packet), packet->len);
		return;
	}
	memcpy(&config_packet, packet->data, sizeof(config_packet));

	if (shared_config_update_remote(&state_of_charge.shared_cfg, &config_packet.shared_cfg_hdr)) {
		state_of_charge.enable = !!(config_packet.flags & FLAG_ENABLE_SOC_DISPLAY);
	}
}

void state_of_charge_apply(color_hsv_t *color) {
	int64_t now = esp_timer_get_time();
	if ((now - state_of_charge.timestamp_init <= MS_TO_US(SOC_DISPLAY_DURATION_MS)) ||
	    state_of_charge.enable) {
		if (state_of_charge.soc >= 0) {
			// Red to Green
			int32_t hue = HSV_HUE_MAX * 120L * (int32_t)state_of_charge.soc / 360 / 100;
			color->h = hue;
		} else {
			// Blue
			color->h = HSV_HUE_MAX * 240L / 360;
		}
	}
}

void state_of_charge_set_display_enable(bool enable) {
	if (state_of_charge.enable != enable) {
		state_of_charge.enable = enable;
		config_changed();
	}
}
