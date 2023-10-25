#include "rainbow_fade.h"

#include "neighbour.h"

#include <string.h>

#include <esp_log.h>
#include <esp_timer.h>

#include "util.h"

#define HUE_CYCLE_TIME_MS	10000
#define CONFIG_TX_INTERVAL_MS	10000
#define CONFIG_CHANGE_TX_TIMES	3

typedef struct rainbow_fade {
	int64_t config_timestamp_us;
	unsigned long hue_cycle_time_ms;
	bool enable;
	bool enable_phase_shift;
	neighbour_rssi_delay_model_t delay_model;
	int64_t last_rx_timestamp_us;
	int64_t last_tx_timestamp_us;
} rainbow_fade_t;

#define FLAG_ENABLE		BIT(0)
#define FLAG_PHASE_SHIFT_ENABLE	BIT(1)

typedef struct rainbow_fade_config_packet {
	uint8_t packet_type;
	uint8_t flags;
	int8_t delay_model_rssi_treshold;
	int8_t delay_model_rssi_limit;
	uint32_t hue_cycle_time_ms;
	int64_t config_timestamp_us;
	int32_t delay_model_us_delay_per_rssi_step;
	int32_t delay_model_delay_limit_us;
} __attribute__((packed)) rainbow_fade_config_packet_t;

static const char *TAG = "rainbow_fade";

static rainbow_fade_t rainbow_fade = {
	.config_timestamp_us = 0,
	.hue_cycle_time_ms = HUE_CYCLE_TIME_MS,
	.enable = true,
	.delay_model = {
		2000,
		0,
		-20,
		-90
	}
};

static void rainbow_fade_tx(void) {
	rainbow_fade_config_packet_t packet = {
		.packet_type = WIRELESS_PACKET_TYPE_RAINBOW_FADE,
		.config_timestamp_us = rainbow_fade.config_timestamp_us,
		.hue_cycle_time_ms = rainbow_fade.hue_cycle_time_ms,
		.flags =
			(rainbow_fade.enable ? FLAG_ENABLE : 0) |
			(rainbow_fade.enable_phase_shift ? FLAG_PHASE_SHIFT_ENABLE : 0),
		.delay_model_rssi_treshold = rainbow_fade.delay_model.delay_rssi_threshold,
		.delay_model_rssi_limit = rainbow_fade.delay_model.delay_rssi_limit,
		.delay_model_us_delay_per_rssi_step = rainbow_fade.delay_model.us_delay_per_rssi_step,
		.delay_model_delay_limit_us = rainbow_fade.delay_model.delay_limit_us
	};

	rainbow_fade.last_tx_timestamp_us = esp_timer_get_time();
	wireless_broadcast((const uint8_t *)&packet, sizeof(packet));
}

static void config_changed(void) {
	int64_t now = neighbour_get_global_clock();
	rainbow_fade.config_timestamp_us = now;
	rainbow_fade.last_rx_timestamp_us = 0;

	for (int i = 0; i < CONFIG_CHANGE_TX_TIMES; i++) {
		rainbow_fade_tx();
	}
}

void rainbow_fade_update() {
	if (!rainbow_fade.config_timestamp_us) {
		// We are in default config, no need to announce that
		return;
	}

	int64_t now = esp_timer_get_time();
	int64_t delta_us_last_rx = now - rainbow_fade.last_rx_timestamp_us;
	if (delta_us_last_rx / 1000LL < CONFIG_TX_INTERVAL_MS * 2) {
		// Up to date config received from neighbour already, no need to distribute
		return;
	}

	int64_t delta_us_last_tx = now - rainbow_fade.last_tx_timestamp_us;
	if (delta_us_last_tx / 1000LL < CONFIG_TX_INTERVAL_MS) {
		// Up to date config sent recently, do not flood the network
		return;
	}

	rainbow_fade_tx();
}

void rainbow_fade_apply(color_hsv_t *color) {
	if (rainbow_fade.enable) {
		neighbour_t *clock_source;
		int64_t now = neighbour_get_global_clock_and_source(&clock_source);
		if (rainbow_fade.enable_phase_shift && clock_source) {
			now += neighbour_calculate_rssi_delay(&rainbow_fade.delay_model, clock_source);
		}
		int64_t cycle_val = now / 1000 * HSV_HUE_STEPS / (int64_t)rainbow_fade.hue_cycle_time_ms;
		uint16_t hue_delta = cycle_val % HSV_HUE_STEPS;

		color->h = (color->h + hue_delta) % HSV_HUE_STEPS;
	}
}

void rainbow_fade_rx(const wireless_packet_t *packet) {
	rainbow_fade_config_packet_t config_packet;
	if (packet->len < sizeof(config_packet)) {
		ESP_LOGD(TAG, "Receied short packet, expected %u bytes but got only %u bytes",
		         sizeof(config_packet), packet->len);
		return;
	}
	memcpy(&config_packet, packet->data, sizeof(config_packet));

	if (config_packet.config_timestamp_us > rainbow_fade.config_timestamp_us) {
		rainbow_fade.hue_cycle_time_ms = config_packet.hue_cycle_time_ms;
		rainbow_fade.enable = !!(config_packet.flags & FLAG_ENABLE);
		rainbow_fade.enable_phase_shift = !!(config_packet.flags & FLAG_PHASE_SHIFT_ENABLE);
		rainbow_fade.delay_model.delay_rssi_threshold = config_packet.delay_model_rssi_treshold;
		rainbow_fade.delay_model.delay_rssi_limit = config_packet.delay_model_rssi_limit;
		rainbow_fade.delay_model.us_delay_per_rssi_step = config_packet.delay_model_us_delay_per_rssi_step;
		rainbow_fade.delay_model.delay_limit_us = config_packet.delay_model_delay_limit_us;
		rainbow_fade.config_timestamp_us = config_packet.config_timestamp_us;
	}

	if (config_packet.config_timestamp_us >= rainbow_fade.config_timestamp_us) {
		rainbow_fade.last_rx_timestamp_us = packet->rx_timestamp;
	}
}

void rainbow_fade_set_cycle_time(unsigned long cycle_time_ms) {
	if (cycle_time_ms != rainbow_fade.hue_cycle_time_ms) {
		rainbow_fade.hue_cycle_time_ms = cycle_time_ms;
		config_changed();
	}
}

void rainbow_fade_set_enable(bool enable) {
	if (enable != rainbow_fade.enable) {
		rainbow_fade.enable = enable;
		config_changed();
	}
}

void rainbow_fade_set_phase_shift_enable(bool enable) {
	if (enable != rainbow_fade.enable_phase_shift) {
		rainbow_fade.enable_phase_shift = enable;
		config_changed();
	}
}

void rainbow_fade_set_rssi_delay_model(const neighbour_rssi_delay_model_t *model) {
	if (memcmp(model, &rainbow_fade.delay_model, sizeof(*model))) {
		rainbow_fade.delay_model = *model;
		config_changed();
	}
}
