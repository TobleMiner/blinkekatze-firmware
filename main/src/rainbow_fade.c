#include "rainbow_fade.h"

#include "neighbour.h"

#include <string.h>

#include <esp_log.h>
#include <esp_timer.h>

#include "color_palette.h"
#include "scheduler.h"
#include "shared_config.h"
#include "util.h"

#define HUE_CYCLE_TIME_MS	10000

typedef struct rainbow_fade {
	unsigned long hue_cycle_time_ms;
	bool enable;
	bool enable_phase_shift;
	neighbour_rssi_delay_model_t delay_model;
	shared_config_t shared_cfg;
	scheduler_task_t update_task;
	unsigned int palette_index;
} rainbow_fade_t;

#define FLAG_ENABLE		BIT(0)
#define FLAG_PHASE_SHIFT_ENABLE	BIT(1)

typedef struct rainbow_fade_config_packet {
	uint8_t packet_type;
	uint8_t flags;
	int8_t delay_model_rssi_treshold;
	int8_t delay_model_rssi_limit;
	uint32_t hue_cycle_time_ms;
	shared_config_hdr_t shared_cfg_hdr;
	int32_t delay_model_us_delay_per_rssi_step;
	int32_t delay_model_delay_limit_us;
} __attribute__((packed)) rainbow_fade_config_packet_t;

static const char *TAG = "rainbow_fade";

static rainbow_fade_t rainbow_fade = {
	.shared_cfg = { 0 },
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
		.hue_cycle_time_ms = rainbow_fade.hue_cycle_time_ms,
		.flags =
			(rainbow_fade.enable ? FLAG_ENABLE : 0) |
			(rainbow_fade.enable_phase_shift ? FLAG_PHASE_SHIFT_ENABLE : 0),
		.delay_model_rssi_treshold = rainbow_fade.delay_model.delay_rssi_threshold,
		.delay_model_rssi_limit = rainbow_fade.delay_model.delay_rssi_limit,
		.delay_model_us_delay_per_rssi_step = rainbow_fade.delay_model.us_delay_per_rssi_step,
		.delay_model_delay_limit_us = rainbow_fade.delay_model.delay_limit_us
	};
	shared_config_hdr_init(&rainbow_fade.shared_cfg, &packet.shared_cfg_hdr);
	wireless_broadcast((const uint8_t *)&packet, sizeof(packet));
	shared_config_tx_done(&rainbow_fade.shared_cfg);
}

static void config_changed(void) {
	shared_config_update_local(&rainbow_fade.shared_cfg);

	for (int i = 0; i < SHARED_CONFIG_TX_TIMES; i++) {
		rainbow_fade_tx();
	}
}

static void rainbow_fade_update(void *priv);
static void rainbow_fade_update(void *priv) {
	if (shared_config_should_tx(&rainbow_fade.shared_cfg)) {
		rainbow_fade_tx();
	}
	scheduler_schedule_task_relative(&rainbow_fade.update_task, rainbow_fade_update, NULL, MS_TO_US(1000));
}

void rainbow_fade_init() {
	scheduler_task_init(&rainbow_fade.update_task);
	scheduler_schedule_task_relative(&rainbow_fade.update_task, rainbow_fade_update, NULL, MS_TO_US(1000));
	rainbow_fade.palette_index = 0;
}

void rainbow_fade_apply(color_t *color) {
	if (rainbow_fade.enable) {
		color_hsv_t *hsv = color_to_hsv(color);

		neighbour_t *clock_source;
		int64_t now = neighbour_get_global_clock_and_source(&clock_source);
		if (rainbow_fade.enable_phase_shift && clock_source) {
			now -= neighbour_calculate_rssi_delay(&rainbow_fade.delay_model, clock_source);
		}
		int64_t cycle_val = now / 1000 * HSV_HUE_STEPS / (int64_t)rainbow_fade.hue_cycle_time_ms;
		uint16_t hue_delta = cycle_val % HSV_HUE_STEPS;

		hsv->h = (hsv->h + hue_delta) % HSV_HUE_STEPS;
/*
		ESP_LOGI(TAG, "====================");
		ESP_LOGI(TAG, "1. HSV: %u, %u, %u", color->hsv.h, color->hsv.s, color->hsv.v);
		color_to_rgb(color);
		ESP_LOGI(TAG, "2. RGB: %u, %u, %u", color->rgb.r, color->rgb.g, color->rgb.b);
		color_to_hsv(color);
		ESP_LOGI(TAG, "3. HSV: %u, %u, %u", color->hsv.h, color->hsv.s, color->hsv.v);
*/
	} else {
		const color_palette_t *palette = &rainbow_stripe_palette;

		neighbour_t *clock_source;
		int64_t now = neighbour_get_global_clock_and_source(&clock_source);

		int64_t us_per_color = DIV_ROUND((int64_t)rainbow_fade.hue_cycle_time_ms * 1000LL, palette->num_colors);
		int64_t color_progress = now % us_per_color;
		uint16_t color_blend_progress = color_progress * COLOR_BLEND_MAX / us_per_color;

		unsigned int current_color_idx = (now / 1000 * palette->num_colors / (int64_t)rainbow_fade.hue_cycle_time_ms) % palette->num_colors;
		unsigned int next_color_idx = (current_color_idx + 1) % palette->num_colors;

		rgb16_t current_color = palette->colors[current_color_idx].color;
		color_scale_rgb(&current_color, palette->color_maxval, 65535);

		rgb16_t next_color = palette->colors[next_color_idx].color;
		color_scale_rgb(&next_color, palette->color_maxval, 65535);

		color_blend_rgb(color, &current_color, &next_color, color_blend_progress);
//		ESP_LOGI(TAG, "idx: %u, progress: %05u, color: %05u, %05u, %05u", current_color_idx, color_blend_progress, color->rgb.r, color->rgb.g, color->rgb.b);
//		color->rgb = current_color;
	}
}

void rainbow_fade_rx(const wireless_packet_t *packet) {
	rainbow_fade_config_packet_t config_packet;
	if (packet->len < sizeof(config_packet)) {
		ESP_LOGD(TAG, "Received short packet, expected %u bytes but got only %u bytes",
		         sizeof(config_packet), packet->len);
		return;
	}
	memcpy(&config_packet, packet->data, sizeof(config_packet));

	if (shared_config_update_remote(&rainbow_fade.shared_cfg, &config_packet.shared_cfg_hdr)) {
		rainbow_fade.hue_cycle_time_ms = config_packet.hue_cycle_time_ms;
		rainbow_fade.enable = !!(config_packet.flags & FLAG_ENABLE);
		rainbow_fade.enable_phase_shift = !!(config_packet.flags & FLAG_PHASE_SHIFT_ENABLE);
		rainbow_fade.delay_model.delay_rssi_threshold = config_packet.delay_model_rssi_treshold;
		rainbow_fade.delay_model.delay_rssi_limit = config_packet.delay_model_rssi_limit;
		rainbow_fade.delay_model.us_delay_per_rssi_step = config_packet.delay_model_us_delay_per_rssi_step;
		rainbow_fade.delay_model.delay_limit_us = config_packet.delay_model_delay_limit_us;
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
