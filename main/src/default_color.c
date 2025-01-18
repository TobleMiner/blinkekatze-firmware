#include "default_color.h"

#include <string.h>

#include <esp_err.h>
#include <esp_log.h>

#include "scheduler.h"
#include "shared_config.h"

typedef struct default_color {
	color_hsv_t default_color;
	shared_config_t shared_cfg;
	scheduler_task_t update_task;
} default_color_t;

typedef struct default_color_config_packet {
	uint8_t packet_type;
	color_hsv_t default_color;
	shared_config_hdr_t shared_cfg_hdr;
} __attribute__((packed)) default_color_config_packet_t;

static const char *TAG = "default_color";

static default_color_t default_color = {
	.default_color = { 0, HSV_SAT_MAX, HSV_VAL_MAX / 2 },
	.shared_cfg = { 0 }
};

static void default_color_tx(void) {
	default_color_config_packet_t packet = {
		.packet_type = WIRELESS_PACKET_TYPE_DEFAULT_COLOR,
		.default_color = default_color.default_color
	};
	shared_config_hdr_init(&default_color.shared_cfg, &packet.shared_cfg_hdr);
	wireless_broadcast((const uint8_t *)&packet, sizeof(packet));
	shared_config_tx_done(&default_color.shared_cfg);
}

static void config_changed(void) {
	shared_config_update_local(&default_color.shared_cfg);

	for (int i = 0; i < SHARED_CONFIG_TX_TIMES; i++) {
		default_color_tx();
	}
}

static void default_color_update(void *priv);
static void default_color_update(void *priv) {
	if (shared_config_should_tx(&default_color.shared_cfg)) {
		default_color_tx();
	}
	scheduler_schedule_task_relative(&default_color.update_task, default_color_update, NULL, MS_TO_US(1000));
}

void default_color_init(platform_t *platform) {
	default_color.default_color.v = platform->default_brightness;
	scheduler_task_init(&default_color.update_task);
	scheduler_schedule_task_relative(&default_color.update_task, default_color_update, NULL, MS_TO_US(1000));
}

void default_color_apply(color_hsv_t *color) {
	*color = default_color.default_color;
}

void default_color_rx(const wireless_packet_t *packet) {
	default_color_config_packet_t config_packet;
	if (packet->len < sizeof(config_packet)) {
		ESP_LOGD(TAG, "Received short packet, expected %u bytes but got only %u bytes",
		         sizeof(config_packet), packet->len);
		return;
	}
	memcpy(&config_packet, packet->data, sizeof(config_packet));

	if (shared_config_update_remote(&default_color.shared_cfg, &config_packet.shared_cfg_hdr)) {
		default_color.default_color = config_packet.default_color;
	}
}

void default_color_set_color(const color_hsv_t *color) {
	if (memcmp(color, &default_color.default_color, sizeof(*color))) {
		default_color.default_color = *color;
		config_changed();
	}
}
