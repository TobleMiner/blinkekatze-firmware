#include "brightness.h"

#include <string.h>

#include <esp_err.h>
#include <esp_log.h>

#include "scheduler.h"
#include "shared_config.h"

typedef struct brightness {
	unsigned int brightness;
	shared_config_t shared_cfg;
	scheduler_task_t update_task;
} brightness_t;

typedef struct brightness_config_packet {
	uint8_t packet_type;
	uint8_t brightness;
	shared_config_hdr_t shared_cfg_hdr;
} __attribute__((packed)) brightness_config_packet_t;

static const char *TAG = "brightness";

static brightness_t brightness = {
	.brightness = 255,
	.shared_cfg = { 0 }
};

static void brightness_tx(void) {
	brightness_config_packet_t packet = {
		.packet_type = WIRELESS_PACKET_TYPE_BRIGHTNESS,
		.brightness = brightness.brightness
	};
	shared_config_hdr_init(&brightness.shared_cfg, &packet.shared_cfg_hdr);
	wireless_broadcast((const uint8_t *)&packet, sizeof(packet));
	shared_config_tx_done(&brightness.shared_cfg);
}

static void config_changed(void) {
	shared_config_update_local(&brightness.shared_cfg);

	for (int i = 0; i < SHARED_CONFIG_TX_TIMES; i++) {
		brightness_tx();
	}
}

static void brightness_update(void *priv);
static void brightness_update(void *priv) {
	if (shared_config_should_tx(&brightness.shared_cfg)) {
		brightness_tx();
	}
	scheduler_schedule_task_relative(&brightness.update_task, brightness_update, NULL, MS_TO_US(1000));
}

void brightness_init(platform_t *platform) {
	brightness.brightness = platform->default_brightness;
	scheduler_task_init(&brightness.update_task);
	scheduler_schedule_task_relative(&brightness.update_task, brightness_update, NULL, MS_TO_US(1000));
}

void brightness_apply(color_hsv_t *color) {
	color->v = DIV_ROUND((uint32_t)color->v * brightness.brightness, 255);
}

void brightness_rx(const wireless_packet_t *packet) {
	brightness_config_packet_t config_packet;
	if (packet->len < sizeof(config_packet)) {
		ESP_LOGD(TAG, "Received short packet, expected %u bytes but got only %u bytes",
		         sizeof(config_packet), packet->len);
		return;
	}
	memcpy(&config_packet, packet->data, sizeof(config_packet));

	if (shared_config_update_remote(&brightness.shared_cfg, &config_packet.shared_cfg_hdr)) {
		brightness.brightness = config_packet.brightness;
	}
}

void brightness_set_brightness(unsigned int brightness_) {
	if (brightness.brightness != brightness_) {
		brightness.brightness = brightness_;
		config_changed();
	}
}
