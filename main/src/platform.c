#include "platform.h"

#include <string.h>

#include <esp_log.h>

#include "platform_blinkekatze.h"
#include "platform_lacklight.h"
#include "platform_laempan.h"

static const char *TAG = "platform";

void platform_init(platform_t *plat, const platform_ops_t *ops) {
	memset(plat, 0, sizeof(*plat));
	plat->ops = ops;
}

esp_err_t platform_probe(platform_t **platform) {
	esp_err_t err = platform_blinkekatze_probe(platform);
	if (!err && *platform) {
		return 0;
	}

	err = platform_laempan_probe(platform);
	if (!err && *platform) {
		return 0;
	}

	err = platform_lacklight_probe(platform);
	if (!err && *platform) {
		return 0;
	}

	ESP_LOGE(TAG, "Failed to find a compatible platform");
	return ESP_FAIL;
}

void platform_pre_schedule(platform_t *platform) {
	if (platform->ops->pre_schedule) {
		platform->ops->pre_schedule(platform);
	}
}

void platform_set_rgb_led_color(platform_t *platform, uint16_t r, uint16_t g, uint16_t b) {
	if (platform->ops->set_rgb_led_color) {
		platform->ops->set_rgb_led_color(platform, r, g, b);
	}
}

bool platform_handle_packet(platform_t *platform, uint8_t packet_type, const wireless_packet_t *packet) {
	if (platform->ops->handle_packet) {
		return platform->ops->handle_packet(platform, packet_type, packet);
	}

	return false;
}

esp_err_t platform_set_color_channel_offset(platform_t *platform, unsigned int channel, unsigned int offset) {
	if (platform->ops->set_color_channel_offset) {
		return platform->ops->set_color_channel_offset(platform, channel, offset);
	}

	return ESP_ERR_NOT_SUPPORTED;
}
