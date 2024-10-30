#include "platform.h"

#include <string.h>

#include <esp_log.h>

#include "platform_blinkekatze.h"

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

	ESP_LOGE(TAG, "Failed to find compatible platform");
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
