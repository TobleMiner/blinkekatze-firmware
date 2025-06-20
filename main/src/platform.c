#include "platform.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <esp_log.h>

#include "color.h"
#include "platform_blinkekatze.h"
#include "platform_lacklight.h"
#include "platform_laempan.h"
#include "platform_tallylight_v2.h"
#include "settings.h"
#include "util.h"

static const char *TAG = "platform";

static platform_def_t *platforms[] = {
	&platform_blinkekatze,
	&platform_laempan,
	&platform_tallylight_v2,
	&platform_lacklight,
};

void platform_init(platform_t *plat, const platform_def_t *def) {
	memset(plat, 0, sizeof(*plat));
	plat->def = def;
	plat->default_brightness = HSV_VAL_MAX / 2;
}

static esp_err_t platform_probe_(const platform_def_t *def, platform_t **platform) {
	esp_err_t err;

	for (unsigned int i = 0; i < 10; i++) {
		esp_err_t err = def->ops->probe(platform);

		if (!err && (!platform || *platform)) {
			return 0;
		}
	}

	return err;
}

static const platform_def_t *find_platform_def_by_name(const char *name) {
	for (unsigned int i = 0; i < ARRAY_SIZE(platforms); i++) {
		const platform_def_t *def = platforms[i];

		if (!strcmp(def->name, name)) {
			return def;
		}
	}

	return 0;
}

esp_err_t platform_probe(platform_t **platform) {
	esp_err_t err;
	const platform_def_t *def;
	char *platform_name = settings_get_platform_name();
	bool store_new_platform = true;

	if (platform_name) {
		def = find_platform_def_by_name(platform_name);
		if (def) {
			err = platform_probe_(def, platform);
			if (!err && *platform) {
				return 0;
			}
		}
		free(platform_name);
		store_new_platform = false;
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(platforms); i++) {
		const platform_def_t *def = platforms[i];

		ESP_LOGI(TAG, "Checking compatibility with platform %s...", def->name);
		err = platform_probe_(def, platform);
		if (!err && *platform) {
			ESP_LOGI(TAG, "Platform %s is compatible", def->name);
			if (store_new_platform) {
				settings_set_platform_name(def->name);
			}
			return 0;
		}
	}

	ESP_LOGE(TAG, "Failed to find a compatible platform");
	return ESP_FAIL;
}

void platform_pre_schedule(platform_t *platform) {
	if (platform->def->ops->pre_schedule) {
		platform->def->ops->pre_schedule(platform);
	}
}

void platform_set_rgb_led_color(platform_t *platform, uint16_t r, uint16_t g, uint16_t b) {
	if (platform->def->ops->set_rgb_led_color) {
		platform->def->ops->set_rgb_led_color(platform, r, g, b);
	}
}

bool platform_handle_packet(platform_t *platform, uint8_t packet_type, const wireless_packet_t *packet) {
	if (platform->def->ops->handle_packet) {
		return platform->def->ops->handle_packet(platform, packet_type, packet);
	}

	return false;
}

esp_err_t platform_set_color_channel_offset(platform_t *platform, unsigned int channel, unsigned int offset) {
	if (platform->def->ops->set_color_channel_offset) {
		return platform->def->ops->set_color_channel_offset(platform, channel, offset);
	}

	return ESP_ERR_NOT_SUPPORTED;
}

bool platform_is(const platform_t *platform, const char *name) {
	return !strcmp(platform->def->name, name);
}

void platform_set_brightness_white(platform_t *platform, uint16_t bright) {
	if (platform->def->ops->set_brightness_white) {
		platform->def->ops->set_brightness_white(platform, bright);
	}
}
