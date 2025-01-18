#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <esp_err.h>

#include "bq24295.h"
#include "bq27546.h"
#include "lis3dh.h"
#include "ltr_303als.h"
#include "spl06.h"
#include "wireless.h"

typedef struct platform platform_t;

typedef struct platform_ops {
	void (*pre_schedule)(platform_t *platform);
	void (*set_rgb_led_color)(platform_t *platform, uint16_t r, uint16_t g, uint16_t b);
	void (*set_brightness_white)(platform_t *platform, uint16_t bright);
	bool (*handle_packet)(platform_t *platform, uint8_t packet_type, const wireless_packet_t *packet);
	esp_err_t (*set_color_channel_offset)(platform_t *platform, unsigned int channel, unsigned int offset);
} platform_ops_t;

struct platform {
	bq27546_t *gauge;
	bq24295_t *charger;
	ltr_303als_t *als;
	lis3dh_t *accelerometer;
	spl06_t *barometer;
	unsigned int default_brightness;
	const char *name;
	const platform_ops_t *ops;
};

void platform_init(platform_t *plat, const platform_ops_t *ops, const char *name);
esp_err_t platform_probe(platform_t **platform);
void platform_pre_schedule(platform_t *platform);
void platform_set_rgb_led_color(platform_t *platform, uint16_t r, uint16_t g, uint16_t b);
void platform_set_brightness_white(platform_t *platform, uint16_t bright);
bool platform_handle_packet(platform_t *platform, uint8_t packet_type, const wireless_packet_t *packet);
esp_err_t platform_set_color_channel_offset(platform_t *platform, unsigned int channel, unsigned int offset);
bool platform_is(const platform_t *platform, const char *name);
