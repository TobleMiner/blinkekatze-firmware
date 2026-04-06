#pragma once

#include "color.h"
#include "platform.h"
#include "wireless.h"

#define BRIGHTNESS_MAX 255

void brightness_init(platform_t *platform);
void brightness_apply(color_hsv_t *color);
void brightness_rx(const wireless_packet_t *packet);
void brightness_set_brightness(unsigned int brightness);
