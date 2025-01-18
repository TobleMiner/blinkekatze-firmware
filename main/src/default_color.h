#pragma once

#include "color.h"
#include "platform.h"
#include "wireless.h"

void default_color_init(platform_t *platform);
void default_color_apply(color_hsv_t *color);
void default_color_rx(const wireless_packet_t *packet);
void default_color_set_color(const color_hsv_t *color);
