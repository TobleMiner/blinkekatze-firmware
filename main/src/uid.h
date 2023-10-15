#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <esp_err.h>

#include "color.h"
#include "wireless.h"

void uid_enable(const uint8_t *address, bool enable);
esp_err_t uid_rx(const wireless_packet_t *packet);
void uid_apply(color_hsv_t *color);
