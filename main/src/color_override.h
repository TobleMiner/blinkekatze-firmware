#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "color.h"
#include "wireless.h"

void color_override_set_enable(bool enable);
void color_override_set_color(const rgb16_t *rgb);
void color_override_apply(rgb16_t *rgb);
void color_override_rx(wireless_packet_t *packet);
void color_override_tx(const rgb16_t *color, int64_t time_start_global_us, int64_t time_stop_global_us);
