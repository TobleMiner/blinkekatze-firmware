#pragma once

#include "bq27546.h"
#include "color.h"
#include "wireless.h"

void state_of_charge_init(bq27546_t *gauge);
void state_of_charge_apply(color_hsv_t *color);
void state_of_charge_rx(const wireless_packet_t *packet);
void state_of_charge_set_display_enable(bool enable);
