#pragma once

#include <esp_err.h>

#include "bq24295.h"
#include "wireless.h"

esp_err_t power_control_init(bq24295_t *charger);
void power_control_rx(const wireless_packet_t *packet);
void power_control_set_ignore_power_switch(bool ignore);
bool power_control_is_powered_off(void);
