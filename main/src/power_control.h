#pragma once

#include <esp_err.h>

#include "bq24295.h"
#include "bq27546.h"
#include "wireless.h"

esp_err_t power_control_init(bq24295_t *charger, bq27546_t *gauge);
void power_control_rx(const wireless_packet_t *packet);
void power_control_set_ignore_power_switch(bool ignore);
bool power_control_is_powered_off(void);
void power_control_set_battery_storage_mode_enable(bool enable);
bool power_control_is_battery_storage_mode_enabled(void);
esp_err_t power_control_set_battery_storage_soc(unsigned int target_soc);
