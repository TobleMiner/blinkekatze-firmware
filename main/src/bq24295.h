#pragma once

#include <stdbool.h>

#include <esp_err.h>

#include "i2c_bus.h"

typedef struct bq24295 {
	i2c_bus_t *i2c_bus;
} bq24295_t;

typedef enum bq24295_watchdog_timeout {
	BQ24295_WATCHDOG_TIMEOUT_DISABLED = 0,
	BQ24295_WATCHDOG_TIMEOUT_40S = 1,
	BQ24295_WATCHDOG_TIMEOUT_80S = 2,
	BQ24295_WATCHDOG_TIMEOUT_160S = 3
} bq24295_watchdog_timeout_t;

typedef enum bq24295_battery_low_threshold {
	BQ24295_BATTERY_LOW_THRESHOLD_3_0V,
	BQ24295_BATTERY_LOW_THRESHOLD_2_8V
} bq24295_battery_low_threshold_t;

typedef enum bq24295_recharge_threshold {
	BQ24295_RECHARGE_THRESHOLD_100MV,
	BQ24295_RECHARGE_THRESHOLD_300MV
} bq24295_recharge_threshold_t;

esp_err_t bq24295_init(bq24295_t *charger, i2c_bus_t *bus);
esp_err_t bq24295_set_input_current_limit(bq24295_t *charger, unsigned int current_ma);
esp_err_t bq24295_reset(bq24295_t *charger);
esp_err_t bq24295_set_shutdown(bq24295_t *charger, bool shutdown);
esp_err_t bq24295_set_otg_enable(bq24295_t *charger, bool enable);
esp_err_t bq24295_set_charge_enable(bq24295_t *charger, bool enable);
esp_err_t bq24295_set_min_system_voltage(bq24295_t *charger, unsigned int voltage_mv);
esp_err_t bq24295_watchdog_reset(bq24295_t *charger);
esp_err_t bq24295_set_charge_current(bq24295_t *charger, unsigned int charge_current_ma);
esp_err_t bq24295_set_termination_current(bq24295_t *charger, unsigned int term_current_ma);
esp_err_t bq24295_set_precharge_current(bq24295_t *charger, unsigned int pchg_current_ma);
esp_err_t bq24295_set_watchdog_timeout(bq24295_t *charger, bq24295_watchdog_timeout_t timeout);
esp_err_t bq24295_set_boost_voltage(bq24295_t *charger, unsigned int boost_voltage_mv);
esp_err_t bq24295_set_battery_low_threshold(bq24295_t *charger, bq24295_battery_low_threshold_t threshold);
esp_err_t bq24295_set_recharge_threshold(bq24295_t *charger, bq24295_recharge_threshold_t threshold);
esp_err_t bq24295_is_charging(bq24295_t *charger, bool *is_charging);
esp_err_t bq24295_is_power_good(bq24295_t *charger, bool *power_good);
