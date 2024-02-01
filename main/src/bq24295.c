#include "bq24295.h"

#include <esp_err.h>
#include <esp_log.h>

#include "util.h"

#define BQ24295_ADDRESS	0x6B

#define REG_INPUT_CTRL		0x00
#define REG_POWER_ON_CFG	0x01
#define REG_CHARGE_CURRENT	0x02
#define REG_PRE_CHARGE_TERM	0x03
#define REG_CHARGE_CTRL		0x04
#define REG_TERM_TIMER		0x05
#define REG_BOOST_THERMAL	0x06
#define REG_MISC_CTRL		0x07
#define REG_SYSTEM_STATUS	0x08
#define REG_VENDOR		0x0a

typedef enum bq24295_charge_status {
	BQ24295_CHARGE_STATUS_NOT_CHARGING	= 0,
	BQ24295_CHARGE_STATUS_PRE_CHARGE	= 1,
	BQ24295_CHARGE_STATUS_FAST_CHARGING	= 2,
	BQ24295_CHARGE_STATUS_CHARGE_DONE	= 3,
} bq24295_charge_status_t;

typedef enum bq24295_vbus_status {
	BQ24295_VBUS_STATUS_UNKNWON		= 0,
	BQ24295_VBUS_STATUS_USB_HOST		= 1,
	BQ24295_VBUS_STATUS_ADAPTER_PORT	= 2,
	BQ24295_VBUS_STATUS_OTG			= 3,
} bq24295_vbus_status_t;

static const unsigned int bq24295_input_current_lookup_ma[] = {
	100, 150, 500, 900, 1000, 1500, 2000, 3000
};

const char *TAG = "bq24295";

static esp_err_t bq24295_r(bq24295_t *charger, uint8_t reg, uint8_t *val) {
	return i2c_bus_read_byte(charger->i2c_bus, BQ24295_ADDRESS, reg, val);
}

esp_err_t bq24295_init(bq24295_t *charger, i2c_bus_t *bus) {
	charger->i2c_bus = bus;

	uint8_t device_id;
	esp_err_t err = bq24295_r(charger, REG_VENDOR, &device_id);
	if (err) {
		ESP_LOGE(TAG, "Failed to read device id register: %d", err);
		return err;
	}

	if (device_id != 0xc0) {
		ESP_LOGE(TAG, "Unsupported device id 0x%02x", device_id);
		return ESP_ERR_NOT_SUPPORTED;
	}

	ESP_LOGI(TAG, "Found BQ24295 @0x%02x", BQ24295_ADDRESS);
	return ESP_OK;
}

static esp_err_t bq24295_rmw(bq24295_t *charger, uint8_t reg, uint8_t clear, uint8_t set) {
	uint8_t val;
	esp_err_t err = bq24295_r(charger, reg, &val);
	if (err) {
		return err;
	}
	val &= ~clear;
	val |= set;
	return i2c_bus_write_byte(charger->i2c_bus, BQ24295_ADDRESS, reg, val);
}

esp_err_t bq24295_set_hiz_enable(bq24295_t *charger, bool enable) {
	if (enable) {
		return bq24295_rmw(charger, REG_INPUT_CTRL, 0x00, 0x80);
	} else {
		return bq24295_rmw(charger, REG_INPUT_CTRL, 0x80, 0x00);
	}
}

esp_err_t bq24295_set_input_current_limit(bq24295_t *charger, unsigned int current_ma) {
	uint8_t limit_bits;
	for (limit_bits = 0; limit_bits < ARRAY_SIZE(bq24295_input_current_lookup_ma); limit_bits++) {
		unsigned int current_limit_ma = bq24295_input_current_lookup_ma[limit_bits];

		if (current_limit_ma > current_ma) {
			if (!limit_bits) {
				return ESP_ERR_INVALID_ARG;
			}
			limit_bits--;
			break;
		} else if (current_limit_ma == current_ma) {
			break;
		}
	}

	return bq24295_rmw(charger, REG_INPUT_CTRL, 0x07, limit_bits);
}

esp_err_t bq24295_get_input_current_limit(bq24295_t *charger, unsigned int *current_ma) {
	uint8_t regval;
	esp_err_t err = bq24295_r(charger, REG_INPUT_CTRL, &regval);
	if (err) {
		return err;
	}

	unsigned int limit_bits = regval & 0x07;
	*current_ma = bq24295_input_current_lookup_ma[limit_bits];
	return ESP_OK;
}

esp_err_t bq24295_reset(bq24295_t *charger) {
	return bq24295_rmw(charger, REG_POWER_ON_CFG, 0x00, 0x81);
}

esp_err_t bq24295_set_shutdown(bq24295_t *charger, bool shutdown) {
	if (shutdown) {
		return bq24295_rmw(charger, REG_MISC_CTRL, 0x14, 0x28);
	} else {
		return bq24295_rmw(charger, REG_MISC_CTRL, 0x34, 0x08);
	}
}

esp_err_t bq24295_set_otg_enable(bq24295_t *charger, bool enable) {
	if (enable) {
		return bq24295_rmw(charger, REG_POWER_ON_CFG, 0x00, 0x21);
	} else {
		return bq24295_rmw(charger, REG_POWER_ON_CFG, 0x20, 0x01);
	}
}

esp_err_t bq24295_set_charge_enable(bq24295_t *charger, bool enable) {
	if (enable) {
		return bq24295_rmw(charger, REG_POWER_ON_CFG, 0x00, 0x41);
	} else {
		return bq24295_rmw(charger, REG_POWER_ON_CFG, 0x40, 0x01);
	}
}

esp_err_t bq24295_set_min_system_voltage(bq24295_t *charger, unsigned int voltage_mv) {
	if (voltage_mv < 3000 || voltage_mv > 3700) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t bits_set = 0;
	unsigned int voltage_delta_mv = voltage_mv - 3000;
	if (voltage_delta_mv >= 400) {
		bits_set |= 1 << 3;
		voltage_delta_mv -= 400;
	}
	if (voltage_delta_mv >= 200) {
		bits_set |= 1 << 2;
		voltage_delta_mv -= 200;
	}
	if (voltage_delta_mv >= 100) {
		bits_set |= 1 << 1;
		voltage_delta_mv -= 100;
	}

	return bq24295_rmw(charger, REG_POWER_ON_CFG, 0x0e, 0x01 | bits_set);
}

esp_err_t bq24295_watchdog_reset(bq24295_t *charger) {
	return bq24295_rmw(charger, REG_POWER_ON_CFG, 0x00, 0x41);
}

esp_err_t bq24295_set_charge_current(bq24295_t *charger, unsigned int charge_current_ma) {
	if (charge_current_ma < 512 || charge_current_ma > 3008) {
		return ESP_ERR_INVALID_ARG;
	}
	unsigned int charge_current_scaled = DIV_ROUND(charge_current_ma - 512, 64);
	unsigned int charge_current_bits = charge_current_scaled << 2;

	return bq24295_rmw(charger, REG_CHARGE_CURRENT, 0xfc, charge_current_bits);
}

esp_err_t bq24295_set_termination_current(bq24295_t *charger, unsigned int term_current_ma) {
	if (term_current_ma < 128 || term_current_ma > 2048) {
		return ESP_ERR_INVALID_ARG;
	}
	unsigned int term_current_scaled = DIV_ROUND(term_current_ma - 128, 128);
	unsigned int term_current_bits = term_current_scaled << 0;

	return bq24295_rmw(charger, REG_PRE_CHARGE_TERM, 0x0f, term_current_bits);
}

esp_err_t bq24295_set_precharge_current(bq24295_t *charger, unsigned int pchg_current_ma) {
	if (pchg_current_ma < 128 || pchg_current_ma > 2048) {
		return ESP_ERR_INVALID_ARG;
	}
	unsigned int pchg_current_scaled = DIV_ROUND(pchg_current_ma - 128, 128);
	unsigned int pchg_current_bits = pchg_current_scaled << 4;

	return bq24295_rmw(charger, REG_PRE_CHARGE_TERM, 0xf0, pchg_current_bits);
}

esp_err_t bq24295_set_watchdog_timeout(bq24295_t *charger, bq24295_watchdog_timeout_t timeout) {
	return bq24295_rmw(charger, REG_TERM_TIMER, 0x70, ((uint8_t)timeout) << 4);
}

esp_err_t bq24295_set_boost_voltage(bq24295_t *charger, unsigned int boost_voltage_mv) {
	if (boost_voltage_mv < 4550 || boost_voltage_mv > 5510) {
		return ESP_ERR_INVALID_ARG;
	}
	unsigned int boost_voltage_scaled = DIV_ROUND(boost_voltage_mv - 4550, 64);
	unsigned int boost_voltage_bits = boost_voltage_scaled << 4;

	return bq24295_rmw(charger, REG_BOOST_THERMAL, 0xf0, boost_voltage_bits);
}

esp_err_t bq24295_set_battery_low_threshold(bq24295_t *charger, bq24295_battery_low_threshold_t threshold) {
	switch (threshold) {
	case BQ24295_BATTERY_LOW_THRESHOLD_3_0V:
		return bq24295_rmw(charger, REG_CHARGE_CTRL, 0x00, 1 << 1);
	case BQ24295_BATTERY_LOW_THRESHOLD_2_8V:
		return bq24295_rmw(charger, REG_CHARGE_CTRL, 1 << 1, 0x00);
	default:
		return ESP_ERR_INVALID_ARG;
	}
}

esp_err_t bq24295_set_recharge_threshold(bq24295_t *charger, bq24295_recharge_threshold_t threshold) {
	switch (threshold) {
	case BQ24295_RECHARGE_THRESHOLD_100MV:
		return bq24295_rmw(charger, REG_CHARGE_CTRL, 1 << 0, 0x00);
	case BQ24295_RECHARGE_THRESHOLD_300MV:
		return bq24295_rmw(charger, REG_CHARGE_CTRL, 0x00, 1 << 0);
	default:
		return ESP_ERR_INVALID_ARG;
	}
}

esp_err_t bq24295_is_charging(bq24295_t *charger, bool *is_charging) {
	uint8_t system_status;
	esp_err_t err = bq24295_r(charger, REG_SYSTEM_STATUS, &system_status);
	if (err) {
		return err;
	}
	bq24295_charge_status_t charge_status = (system_status >> 4) & 0x3;
	*is_charging =
		(charge_status == BQ24295_CHARGE_STATUS_PRE_CHARGE) ||
		(charge_status == BQ24295_CHARGE_STATUS_FAST_CHARGING);
	return ESP_OK;
}

esp_err_t bq24295_is_power_good(bq24295_t *charger, bool *power_good) {
	uint8_t system_status;
	esp_err_t err = bq24295_r(charger, REG_SYSTEM_STATUS, &system_status);
	if (err) {
		return err;
	}
	*power_good = !!((system_status >> 2) & 0x1);
	return ESP_OK;
}
