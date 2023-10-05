#include "bq24295.h"

#include "util.h"

#define BQ24295_ADDRESS	0x6B

#define REG_INPUT_CTRL		0x00
#define REG_POWER_ON_CFG	0x01
#define REG_CHARGE_CURRENT	0x02
#define REG_PRE_CHARGE_TERM	0x03
#define REG_TERM_TIMER		0x05
#define REG_BOOST_THERMAL	0x06
#define REG_MISC_CTRL		0x07

esp_err_t bq24295_init(bq24295_t *charger, i2c_bus_t *bus) {
	charger->i2c_bus = bus;

	return ESP_OK;
}

static esp_err_t bq24295_rmw(bq24295_t *charger, uint8_t reg, uint8_t clear, uint8_t set) {
	uint8_t val;
	esp_err_t err = i2c_bus_read_byte(charger->i2c_bus, BQ24295_ADDRESS, reg, &val);
	if (err) {
		return err;
	}
	val &= ~clear;
	val |= set;
	return i2c_bus_write_byte(charger->i2c_bus, BQ24295_ADDRESS, reg, val);
}

esp_err_t bq24295_set_input_current_limit(bq24295_t *charger, unsigned int current_ma) {
	if (current_ma < 100) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t limit_bits = 0;
	if (current_ma >= 3000) {
		limit_bits = 0x07;
	} else if (current_ma >= 2000) {
		limit_bits = 0x06;
	} else if (current_ma >= 1500) {
		limit_bits = 0x05;
	} else if (current_ma >= 1000) {
		limit_bits = 0x04;
	} else if (current_ma >= 900) {
		limit_bits = 0x03;
	} else if (current_ma >= 500) {
		limit_bits = 0x02;
	} else if (current_ma >= 150) {
		limit_bits = 0x01;
	}

	return bq24295_rmw(charger, REG_INPUT_CTRL, 0x07, limit_bits);
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
