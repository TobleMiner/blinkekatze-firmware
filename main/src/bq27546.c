#include "bq27546.h"

#include <endian.h>
#include <stdint.h>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "util.h"

#define CMD_CONTROL			0x00
#define CMD_TEMPERATURE_0_1K		0x06
#define CMD_CELL_VOLTAGE_MV		0x08
#define CMD_AVERAGE_CURRENT_MA		0x14
#define CMD_TIME_TO_EMPTY		0x16
#define CMD_FULL_CHARGE_CAPACITY	0x18
#define CMD_REMAINING_CAPACITY		0x22
#define CMD_STATE_OF_CHARGE		0x2c
#define CMD_STATE_OF_HEALTH		0x2e

#define SUBCMD_CONTROL_STATUS		0x0000
#define 	CONTROL_STATUS_SEALED	(1 << 13)
#define SUBCMD_DEVICE_TYPE		0x0001
#define SUBCMD_FW_VERSION		0x0002
#define SUBCMD_HW_VERSION		0x0003
#define SUBCMD_SEALED			0x0020
#define SUBCMD_IT_ENABLE		0x0021

#define ADDRESS		0x55
#define CHIP_ID		0x0546

static const char *TAG = "BQ27546";

static void lock(bq27546_t *bq) {
	xSemaphoreTake(bq->lock, portMAX_DELAY);
}

static void unlock(bq27546_t *bq) {
	xSemaphoreGive(bq->lock);
}

static esp_err_t run_subcmd(bq27546_t *bq, uint8_t cmd, uint16_t subcmd) {
	uint8_t buf[3] = { cmd };

	le16enc(&buf[1], subcmd);
	return i2c_bus_write(bq->i2c_bus, ADDRESS, buf, sizeof(buf));
}

static esp_err_t read_word_subcmd(bq27546_t *bq, uint8_t cmd, uint16_t subcmd, uint16_t *word) {
	esp_err_t err;
	uint8_t buf[3] = { cmd };

	le16enc(&buf[1], subcmd);
	lock(bq);
	err = i2c_bus_write(bq->i2c_bus, ADDRESS, buf, sizeof(buf));
	if (err) {
		unlock(bq);
		return err;
	}

	err = i2c_bus_write_then_read(bq->i2c_bus, ADDRESS, &cmd, 1, buf, 2);
	unlock(bq);
	if (!err) {
		*word = le16dec(buf);
	}
	return err;
}

static esp_err_t read_word(bq27546_t *bq, uint8_t cmd, uint16_t *word) {
	esp_err_t err;
	uint8_t buf[2];

	lock(bq);
	err = i2c_bus_write_then_read(bq->i2c_bus, ADDRESS, &cmd, 1, buf, sizeof(buf));
	unlock(bq);
	if (!err) {
		*word = le16dec(buf);
	}
	return err;
}

esp_err_t bq27546_init(bq27546_t *bq, i2c_bus_t *bus) {
	uint16_t gauge_id, fw_version, hw_version;

	bq->i2c_bus = bus;
	bq->lock = xSemaphoreCreateMutexStatic(&bq->lock_buffer);

	esp_err_t err = read_word_subcmd(bq, CMD_CONTROL, SUBCMD_DEVICE_TYPE, &gauge_id);
	if (err) {
		ESP_LOGE(TAG, "Failed to read gauge id: %d", err);
		return err;
	}

	if (gauge_id != CHIP_ID) {
		ESP_LOGE(TAG, "Invalid gauge id, expected 0x%04x but got 0x%04x", CHIP_ID, gauge_id);
		return ESP_ERR_NOT_SUPPORTED;
	}

	err = read_word_subcmd(bq, CMD_CONTROL, SUBCMD_FW_VERSION, &fw_version);
	if (err) {
		ESP_LOGE(TAG, "Failed to read gauge firmware version: %d", err);
		return err;
	}

	err = read_word_subcmd(bq, CMD_CONTROL, SUBCMD_HW_VERSION, &hw_version);
	if (err) {
		ESP_LOGE(TAG, "Failed to read gauge hardware version: %d", err);
		return err;
	}

	ESP_LOGI(TAG, "BQ27546 battery gauge @0x%02x, hardware version %u, firmware version %u", ADDRESS, hw_version, fw_version);

	return 0;
}

esp_err_t bq27546_write_flash_execute_(bq27546_t *bq, const bq27546_flash_op_t *flash_ops, size_t num_ops, uint8_t *read_buffer) {
	esp_err_t err;
	for (size_t i = 0; i < num_ops; i++) {
		const bq27546_flash_op_t *op = &flash_ops[i];

		switch (op->type) {
		case BQ27546_FLASH_CMD_WRITE:
			ESP_LOGI(TAG, "Writing %lu bytes to 0x%02x...", (unsigned long)op->compare.len, op->write.i2c_address);
			ESP_LOG_BUFFER_HEXDUMP(TAG, op->write.data, op->write.len, ESP_LOG_INFO);
			err = i2c_bus_soft_write(bq->i2c_bus, op->write.i2c_address, op->write.data, op->write.len, 10000);
			if (err) {
				ESP_LOGE(TAG, "Failed writing to gauge, op %lu, %d", (unsigned long)i, err);
				return err;
			}
			break;
		case BQ27546_FLASH_CMD_COMPARE:
			ESP_LOGI(TAG, "Reading %lu bytes from 0x%02x@0x%02x...", (unsigned long)op->compare.len, op->compare.i2c_address, op->compare.reg);
			memset(read_buffer, 0x55, op->compare.len);
			err = i2c_bus_soft_write_then_read(bq->i2c_bus, op->compare.i2c_address, &op->compare.reg, 1, read_buffer, op->compare.len, 10000);
			if (err) {
				ESP_LOGE(TAG, "Failed reading from gauge, op %lu: %d", (unsigned long)i, err);
				return err;
			}
			ESP_LOG_BUFFER_HEXDUMP(TAG, read_buffer, op->compare.len, ESP_LOG_INFO);
			if (memcmp(op->compare.data, read_buffer, op->compare.len)) {
				ESP_LOGE(TAG, "Data read back does not match, op %lu", (unsigned long)i);
				return ESP_FAIL;
			}
			break;
		case BQ27546_FLASH_CMD_WAIT:
			ESP_LOGI(TAG, "Sleeping for %lu ms...", (unsigned long)op->wait.delay_ms);
			vTaskDelay(pdMS_TO_TICKS(op->wait.delay_ms));
			break;
		default:
			ESP_LOGE(TAG, "Unknown flash operation 0x%02x, op %lu", op->type, (unsigned long)i);
			return ESP_FAIL;
		}
		/* Wait one tick to ensure watchdog stays happy */
		vTaskDelay(1);
	}

	return ESP_OK;
}

esp_err_t bq27546_write_flash(bq27546_t *bq, const bq27546_flash_op_t *flash_ops, size_t num_ops) {
	size_t read_buffer_size = 0;
	for (size_t i = 0; i < num_ops; i++) {
		const bq27546_flash_op_t *op = &flash_ops[i];

		if (op->type == BQ27546_FLASH_CMD_COMPARE) {
			read_buffer_size = MAX(read_buffer_size, op->compare.len);
		}
	}

	uint8_t *read_buffer = NULL;
	if (read_buffer_size) {
		read_buffer = malloc(read_buffer_size);
		if (!read_buffer) {
			return ESP_ERR_NO_MEM;
		}
	}

	lock(bq);
	i2c_bus_enter_soft_exclusive(bq->i2c_bus);
	esp_err_t err = bq27546_write_flash_execute_(bq, flash_ops, num_ops, read_buffer);
	i2c_bus_leave_soft_exclusive(bq->i2c_bus);
	unlock(bq);

	if (read_buffer) {
		free(read_buffer);
	}

	return err;
}

static int bq27546_get_unsigned_param(bq27546_t *bq, uint8_t cmd) {
	uint16_t param;
	esp_err_t err;

	err = read_word(bq, cmd, &param);
	if (err) {
		if (err > 0) {
			err = -err;
		}
		return err;
	}

	return param;
}

int bq27546_get_voltage_mv(bq27546_t *bq) {
	return bq27546_get_unsigned_param(bq, CMD_CELL_VOLTAGE_MV);
}

esp_err_t bq27546_get_current_ma(bq27546_t *bq, int *current_ma_out) {
	esp_err_t err;
	uint16_t ucurrent_ma;
	int16_t current_ma;

	err = read_word(bq, CMD_AVERAGE_CURRENT_MA, &ucurrent_ma);
	if (err) {
		return err;
	}
	current_ma = (int16_t)ucurrent_ma;
	*current_ma_out = current_ma;
	return 0;
}

int bq27546_get_state_of_charge_percent(bq27546_t *bq) {
	int val = bq27546_get_unsigned_param(bq, CMD_STATE_OF_CHARGE);

	if (val > 100) {
		return -ESP_ERR_INVALID_RESPONSE;
	}
	return val;
}

int bq27546_get_state_of_health_percent(bq27546_t *bq) {
	int val = bq27546_get_unsigned_param(bq, CMD_STATE_OF_HEALTH);

	if (val > 100) {
		return -ESP_ERR_INVALID_RESPONSE;
	}
	return val;
}

int bq27546_get_time_to_empty_min(bq27546_t *bq) {
	return bq27546_get_unsigned_param(bq, CMD_TIME_TO_EMPTY);
}

int bq27546_get_temperature_0_1k(bq27546_t *bq) {
	return bq27546_get_unsigned_param(bq, CMD_TEMPERATURE_0_1K);
}


int bq27546_get_full_charge_capacity_mah(bq27546_t *bq) {
	return bq27546_get_unsigned_param(bq, CMD_FULL_CHARGE_CAPACITY);
}

int bq27546_get_remaining_capacity_mah(bq27546_t *bq) {
	return bq27546_get_unsigned_param(bq, CMD_REMAINING_CAPACITY);
}

int bq27546_is_sealed(bq27546_t *bq) {
	uint16_t control_status;
	esp_err_t err = read_word_subcmd(bq, CMD_CONTROL, SUBCMD_CONTROL_STATUS, &control_status);
	if (err) {
		return err > 0 ? -err : err;
	}

	return !!(control_status & CONTROL_STATUS_SEALED);
}

esp_err_t bq27546_seal(bq27546_t *bq) {
	return run_subcmd(bq, CMD_CONTROL, SUBCMD_SEALED);
}

esp_err_t bq27546_it_enable(bq27546_t *bq) {
	return run_subcmd(bq, CMD_CONTROL, SUBCMD_IT_ENABLE);
}
