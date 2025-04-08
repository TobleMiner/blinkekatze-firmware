#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <driver/rmt_tx.h>
#include <driver/spi_master.h>
#include <esp_err.h>

#include "platform.h"

typedef struct platform_tallylight_v2 {
	platform_t base;
	rmt_channel_handle_t rmt_channel;
	rmt_encoder_handle_t rmt_encoder;
	uint8_t led_color_data[6];
} platform_tallylight_v2_t;

esp_err_t platform_tallylight_v2_probe(platform_t **ret);
