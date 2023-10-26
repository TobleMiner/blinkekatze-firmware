#pragma once

#include <stdint.h>

#include <esp_err.h>

#include "color.h"
#include "lis3dh.h"
#include "neighbour_rssi_delay_model.h"
#include "neighbour.h"
#include "shared_config.h"
#include "wireless.h"

#define BONK_MAX_INTENSITY 1000
#define BONK_NUM_EVENTS	   100

typedef struct bonk_event {
	int64_t timestamp_us;
	uint32_t magnitude;
} bonk_event_t;

typedef struct bonk {
	lis3dh_t *accel;
	shared_config_t shared_cfg;
	uint32_t magnitude;
	bonk_event_t bonks[BONK_NUM_EVENTS];
	unsigned int bonk_write_pos;
	neighbour_rssi_delay_model_t delay_model;
	uint16_t bonk_duration_ms;
	bool enable;
	bool enable_decay;
	bool enable_delay;
} bonk_t;

void bonk_init(bonk_t *bonk, lis3dh_t *accel);
esp_err_t bonk_update(bonk_t *bonk);
void bonk_rx(bonk_t *bonk, const wireless_packet_t *packet, const neighbour_t *neigh);
unsigned int bonk_get_intensity(const bonk_t *bonk);
void bonk_apply(bonk_t *bonk, color_hsv_t *color);
void bonk_set_enable(bonk_t *bonk, bool enable);
void bonk_set_decay_enable(bonk_t *bonk, bool enable);
void bonk_set_delay_enable(bonk_t *bonk, bool enable);
void bonk_set_duration(bonk_t *bonk, unsigned int duration);
void bonk_set_rssi_delay_model(bonk_t *bonk, const neighbour_rssi_delay_model_t *model);
