#pragma once

#include <stdint.h>

#include <esp_err.h>

#include "lis3dh.h"
#include "neighbour.h"
#include "wireless.h"

typedef struct bonk {
	lis3dh_t *accel;
	int64_t last_bonk_us;
	uint32_t last_bonk_magnitude;
} bonk_t;

#define BONK_MAX_INTENSITY 1000

void bonk_init(bonk_t *bonk, lis3dh_t *accel);
esp_err_t bonk_update(bonk_t *bonk);
void bonk_rx(bonk_t *bonk, const wireless_packet_t *packet, const neighbour_t *neigh);
unsigned int bonk_get_intensity(const bonk_t *bonk);
