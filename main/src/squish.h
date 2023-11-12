#pragma once

#include <stdint.h>

#include <esp_err.h>

#include "color.h"
#include "neighbour.h"
#include "scheduler.h"
#include "spl06.h"
#include "wireless.h"

#define REMOTE_SQUISH_BUFFER_SIZE	100

typedef struct squish_remote {
	int64_t timestamp_us;
	uint16_t squish;
} squish_remote_t;

typedef struct squish {
	spl06_t *baro;
	int64_t pressure_at_rest_milli;
	unsigned int num_pressure_samples;
	unsigned int local_squishedness;
	unsigned int squishedness;
	int64_t timestamp_last_update_us;
	squish_remote_t remote_squishes[REMOTE_SQUISH_BUFFER_SIZE];
	unsigned int remote_squish_write_pos;
	int64_t timestamp_last_tx_us;
	bool peak_not_sent;
	scheduler_task_t update_task;
} squish_t;

void squish_init(squish_t *squish, spl06_t *baro);
void squish_apply(const squish_t *squish, color_hsv_t *color);
void squish_rx(squish_t *squish, const wireless_packet_t *packet, const neighbour_t *neigh);
