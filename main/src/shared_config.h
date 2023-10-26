#pragma once

#include <stdbool.h>
#include <stdint.h>

#define SHARED_CONFIG_MIN_TX_INTERVAL_MS	10000
#define SHARED_CONFIG_TX_TIMES			3

typedef struct shared_config {
	int64_t config_timestamp_global;
	int64_t last_config_rx_timestamp_local;
	int64_t last_config_tx_timestamp_local;
} shared_config_t;

typedef struct shared_config_hdr {
	int64_t config_timestamp_global;
} __attribute__((packed)) shared_config_hdr_t;

bool shared_config_update_remote(shared_config_t *config, const void *hdr);
void shared_config_update_local(shared_config_t *config);
bool shared_config_should_tx(shared_config_t *config);
void shared_config_hdr_init(const shared_config_t *config, void *hdr);
void shared_config_tx_done(shared_config_t *config);
