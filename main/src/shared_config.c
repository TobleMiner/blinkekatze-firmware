#include "shared_config.h"

#include <string.h>

#include <esp_timer.h>

#include "neighbour.h"

bool shared_config_update_remote(shared_config_t *config, const void *hdr) {
	shared_config_hdr_t cfg_hdr;
	memcpy(&cfg_hdr, hdr, sizeof(cfg_hdr));

	if (cfg_hdr.config_timestamp_global >= config->config_timestamp_global) {
		config->last_config_rx_timestamp_local = esp_timer_get_time();
	}

	bool cfg_update = cfg_hdr.config_timestamp_global > config->config_timestamp_global;
	if (cfg_update) {
		config->config_timestamp_global = cfg_hdr.config_timestamp_global;
	}
	return cfg_update;
}

void shared_config_update_local(shared_config_t *config) {
	config->config_timestamp_global = neighbour_get_global_clock();
}

bool shared_config_should_tx(shared_config_t *config) {
	if (!config->config_timestamp_global) {
		return false;
	}

	int64_t now = esp_timer_get_time();
	int64_t ms_since_last_rx = (now - config->last_config_rx_timestamp_local) / 1000LL;
	if (ms_since_last_rx < SHARED_CONFIG_MIN_TX_INTERVAL_MS * 2) {
		return false;
	}

	int64_t ms_since_last_tx = (now - config->last_config_tx_timestamp_local) / 1000LL;
	return ms_since_last_tx >= SHARED_CONFIG_MIN_TX_INTERVAL_MS;
}

void shared_config_hdr_init(const shared_config_t *config, void *hdr) {
	shared_config_hdr_t cfg_hdr = {
		config->config_timestamp_global
	};
	memcpy(hdr, &cfg_hdr, sizeof(cfg_hdr));
}

void shared_config_tx_done(shared_config_t *config) {
	config->last_config_tx_timestamp_local = esp_timer_get_time();
}
