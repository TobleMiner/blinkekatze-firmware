#include "ota.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdint.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>

#include <esp_image_format.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/if_api.h>

#include "neighbour.h"
#include "neighbour_static_info.h"
#include "scheduler.h"
#include "tcp_client.h"
#include "tcp_memory_server.h"

#define OTA_UPDATE_SERVE_INTERVAL_MS	10000
#define OTA_UPDATE_DOWNLOAD_STALL_MS	5000
#define OTA_UPDATE_PROGRESS_INTERVAL_MS	1000
#define OTA_UPDATE_BLINK_INTERVAL_MS	500
#define OTA_STA_CONNECT_TIMEOUT_MS	10000
#define OTA_STATUS_TIMEOUT_MS		5000

typedef enum ota_state {
	OTA_STATE_IDLE,
	OTA_STATE_SERVING,
	OTA_STATE_STATION_CONNECT,
	OTA_STATE_STATION_CONNECTING,
	OTA_STATE_DISCOVER_SERVER,
	OTA_STATE_DOWNLOAD_IN_PROGRESS,
	OTA_STATE_FINISHED
} ota_state_t;

static const char * ota_state_strings[] = {
	[OTA_STATE_IDLE] = "OTA_STATE_IDLE",
	[OTA_STATE_SERVING] = "OTA_STATE_SERVING",
	[OTA_STATE_STATION_CONNECT] = "OTA_STATE_STATION_CONNECT",
	[OTA_STATE_STATION_CONNECTING] = "OTA_STATE_STATION_CONNECTING",
	[OTA_STATE_DISCOVER_SERVER] = "OTA_STATE_DISCOVER_SERVER",
	[OTA_STATE_DOWNLOAD_IN_PROGRESS] = "OTA_STATE_DOWNLOAD_IN_PROGRESS",
	[OTA_STATE_FINISHED] = "OTA_STATE_FINISHED"
};

typedef struct ota {
	bool ignore_version;
	const void *firmware_mmap_ptr;
	size_t firmware_size;
	esp_partition_mmap_handle_t firmware_mmap_handle;
	ota_state_t state;
	int64_t last_tx_timestamp_us;
	uint8_t update_peer[ESP_NOW_ETH_ALEN];
	int ap_ifindex;
	int sta_ifindex;
	size_t bytes_transfered;
	int64_t last_download_progress_timestamp_us;
	int64_t sta_connect_timestamp_us;
	portMUX_TYPE tcp_client_lock;
	esp_ota_handle_t ota_handle;
	size_t update_size;
	tcp_memory_server_t tcp_server;
	tcp_client_t tcp_client;
	const esp_partition_t *update_part;
	scheduler_task_t update_task;
} ota_t;

typedef enum ota_packet_type {
	OTA_PACKET_TYPE_INIT = 0,
	OTA_PACKET_TYPE_PROGRESS = 1
} ota_packet_type_t;

typedef struct ota_packet {
	uint8_t packet_type;
	uint8_t ota_packet_type;
	union {
		struct {
			uint32_t firmware_size;
		} init;
		struct {
			uint32_t download_size;
			uint32_t download_progress;
		} progress;
	};
} __attribute__((packed)) ota_packet_t;

static const char *TAG = "ota";

static ota_t ota;

static esp_err_t patition_get_image_size(const esp_partition_t *part, size_t *size_out) {
	esp_partition_pos_t pos = {
		.offset = part->address,
		.size = part->size
	};
	esp_image_metadata_t meta = {
		.start_addr = part->address
	};
	esp_err_t err = esp_image_verify(ESP_IMAGE_VERIFY, &pos, &meta);
	if (!err) {
		*size_out = meta.image_len;
	}
	return err;
}

static void ota_announce_serve(void) {
	int64_t now = esp_timer_get_time();
	int32_t delta_ms = (now - ota.last_tx_timestamp_us) / 1000;
	if (delta_ms >= OTA_UPDATE_SERVE_INTERVAL_MS || !ota.last_tx_timestamp_us) {
		ota_packet_t ota_packet = { 0 };
		ota_packet.packet_type = WIRELESS_PACKET_TYPE_OTA;
		ota_packet.ota_packet_type = OTA_PACKET_TYPE_INIT;
		ota_packet.init.firmware_size = ota.firmware_size;
		wireless_broadcast((const uint8_t *)&ota_packet, sizeof(ota_packet));
		ota.last_tx_timestamp_us = now;
	}
}

static esp_err_t ota_initiate_station_connection(uint8_t *update_address) {
	const neighbour_t *neigh = neighbour_find_by_address(update_address);
	if (!neigh) {
		return ESP_ERR_NOT_FOUND;
	}

	wifi_config_t cfg = { 0 };
	if (!neighbour_static_info_get_ap_password(neigh, (char *)cfg.sta.password, sizeof(cfg.sta.password))) {
		return ESP_ERR_NOT_FOUND;
	}
	neighbour_static_info_get_ap_ssid(neigh, (char *)cfg.sta.ssid, sizeof(cfg.sta.ssid));
	ESP_LOGI(TAG, "Connecting to peer as station...");
	esp_err_t err = wireless_connect_to_ap(&cfg);
	if (err) {
		ESP_LOGE(TAG, "Failed to connect to AP, going back to idle state");
		ota.state = OTA_STATE_IDLE;
	} else {
		ota.sta_connect_timestamp_us = esp_timer_get_time();
		ESP_LOGI(TAG, "Connection init ok, waiting for UDP announcement...");
		ota.state = OTA_STATE_STATION_CONNECTING;
	}

	return err;
}

static esp_err_t ota_wait_station_connected() {
	int64_t now = esp_timer_get_time();
	int32_t delta_ms = (now - ota.sta_connect_timestamp_us) / 1000LL;

	if (wireless_is_sta_connected()) {
		ESP_LOGI(TAG, "Connected to AP, starting server discovery...");
		ota.state = OTA_STATE_DISCOVER_SERVER;
	} else if (delta_ms >= OTA_STA_CONNECT_TIMEOUT_MS) {
		ESP_LOGE(TAG, "Failed to connect, going back to idle state");
		wireless_disconnect_from_ap();
		ota.state = OTA_STATE_IDLE;
		return ESP_ERR_TIMEOUT;
	}

	return ESP_OK;
}

static esp_err_t ota_tcp_client_init(void *priv) {
	const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
	if (!update_part) {
		ESP_LOGE(TAG, "Failed to determine update partition");
		return ESP_FAIL;
	}
	ota.update_part = update_part;
	esp_err_t err = esp_ota_begin(update_part, ota.update_size, &ota.ota_handle);
	if (err) {
		ESP_LOGE(TAG, "Failed to start OTA update: %d", err);
	}
	return err;
}

static esp_err_t ota_tcp_client_finish(void *priv) {
	esp_err_t err = tcp_client_get_cb_err(&ota.tcp_client);
	if (err) {
		ESP_LOGE(TAG, "HTTP download failed: %d", err);
		esp_ota_abort(ota.ota_handle);
	} else {
		ESP_LOGI(TAG, "HTTP download done");
		err = esp_ota_end(ota.ota_handle);
		if (err) {
			ESP_LOGE(TAG, "OTA verification failed: %d", err);
			return err;
		} else {
			err = esp_ota_set_boot_partition(ota.update_part);
			if (err) {
				ESP_LOGE(TAG, "Failed to set boot partition: %d", err);
				return err;
			} else {
				ESP_LOGI(TAG, "OTA successful");
			}
		}
	}
	return ESP_OK;
}

static esp_err_t ota_tcp_client_data_handler(void *priv, const void *data, size_t len) {
	taskENTER_CRITICAL(&ota.tcp_client_lock);
	ota.bytes_transfered += len;
	ota.last_download_progress_timestamp_us = esp_timer_get_time();
	taskEXIT_CRITICAL(&ota.tcp_client_lock);
	ESP_LOGD(TAG, "Chunk, len %u, total size %u", len, ota.bytes_transfered);
	esp_err_t err = esp_ota_write(ota.ota_handle, data, len);
	if (err) {
		ESP_LOGE(TAG, "Failed to write OTA firmware chunk: %d", err);
		return err;
	}

	if (ota.bytes_transfered == ota.update_size) {
		tcp_client_cancel(&ota.tcp_client);
	}

	return ESP_OK;
}

static esp_err_t ota_start_download(void) {
	ota.last_download_progress_timestamp_us = esp_timer_get_time();
	ota.bytes_transfered = 0;

	struct ifreq ifr;
	lwip_if_indextoname(ota.sta_ifindex, ifr.ifr_name);
	struct in_addr inaddr;
	inet_pton(AF_INET, "192.168.4.1", &inaddr);
	return tcp_client_init(&ota.tcp_client, &inaddr, 1337, &ifr, ota_tcp_client_init, ota_tcp_client_data_handler, ota_tcp_client_finish, NULL);
}

static esp_err_t ota_discover_server() {
	esp_err_t err = ota_start_download();
	if (err) {
		ESP_LOGE(TAG, "Failed to start OTA download: %d", err);
	} else {
		ota.state = OTA_STATE_DOWNLOAD_IN_PROGRESS;
	}
	return ESP_OK;
}

static esp_err_t ota_supervise_download_progress() {
	int64_t now = esp_timer_get_time();
	int32_t delta_ms = (now - ota.last_tx_timestamp_us) / 1000;
	if (delta_ms >= OTA_UPDATE_PROGRESS_INTERVAL_MS) {
		ota_packet_t ota_packet = { 0 };
		ota_packet.packet_type = WIRELESS_PACKET_TYPE_OTA;
		ota_packet.ota_packet_type = OTA_PACKET_TYPE_PROGRESS;
		ota_packet.progress.download_size = ota.update_size;
		ota_packet.progress.download_progress = ota.bytes_transfered;
		wireless_broadcast((const uint8_t *)&ota_packet, sizeof(ota_packet));
		ota.last_tx_timestamp_us = now;
	}

	if (tcp_client_is_done(&ota.tcp_client)) {
		ota.state = OTA_STATE_FINISHED;
		if (tcp_client_get_err(&ota.tcp_client) || tcp_client_get_cb_err(&ota.tcp_client)) {
			ESP_LOGI(TAG, "OTA failed");
			return ESP_FAIL;
		} else {
			ESP_LOGI(TAG, "OTA succeeded");
		}
	}
	return ESP_OK;
/*
	int64_t now = esp_timer_get_time();
	taskENTER_CRITICAL(&ota.http_client_lock);
	int64_t last_download_progress_timestamp_us = ota.last_download_progress_timestamp_us;
	taskEXIT_CRITICAL(&ota.http_client_lock);
	int32_t delta_ms = (now - last_download_progress_timestamp_us) / 1000LL;

	if (delta_ms >= OTA_UPDATE_DOWNLOAD_STALL_MS) {
		ESP_LOGI(TAG, "Download timeout");
	}

*/
}

static esp_err_t ota_handle_done(void) {
	if (tcp_client_get_err(&ota.tcp_client) || tcp_client_get_cb_err(&ota.tcp_client)) {
		ota.state = OTA_STATE_IDLE;
		return ESP_OK;
	} else {
		esp_restart();
		/* Must not return */
		return ESP_FAIL;
	}
}

static esp_err_t ota_update_() {
	switch (ota.state) {
	case OTA_STATE_SERVING:
		ota_announce_serve();
		return ESP_OK;
	case OTA_STATE_STATION_CONNECT:
		return ota_initiate_station_connection(ota.update_peer);
	case OTA_STATE_STATION_CONNECTING:
		return ota_wait_station_connected();
	case OTA_STATE_DISCOVER_SERVER:
		return ota_discover_server();
	case OTA_STATE_DOWNLOAD_IN_PROGRESS:
		return ota_supervise_download_progress();
	case OTA_STATE_FINISHED:
		return ota_handle_done();
	default:
	}

	return ESP_OK;
}

static void ota_update(void *arg);
static void ota_update(void *arg) {
	ota_update_();
	scheduler_schedule_task_relative(&ota.update_task, ota_update, NULL, MS_TO_US(1000));
}

esp_err_t ota_init() {
	memset(&ota, 0, sizeof(ota));
	ota.tcp_client_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
	const esp_partition_t *booted_part = esp_ota_get_running_partition();
	if (!booted_part) {
		ESP_LOGE(TAG, "Failed to determine booted partition");
		return ESP_ERR_NOT_FOUND;
	}

	esp_err_t err = esp_partition_mmap(booted_part, 0, booted_part->size, ESP_PARTITION_MMAP_DATA, &ota.firmware_mmap_ptr, &ota.firmware_mmap_handle);
	if (err) {
		return err;
	}

	err = patition_get_image_size(booted_part, &ota.firmware_size);
	if (err) {
		ESP_LOGW(TAG, "Failed to determine actual firmware size, using full partition size");
		ota.firmware_size = booted_part->size;
	} else {
		ESP_LOGI(TAG, "Size of booted image: %lu", (unsigned long)ota.firmware_size);
	}

	ota.ap_ifindex = wireless_get_ap_ifindex();
	ota.sta_ifindex = wireless_get_sta_ifindex();

	struct ifreq ifr;
	lwip_if_indextoname(ota.ap_ifindex, ifr.ifr_name);
	err = tcp_memory_server_init(&ota.tcp_server, ota.firmware_mmap_ptr, ota.firmware_size, 1337, &ifr);
	if (err) {
		ESP_LOGE(TAG, "Failed to initialize TCP update server");
		return err;
	}

	scheduler_task_init(&ota.update_task);
	scheduler_schedule_task_relative(&ota.update_task, ota_update, NULL, 0);

	return ESP_OK;
}

static esp_err_t handle_ota_start_packet(const ota_packet_t *ota_packet, const wireless_packet_t *packet, const neighbour_t *neigh) {
	if (ota.state == OTA_STATE_IDLE) {
		ESP_LOGD(TAG, "Got OTA init from "MACSTR", upate size: %lu bytes",
			 MAC2STR(packet->src_addr),
			 (unsigned long)ota_packet->init.firmware_size);
		if (neigh) {
			const uint8_t *neighbour_firmware_hash = neighbour_static_info_get_firmware_sha256_hash(neigh);
			if (neighbour_firmware_hash) {
				const esp_app_desc_t *app_desc = esp_app_get_description();
				const uint8_t *local_firmware_hash = app_desc->app_elf_sha256;
				if (memcmp(local_firmware_hash, neighbour_firmware_hash, 32) || ota.ignore_version) {
					ESP_LOGI(TAG, "Remote firmware is different from local firmware, commencing update");
					memcpy(ota.update_peer, packet->src_addr, ESP_NOW_ETH_ALEN);
					ota.update_size = ota_packet->init.firmware_size;
					ota.state = OTA_STATE_STATION_CONNECT;
				} else {
					ESP_LOGD(TAG, "Ignoring OTA init, neighbour firmware identical to local firmware");
				}
			} else {
				ESP_LOGI(TAG, "Ignoring OTA init, neighbour firmware version not known yet");
			}
		}
	}
	return ESP_OK;
}

static esp_err_t handle_ota_progress_packet(const ota_packet_t *ota_packet, const wireless_packet_t *packet, const neighbour_t *neigh) {
	neighbour_ota_info_t info = {
		packet->rx_timestamp,
		ota_packet->progress.download_size,
		ota_packet->progress.download_progress
	};
	if (neigh) {
		neighbour_update_ota_info(neigh, &info);
	}
	return ESP_OK;
}

esp_err_t ota_rx(const wireless_packet_t *packet, const neighbour_t *neigh) {
	ota_packet_t ota_packet;
	if (packet->len < sizeof(ota_packet)) {
		ESP_LOGD(TAG, "Got short OTA packet, expected %u bytes but got only %u bytes",
			 sizeof(ota_packet), packet->len);
		return ESP_ERR_INVALID_ARG;
	}
	memcpy(&ota_packet, packet->data, sizeof(ota_packet));

	switch (ota_packet.ota_packet_type) {
	case OTA_PACKET_TYPE_INIT:
		return handle_ota_start_packet(&ota_packet, packet, neigh);
	case OTA_PACKET_TYPE_PROGRESS:
		return handle_ota_progress_packet(&ota_packet, packet, neigh);
	default:
	}

	return ESP_OK;
}

esp_err_t ota_serve_update(bool serve) {
	if (serve) {
		if (ota.state == OTA_STATE_IDLE) {
			ota.state = OTA_STATE_SERVING;
		}

		if (ota.state != OTA_STATE_SERVING) {
			return ESP_ERR_INVALID_STATE;
		}
	} else {
		if (ota.state == OTA_STATE_SERVING) {
			ota.state = OTA_STATE_IDLE;
		}

		if (ota.state != OTA_STATE_IDLE) {
			return ESP_ERR_INVALID_STATE;
		}
	}

	return ESP_OK;
}

void ota_indicate_update(color_hsv_t *color) {
	if (ota.state == OTA_STATE_DOWNLOAD_IN_PROGRESS) {
		int64_t now = neighbour_get_global_clock();
		int32_t now_ms = now / 1000LL;
		unsigned int cycle_ms = now_ms % (OTA_UPDATE_BLINK_INTERVAL_MS * 2);

		if (cycle_ms >= OTA_UPDATE_BLINK_INTERVAL_MS) {
			color->h = 26788;
			color->s = 41680;
//			color->v = 64224;
		} else {
			color->h = 47595;
			color->s = 19792;
//			color->v = 62979;
		}
	}
}

const char *ota_state_to_string(ota_state_t state) {
	if (state >= ARRAY_SIZE(ota_state_strings)) {
		return "<invalid>";
	}

	return ota_state_strings[state];
}

void ota_print_status(void) {
	printf("State: %s\r\n", ota_state_to_string(ota.state));
	if (ota.state == OTA_STATE_DOWNLOAD_IN_PROGRESS) {
		taskENTER_CRITICAL(&ota.tcp_client_lock);
		size_t progress = ota.bytes_transfered;
		taskEXIT_CRITICAL(&ota.tcp_client_lock);
		printf("Progress: %lu/%lu bytes\r\n", (unsigned long)progress, (unsigned long)ota.update_size);
	}
}

void ota_set_ignore_version(bool ignore_version) {
	ota.ignore_version = ignore_version;
}

ssize_t ota_neighbour_info_to_string(const neighbour_ota_info_t *info, char *dst, size_t len) {
	int64_t now = esp_timer_get_time();
	int32_t delta_ms = (now - info->last_local_ota_progress_timestamp_us) / 1000LL;
	if (!info->last_local_ota_progress_timestamp_us || delta_ms > OTA_STATUS_TIMEOUT_MS) {
		return snprintf(dst, len, "Idle");
	}

	if (!info->update_size) {
		return snprintf(dst, len, "Updating...");
	}

	unsigned int progress_percent = DIV_ROUND_UP(info->update_progress * 100UL, info->update_size);
	return snprintf(dst, len, "Updating... %3u%%", progress_percent);
}
