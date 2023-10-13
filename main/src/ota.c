#include "ota.h"

#include <errno.h>
#include <netinet/in.h>
#include <stdint.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>

#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_timer.h>

#include "httpd.h"
#include "neighbour.h"
#include "neighbour_static_info.h"

#define OTA_UPDATE_SERVE_INTERVAL_MS	10000

typedef enum ota_state {
	OTA_STATE_IDLE,
	OTA_STATE_SERVING,
	OTA_STATE_STATION_CONNECT,
	OTA_STATE_STATION_CONNECTING,
	OTA_STATE_DOWNLOAD_IN_PROGRESS,
	OTA_STATE_FINISHED
} ota_state_t;

typedef struct ota {
	httpd_t http_server;
	const void *firmware_mmap_ptr;
	size_t firmware_size;
	esp_partition_mmap_handle_t firmware_mmap_handle;
	ota_state_t state;
	int64_t last_tx_timestamp_us;
	uint8_t update_peer[ESP_NOW_ETH_ALEN];
	int mcast_socket;
	int ap_ifindex;
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
			uint64_t bytes;
		} progress;
	};
} __attribute__((packed)) ota_packet_t;

static const char *TAG = "ota";

static ota_t ota;

static esp_err_t firmware_get_cb(struct httpd_request_ctx* ctx, void* priv) {
	return httpd_response_write(ctx, (const char *)ota.firmware_mmap_ptr, ota.firmware_size);
}

esp_err_t ota_init() {
	memset(&ota, 0, sizeof(ota));
	const esp_partition_t *booted_part = esp_ota_get_running_partition();
	if (!booted_part) {
		ESP_LOGE(TAG, "Failed to determine booted partition");
		return ESP_ERR_NOT_FOUND;
	}

	esp_err_t err = esp_partition_mmap(booted_part, 0, booted_part->size, ESP_PARTITION_MMAP_DATA, &ota.firmware_mmap_ptr, &ota.firmware_mmap_handle);
	if (err) {
		return err;
	}
	ota.firmware_size = booted_part->size;

	err = httpd_init(&ota.http_server, " ", 8);
	if (err) {
		return err;
	}

	err = httpd_add_get_handler(&ota.http_server, "/api/ota/firmware", firmware_get_cb, NULL, 0);
	if (err) {
		return err;
	}

	ota.ap_ifindex = wireless_get_ap_ifindex();
	int sock = socket(PF_INET6, SOCK_DGRAM, IPPROTO_IPV6);
	if (sock < 0) {
		ESP_LOGE(TAG, "Failed to create OTA advertisement socket\n");
		return ESP_FAIL;
	}
	struct sockaddr_in6 listen_address = { .sin6_family = AF_INET6, .sin6_port = htons(1337) };
	int ret = bind(sock, (struct sockaddr *)&listen_address, sizeof(listen_address));
	if (ret) {
		ESP_LOGE(TAG, "Failed to bind to listen address: %d", errno);
		return ESP_FAIL;
	}
	struct ipv6_mreq group;
	group.ipv6mr_interface = ota.ap_ifindex;
	inet_pton(AF_INET6, "ff01::1", &group.ipv6mr_multiaddr);
	ret = setsockopt(sock, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP, &group, sizeof(group));
	if (ret) {
		ESP_LOGE(TAG, "Failed to subscribe to multicast group: %d", errno);
		return ESP_FAIL;
	}
	const struct timeval rx_timeout = {
		.tv_sec = 0,
		.tv_usec = 10
	};
	if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rx_timeout, sizeof(rx_timeout))) {
		ESP_LOGE(TAG, "Failed to setup OTA advertisment receive timeout\n");
		return ESP_FAIL;
	}
	ota.mcast_socket = sock;

	return ESP_OK;
}

static void ota_announce_serve(void) {
	int64_t now = esp_timer_get_time();
	int32_t delta_ms = (now - ota.last_tx_timestamp_us) / 1000;
	if (delta_ms >= OTA_UPDATE_SERVE_INTERVAL_MS) {
		ota_packet_t ota_packet = { 0 };
		ota_packet.packet_type = WIRELESS_PACKET_TYPE_OTA;
		ota_packet.ota_packet_type = OTA_PACKET_TYPE_INIT;
		wireless_broadcast((const uint8_t *)&ota_packet, sizeof(ota_packet));

		const char *msg = "OTA";
		struct sockaddr_in6 dst_addr = {
			.sin6_family = AF_INET6,
			.sin6_port = htons(1337),
			.sin6_flowinfo = 0,
/*
			.sin6_addr = {
				.s6_addr = {
					0xff, 0x02, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x01
				}
			},
*/
			.sin6_scope_id = (uint32_t)ota.ap_ifindex
		};
		inet_pton(AF_INET6, "ff02::1", &dst_addr.sin6_addr);
		ssize_t ret = sendto(ota.mcast_socket, msg, sizeof(msg), 0, (void *)&dst_addr, sizeof(dst_addr));
		if (ret != sizeof(msg)) {
			ESP_LOGE(TAG, "Failed to send multicast announcement: %d", errno);
		}
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
		ESP_LOGI(TAG, "Failed to connect, going back to idle state");
		ota.state = OTA_STATE_IDLE;
	} else {
		ESP_LOGI(TAG, "Connection init ok, waiting for UDP announcement...");
		ota.state = OTA_STATE_STATION_CONNECTING;
	}

	return err;
}

static esp_err_t ota_discover_server() {
	char payload[3];
	struct sockaddr_in6 src_addr;
	socklen_t src_addr_len = sizeof(src_addr);
	fd_set fds_read;
	FD_ZERO(&fds_read);
	FD_SET(ota.mcast_socket, &fds_read);
	struct timeval timeout = { 0 };
	int ready = select(ota.mcast_socket + 1, &fds_read, NULL, NULL, &timeout);
	if (ready > 0 && FD_ISSET(ota.mcast_socket, &fds_read)) {
		ssize_t ret = recvfrom(ota.mcast_socket, payload, sizeof(payload), 0, (struct sockaddr *)&src_addr, &src_addr_len);
		if (ret > 0) {
			ESP_LOGI(TAG, "Received UDP message: %.*s", (size_t)ret, payload);
		}
	}
	return ESP_OK;
}

esp_err_t ota_update() {
	switch (ota.state) {
	case OTA_STATE_SERVING:
		ota_announce_serve();
		return ESP_OK;
	case OTA_STATE_STATION_CONNECT:
		return ota_initiate_station_connection(ota.update_peer);
	case OTA_STATE_STATION_CONNECTING:
		return ota_discover_server();
	default:
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

	if (ota_packet.ota_packet_type == OTA_PACKET_TYPE_INIT) {
		if (ota.state == OTA_STATE_IDLE) {
			ESP_LOGI(TAG, "Got OTA command");
			ota.state = OTA_STATE_STATION_CONNECT;
			memcpy(ota.update_peer, packet->src_addr, ESP_NOW_ETH_ALEN);
		}
	}

	return ESP_OK;
}

esp_err_t ota_serve_update() {
	if (ota.state == OTA_STATE_IDLE) {
		ota.state = OTA_STATE_SERVING;
	}

	if (ota.state != OTA_STATE_SERVING) {
		return ESP_ERR_INVALID_STATE;
	}

	return ESP_OK;
}
