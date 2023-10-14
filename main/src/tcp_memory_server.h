#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <sys/socket.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_err.h>

#define TCP_MEMORY_SERVER_MAX_CLIENTS	8

typedef struct tcp_memory_server_client {
	int socket;
	size_t offset;
} tcp_memory_server_client_t;

typedef struct tcp_memory_server {
	bool exit;
	int listen_socket;
	struct ifreq bind_iface;
	const uint8_t *memory_addr;
	size_t memory_size;
	unsigned int port;
	tcp_memory_server_client_t clients[TCP_MEMORY_SERVER_MAX_CLIENTS];
	TaskHandle_t task;
} tcp_memory_server_t;

esp_err_t tcp_memory_server_init(tcp_memory_server_t *server, const uint8_t *memory_addr, size_t memory_size, unsigned int port, const struct ifreq *bind_iface);
