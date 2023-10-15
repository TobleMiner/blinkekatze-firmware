#include "tcp_memory_server.h"

#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <sys/select.h>
#include <sys/socket.h>

#include <esp_log.h>
#include <esp_timer.h>

#include "futil.h"
#include "util.h"

#define CHUNK_SIZE 		1024
#define MAX_PENDING_CONNECTIONS	8
#define TASK_STACK_SIZE		4096
#define CLIENT_DATA_TIMEOUT_MS  10000

static const char *TAG = "tcp_memory_server";

static int find_empty_client_slot(tcp_memory_server_t *server) {
	for (int i = 0; i < ARRAY_SIZE(server->clients); i++) {
		tcp_memory_server_client_t *client = &server->clients[i];
		if (client->socket == -1) {
			return i;
		}
	}

	return -1;
}

static int prepare_fd_sets(tcp_memory_server_t *server, fd_set *fd_read, fd_set *fd_write, fd_set *fd_err) {
	int fd = server->listen_socket;

	FD_ZERO(fd_read);
	FD_ZERO(fd_write);
	FD_ZERO(fd_err);
	FD_SET(server->listen_socket, fd_read);
	for (int i = 0; i < ARRAY_SIZE(server->clients); i++) {
		tcp_memory_server_client_t *client = &server->clients[i];

		if (client->socket >= 0) {
			fd = MAX(fd, client->socket);
			FD_SET(client->socket, fd_write);
			FD_SET(client->socket, fd_err);
		}
	}

	return fd;
}

static void client_close_connection(tcp_memory_server_client_t *client) {
	shutdown(client->socket, SHUT_RDWR);
	close(client->socket);
	client->socket = -1;
}

static void tcp_memory_server_main_loop(void *arg) {
	tcp_memory_server_t *server = arg;
	while (!server->exit) {
		fd_set fd_read;
		fd_set fd_write;
		fd_set fd_err;
		int max_fd = prepare_fd_sets(server, &fd_read, &fd_write, &fd_err);
		struct timeval timeout = { .tv_sec = 1, .tv_usec = 0 };
		int ret = select(max_fd + 1, &fd_read, &fd_write, &fd_err, &timeout);
		if (ret >= 0) {
			int64_t now = esp_timer_get_time();
			if (FD_ISSET(server->listen_socket, &fd_read)) {
				do {
					ret = accept(server->listen_socket, NULL, NULL);
					if (ret >= 0) {
						int sock = ret;
						int client_slot = find_empty_client_slot(server);
						if (client_slot >= 0) {
							tcp_memory_server_client_t *client = &server->clients[client_slot];
							client->socket = sock;
							client->offset = 0;
							client->last_tx_timestamp_us = now;
						} else {
							shutdown(sock, SHUT_RDWR);
							close(sock);
						}
					}
				} while (ret >= 0);
			}
			for (int i = 0; i < ARRAY_SIZE(server->clients); i++) {
				tcp_memory_server_client_t *client = &server->clients[i];
				if (client->socket >= 0) {
					if (FD_ISSET(client->socket, &fd_err)) {
						ESP_LOGE(TAG, "Socket of client %d failed", i);
						client_close_connection(client);
					} else if (FD_ISSET(client->socket, &fd_write)) {
						size_t data_len = server->memory_size - client->offset;
						if (data_len > CHUNK_SIZE) {
							data_len = CHUNK_SIZE;
						}
						ssize_t write_len = write(client->socket, server->memory_addr + client->offset, data_len);
						if (write_len >= 0) {
							client->offset += write_len;
							client->last_tx_timestamp_us = now;
						} else if (errno != EAGAIN && errno != EWOULDBLOCK) {
							ESP_LOGE(TAG, "Failed to write to client %d: %d", i, errno);
							client_close_connection(client);
						}
					} else {
						int32_t delta_ms = (now - client->last_tx_timestamp_us) / 1000LL;
						if (delta_ms >= CLIENT_DATA_TIMEOUT_MS) {
							ESP_LOGE(TAG, "Data timeout on client %d: %d", i, errno);
							client_close_connection(client);
						}
					}
				}
			}
		} else if (ret < 0) {
			ESP_LOGE(TAG, "select failed: %d", errno);
		}
	}

	vTaskDelete(NULL);
}

esp_err_t tcp_memory_server_init(tcp_memory_server_t *server, const uint8_t *memory_addr, size_t memory_size, unsigned int port, const struct ifreq *bind_iface) {
	memset(server, 0, sizeof(*server));

	esp_err_t retval = 0;
	server->memory_addr = memory_addr;
	server->memory_size = memory_size;
	server->port = port;
	server->bind_iface = *bind_iface;
	server->exit = false;

	for (int i = 0; i < ARRAY_SIZE(server->clients); i++) {
		tcp_memory_server_client_t *client = &server->clients[i];
		client->socket = -1;
		client->offset = 0;
	}

	int listen_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (listen_socket < 0) {
		ESP_LOGE(TAG, "Failed to setup listening socket: %d", errno);
		return ESP_FAIL;
	}

	const int one = 1;
	int ret = setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
	if (ret < 0) {
		ESP_LOGE(TAG, "Failed to enable SO_REUSEADDR: %d", errno);
		retval = ESP_FAIL;
		goto out_socket;
	}

	struct sockaddr_in listen_address = {
		.sin_family = AF_INET,
		.sin_port = htons(port),
		.sin_addr = {
			.s_addr = 0
		}
	};

	ret = bind(listen_socket, (struct sockaddr *)&listen_address, sizeof(listen_address));
	if (ret < 0) {
		ESP_LOGE(TAG, "Failed to bind listen socket: %d", errno);
		retval = ESP_FAIL;
		goto out_socket;
	}

	ret = setsockopt(listen_socket, SOL_SOCKET, SO_BINDTODEVICE, &server->bind_iface, sizeof(server->bind_iface));
	if (ret < 0) {
		ESP_LOGE(TAG, "Failed to bind server socket to device '%.*s': %d",
			 sizeof(server->bind_iface.ifr_name), server->bind_iface.ifr_name, errno);
		retval = ESP_FAIL;
		goto out_socket;
	}

	ret = listen(listen_socket, MAX_PENDING_CONNECTIONS);
	if (ret < 0) {
		ESP_LOGE(TAG, "Failed to listen: %d", errno);
		retval = ESP_FAIL;
		goto out_socket;
	}

	ret = futil_set_fd_blocking(listen_socket, false);
	if (ret) {
		ESP_LOGE(TAG, "Failed to set listen socket non-blocking: %d", ret);
		retval = ESP_FAIL;
		goto out_socket;
	}

	server->listen_socket = listen_socket;

	if (xTaskCreate(tcp_memory_server_main_loop, "tcp_memory_server", TASK_STACK_SIZE, server, 0, &server->task) != pdPASS) {
		ESP_LOGE(TAG, "Failed to create server task");
		retval = ESP_FAIL;
		goto out_socket;
	}

	return ESP_OK;

out_socket:
	close(listen_socket);
	return retval;
}
