#include "tcp_client.h"

#include <errno.h>
#include <sys/select.h>
#include <sys/socket.h>

#include <esp_log.h>

#include "futil.h"
#include "util.h"

#define TASK_STACK_SIZE	4096

#define EVENT_TASK_EXIT BIT(0)

static const char *TAG = "tcp_client";

static void tcp_client_main_loop(void *arg) {
	tcp_client_t *client = arg;

	if (client->init_cb) {
		esp_err_t err = client->init_cb(client->cb_priv);
		if (err) {
			ESP_LOGE(TAG, "Init cb failed: %d", err);
			client->cb_err = err;
			goto exit;
		}
	}

	int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (sock <= 0) {
		ESP_LOGE(TAG, "Failed to create client socket: %d", errno);
		client->err = errno;
		goto exit;
	}

	int err = setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, &client->bind_iface, sizeof(client->bind_iface));
	if (err < 0) {
		ESP_LOGE(TAG, "Failed to bind client socket to device '%.*s': %d",
			 sizeof(client->bind_iface.ifr_name), client->bind_iface.ifr_name, errno);
		client->err = errno;
		goto out_socket;
	}

	err = connect(sock, (struct sockaddr *)&client->remote_addr, sizeof(client->remote_addr));
	if (err < 0) {
		ESP_LOGE(TAG, "Failed to connect to server: %d", errno);
		client->err = errno;
		goto out_socket;
	}

	err = futil_set_fd_blocking(sock, false);
	if (err) {
		ESP_LOGE(TAG, "Failed to set client socket non-blocking: %d", err);
		client->err = err;
		goto out_socket;
	}

	ssize_t read_len;
	while (!client->do_exit) {
		fd_set fds_read;
		fd_set fds_err;
		FD_ZERO(&fds_read);
		FD_ZERO(&fds_err);
		FD_SET(sock, &fds_read);
		FD_SET(sock, &fds_err);
		struct timeval timeout = { .tv_sec = 1, .tv_usec = 0 };
		int ret = select(sock + 1, &fds_read, NULL, &fds_err, &timeout);
		if (ret > 0) {
			if (FD_ISSET(sock, &fds_read)) {
				read_len = read(sock, client->data_buf, sizeof(client->data_buf));
				if (read_len > 0) {
					esp_err_t cb_err = client->data_cb(client->cb_priv, client->data_buf, (size_t)read_len);
					if (cb_err) {
						ESP_LOGE(TAG, "Callback error, bailing out");
						client->cb_err = cb_err;
						break;
					}
				} else if (read_len < 0) {
					ESP_LOGE(TAG, "Read failed: %d", errno);
					client->err = errno;
					break;
				}
			}
			if (FD_ISSET(sock, &fds_err)) {
				ESP_LOGE(TAG, "Client fd has errors, bailing out");
				client->err = -1;
				break;
			}
		}
	}

out_socket:
	shutdown(sock, SHUT_RDWR);
	close(sock);
exit:
	if (client->finish_cb) {
		esp_err_t err = client->finish_cb(client->cb_priv);
		if (err) {
			ESP_LOGE(TAG, "Finish cb failed: %d", err);
			client->cb_err = err;
		}
	}
	xEventGroupSetBits(client->task_events, EVENT_TASK_EXIT);
	vTaskDelete(NULL);
}

esp_err_t tcp_client_init(tcp_client_t *client, struct in_addr *address, unsigned short port, const struct ifreq *bind_iface, tcp_client_init_cb_f init_cb, tcp_client_data_cb_f data_cb, tcp_client_init_cb_f finish_cb, void *cb_priv) {
	memset(client, 0, sizeof(*client));
	client->init_cb = init_cb;
	client->data_cb = data_cb;
	client->finish_cb = finish_cb;
	client->cb_priv = cb_priv;
	client->bind_iface = *bind_iface;
	client->remote_addr.sin_family = AF_INET;
	client->remote_addr.sin_port = htons(port);
	client->remote_addr.sin_addr = *address;
	client->task_events = xEventGroupCreateStatic(&client->task_events_buffer);
	xEventGroupClearBits(client->task_events, EVENT_TASK_EXIT);

	if (xTaskCreate(tcp_client_main_loop, "tcp_client", TASK_STACK_SIZE, client, 0, &client->task) != pdPASS) {
		ESP_LOGE(TAG, "Failed to create client task");
		return ESP_FAIL;
	}

	return ESP_OK;
}

void tcp_client_cancel(tcp_client_t *client) {
	client->do_exit = true;
}

bool tcp_client_is_done(tcp_client_t *client) {
	return !!(xEventGroupGetBits(client->task_events) & EVENT_TASK_EXIT);
}

int tcp_client_get_err(tcp_client_t *client) {
	return client->err;
}

esp_err_t tcp_client_get_cb_err(tcp_client_t *client) {
	return client->cb_err;
}
