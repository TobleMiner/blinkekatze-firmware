#pragma once

#include <netinet/in.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/socket.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#define TCP_CLIENT_BUFFER_SIZE 512

typedef esp_err_t (*tcp_client_init_cb_f)(void *priv);
typedef esp_err_t (*tcp_client_data_cb_f)(void *priv, const void *data, size_t len);
typedef esp_err_t (*tcp_client_finish_cb_f)(void *priv);

typedef struct tcp_client {
	void *cb_priv;
	tcp_client_init_cb_f init_cb;
	tcp_client_data_cb_f data_cb;
	tcp_client_finish_cb_f finish_cb;
	struct sockaddr_in remote_addr;
	int err;
	esp_err_t cb_err;
	struct ifreq bind_iface;
	TaskHandle_t task;
	StaticEventGroup_t task_events_buffer;
	EventGroupHandle_t task_events;
	uint8_t data_buf[TCP_CLIENT_BUFFER_SIZE];
	bool do_exit;
} tcp_client_t;

esp_err_t tcp_client_init(tcp_client_t *client, struct in_addr *address, unsigned short port, const struct ifreq *bind_iface, tcp_client_init_cb_f init_cb, tcp_client_data_cb_f data_cb, tcp_client_finish_cb_f finish_cb, void *cb_priv);
void tcp_client_cancel(tcp_client_t *client);
bool tcp_client_is_done(tcp_client_t *client);
int tcp_client_get_err(tcp_client_t *client);
esp_err_t tcp_client_get_cb_err(tcp_client_t *client);
