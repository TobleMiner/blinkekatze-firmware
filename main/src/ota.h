#pragma once

#include <stdbool.h>
#include <sys/types.h>

#include <esp_err.h>

#include "color.h"
#include "neighbour.h"
#include "wireless.h"

esp_err_t ota_init(void);
esp_err_t ota_rx(const wireless_packet_t *packet, const neighbour_t *neigh);
esp_err_t ota_serve_update(bool serve);
void ota_indicate_update(color_hsv_t *color);
void ota_print_status(void);
void ota_set_ignore_version(bool ignore_version);
ssize_t ota_neighbour_info_to_string(const neighbour_ota_info_t *info, char *dst, size_t len);
