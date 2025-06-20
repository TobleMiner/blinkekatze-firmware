#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <esp_err.h>

#include "color.h"
#include "wireless.h"

esp_err_t reboot_rx(const wireless_packet_t *packet);
void reboot_tx(const uint8_t *address);
