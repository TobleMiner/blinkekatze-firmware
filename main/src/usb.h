#pragma once

#include <stdbool.h>

#include "wireless.h"

void usb_init(void);
void usb_reapply_enable(void);

void usb_set_enable(bool enable);

bool usb_is_enable_overriden(void);
void usb_set_enable_override(bool enable);

void usb_config_rx(const wireless_packet_t *packet);
