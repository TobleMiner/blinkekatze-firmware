#pragma once

#include <stdbool.h>

void settings_init(void);

void settings_set_usb_enable(bool enable);
bool settings_get_usb_enable(void);

void settings_set_usb_enable_override(bool enable);
bool settings_get_usb_enable_override(void);
