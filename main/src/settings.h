#pragma once

#include <stdbool.h>

void settings_init(void);

void settings_set_usb_enable(bool enable);
bool settings_get_usb_enable(void);

void settings_set_usb_enable_override(bool enable);
bool settings_get_usb_enable_override(void);

void settings_set_color_channel_zero_offset(unsigned int channel, unsigned int offset);
unsigned int settings_get_color_channel_zero_offset(unsigned int channel);

void settings_set_platform_name(const char *name);
char *settings_get_platform_name(void);
