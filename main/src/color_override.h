#pragma once

#include <stdbool.h>

#include "color.h"

void color_override_set_enable(bool enable);
void color_override_set_color(const rgb16_t *rgb);
void color_override_apply(rgb16_t *rgb);
