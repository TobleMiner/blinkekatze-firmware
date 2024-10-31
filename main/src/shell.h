#pragma once

#include <esp_err.h>

#include "bonk.h"
#include "platform.h"

esp_err_t shell_init(bonk_t *bonk, platform_t *platform);
