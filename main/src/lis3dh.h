#pragma once

#include <stdint.h>

#include "i2c_bus.h"

typedef struct lis3dh {
	i2c_bus_t *bus;
	uint8_t address;
} lis3dh_t;