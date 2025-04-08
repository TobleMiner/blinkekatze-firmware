#include "gpio.h"

#include <stdbool.h>

#include <driver/gpio.h>

static bool gpio_initialized = false;

void gpio_init() {
	if (!gpio_initialized) {
		gpio_install_isr_service(0);
		gpio_initialized = true;
	}
}
