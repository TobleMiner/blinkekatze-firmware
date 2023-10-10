#include "status_leds.h"

#include <stdbool.h>
#include <stdint.h>

#include <driver/gpio.h>
#include <esp_timer.h>

#include "neighbour.h"
#include "util.h"

#define GPIO_LED1	20
#define GPIO_LED2	21

typedef enum status_led_mode {
	STATUS_LED_OFF,
	STATUS_LED_ON,
	STATUS_LED_BLINK
} status_led_mode_t;

typedef struct status_led {
	unsigned int gpio;
	status_led_mode_t mode;
	bool state;
	uint32_t blink_interval_ms;
	int64_t timestamp_last_change_us;
} status_led_t;

#define STATUS_LED(gpio_, initial_mode_) { gpio_, initial_mode_, 0, 0, 0 }

void status_leds_init(void) {
	gpio_reset_pin(GPIO_LED1);
	gpio_reset_pin(GPIO_LED2);
	gpio_set_direction(GPIO_LED1, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_LED2, GPIO_MODE_OUTPUT);
}

static status_led_t status_leds[] = {
	STATUS_LED(GPIO_LED1, STATUS_LED_OFF),
	STATUS_LED(GPIO_LED2, STATUS_LED_OFF)
};

static status_led_t *red_led = &status_leds[0];
static status_led_t *green_led = &status_leds[1];

void status_led_update(status_led_t *led) {
	switch (led->mode) {
	case STATUS_LED_ON:
		gpio_set_level(led->gpio, 1);
		break;
	case STATUS_LED_OFF:
		gpio_set_level(led->gpio, 0);
		break;
	case STATUS_LED_BLINK: {
		int64_t now = esp_timer_get_time();
		int64_t delta_us = now - led->timestamp_last_change_us;
		if (delta_us / 1000 >= led->blink_interval_ms) {
			led->state = !led->state;
			gpio_set_level(led->gpio, led->state);
			led->timestamp_last_change_us = now;
		}
		break;
	}
	}
}

void status_leds_update(void) {
	if (neighbour_has_neighbours()) {
		green_led->mode = STATUS_LED_ON;
	} else {
		green_led->mode = STATUS_LED_BLINK;
		green_led->blink_interval_ms = 1000;
	}

	for (int i = 0; i < ARRAY_SIZE(status_leds); i++) {
		status_led_update(&status_leds[i]);
	}
}
