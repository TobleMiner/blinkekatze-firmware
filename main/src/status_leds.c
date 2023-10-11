#include "status_leds.h"

#include <stdbool.h>
#include <stdint.h>

#include <driver/gpio.h>
#include <esp_timer.h>

#include "neighbour.h"
#include "util.h"

#define GPIO_LED1	20
#define GPIO_LED2	21

typedef struct status_led {
	unsigned int gpio;
	status_led_mode_t mode;
	bool state;
	uint32_t interval_ms;
	int64_t timestamp_last_change_us;
} status_led_t;

#define STATUS_LED(gpio_) { gpio_, STATUS_LED_MODE_OFF, 0, 0, 0 }

void status_leds_init(void) {
	gpio_reset_pin(GPIO_LED1);
	gpio_reset_pin(GPIO_LED2);
	gpio_set_direction(GPIO_LED1, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_LED2, GPIO_MODE_OUTPUT);
}

static status_led_t status_leds[] = {
	STATUS_LED(GPIO_LED1),
	STATUS_LED(GPIO_LED2)
};

void status_led_update(status_led_t *led) {
	switch (led->mode) {
	case STATUS_LED_MODE_ON:
		gpio_set_level(led->gpio, 1);
		break;
	case STATUS_LED_MODE_OFF:
		gpio_set_level(led->gpio, 0);
		break;
	case STATUS_LED_MODE_BLINK: {
		int64_t now = esp_timer_get_time();
		int64_t delta_us = now - led->timestamp_last_change_us;
		if (delta_us / 1000 >= led->interval_ms) {
			led->state = !led->state;
			gpio_set_level(led->gpio, led->state);
			led->timestamp_last_change_us = now;
		}
		break;
	}
	case STATUS_LED_MODE_STROBE: {
		if (led->state) {
			int64_t now = esp_timer_get_time();
			int64_t delta_us = now - led->timestamp_last_change_us;
			if (delta_us / 1000 >= led->interval_ms) {
				led->state = !led->state;
				gpio_set_level(led->gpio, led->state);
				led->timestamp_last_change_us = now;
			}
		}
		break;
	}
	}
}

void status_leds_update(void) {
	for (int i = 0; i < ARRAY_SIZE(status_leds); i++) {
		status_led_update(&status_leds[i]);
	}
}

void status_led_set_mode(status_led_id_t led_id, status_led_mode_t mode) {
	status_led_t *led = &status_leds[led_id];
	led->mode = mode;
}

void status_led_set_blink(status_led_id_t led_id, unsigned int blink_interval_ms) {
	status_led_t *led = &status_leds[led_id];
	led->mode = STATUS_LED_MODE_BLINK;
	led->interval_ms = blink_interval_ms;
}

void status_led_set_strobe(status_led_id_t led_id, unsigned int strobe_duration_ms) {
	status_led_t *led = &status_leds[led_id];
	led->state = false;
	gpio_set_level(led->gpio, led->state);
	led->mode = STATUS_LED_MODE_STROBE;
	led->interval_ms = strobe_duration_ms;
}

void status_led_strobe(status_led_id_t led_id) {
	status_led_t *led = &status_leds[led_id];
	if (!led->state) {
		led->state = true;
		gpio_set_level(led->gpio, led->state);
		led->timestamp_last_change_us = esp_timer_get_time();
	}
}
