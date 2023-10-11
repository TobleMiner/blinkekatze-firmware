#pragma once

typedef enum status_led_id {
	STATUS_LED_RED = 0,
	STATUS_LED_GREEN = 1
} status_led_id_t;

typedef enum status_led_mode {
	STATUS_LED_MODE_OFF,
	STATUS_LED_MODE_ON,
	STATUS_LED_MODE_BLINK,
	STATUS_LED_MODE_STROBE
} status_led_mode_t;

void status_leds_init(void);
void status_leds_update(void);
void status_led_set_mode(status_led_id_t led_id, status_led_mode_t mode);
void status_led_set_blink(status_led_id_t led_id, unsigned int blink_interval_ms);
void status_led_set_strobe(status_led_id_t led_id, unsigned int strobe_duration_ms);
void status_led_strobe(status_led_id_t led_id);
