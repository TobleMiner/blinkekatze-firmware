#include "usb.h"

#include <esp_err.h>
#include <esp_log.h>
#include <hal/gpio_ll.h>

#include "settings.h"
#include "scheduler.h"
#include "shared_config.h"

#define FLAG_USB_ENABLE BIT(0)

typedef struct usb_config_packet {
	uint8_t packet_type;
	uint8_t flags;
	shared_config_hdr_t shared_cfg_hdr;
} __attribute__((packed)) usb_config_packet_t;

static const char *TAG = "usb";

static bool usb_enable;
static bool usb_enable_override;
scheduler_task_t usb_update_task;
shared_config_t usb_shared_cfg;

static void usb_enable_update(void) {
	ESP_LOGI(TAG, "USB enable: %d, USB enable override: %d", usb_enable, usb_enable_override);
	if (usb_enable || usb_enable_override) {
		SET_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
	} else {
		CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
	}
}

static void usb_config_tx(void) {
	usb_config_packet_t packet = {
		.packet_type = WIRELESS_PACKET_TYPE_USB_CONFIG,
		.flags = usb_enable ? FLAG_USB_ENABLE : 0
	};
	shared_config_hdr_init(&usb_shared_cfg, &packet.shared_cfg_hdr);
	wireless_broadcast((const uint8_t *)&packet, sizeof(packet));
	shared_config_tx_done(&usb_shared_cfg);
}

static void config_changed(void) {
	shared_config_update_local(&usb_shared_cfg);

	for (int i = 0; i < SHARED_CONFIG_TX_TIMES; i++) {
		usb_config_tx();
	}
}

static void usb_update(void *arg);
static void usb_update(void *arg) {
	if (shared_config_should_tx(&usb_shared_cfg)) {
		usb_config_tx();
	}
	scheduler_schedule_task_relative(&usb_update_task, usb_update, NULL, MS_TO_US(10000));
}

void usb_init() {
	usb_enable = settings_get_usb_enable();
	usb_enable_override = settings_get_usb_enable_override();
	usb_enable_update();

	scheduler_task_init(&usb_update_task);
	scheduler_schedule_task_relative(&usb_update_task, usb_update, NULL, MS_TO_US(100));
}

void usb_reapply_enable(void) {
	usb_enable_update();
}


static void usb_set_enable_(bool enable) {
	usb_enable = enable;
	settings_set_usb_enable(enable);
	usb_enable_update();
}

void usb_set_enable(bool enable) {
	bool old_enable = settings_get_usb_enable();
	if (enable != old_enable) {
		usb_set_enable_(enable);
		config_changed();
	}
}

bool usb_is_enable_overriden(void) {
	return settings_get_usb_enable_override();
}

void usb_set_enable_override(bool enable) {
	if (enable != usb_enable_override) {
		usb_enable_override = enable;
		settings_set_usb_enable_override(enable);
		usb_enable_update();
	}
}

void usb_config_rx(const wireless_packet_t *packet) {
	usb_config_packet_t config_packet;
	if (packet->len < sizeof(config_packet)) {
		ESP_LOGD(TAG, "Received short packet, expected %u bytes but got only %u bytes",
		         sizeof(config_packet), packet->len);
		return;
	}
	memcpy(&config_packet, packet->data, sizeof(config_packet));
	if (shared_config_update_remote(&usb_shared_cfg, &config_packet.shared_cfg_hdr)) {
		usb_set_enable_(!!(config_packet.flags & FLAG_USB_ENABLE));
	}
}
