#include "usb.h"

#include <hal/gpio_ll.h>

#include "settings.h"

#define FLAG_USB_ENABLE_OVERRIDE BIT(0)

typedef struct usb_config_packet {
	uint8_t packet_type;
	uint8_t target_address[ETH_ALEN];
	uint8_t flags;
} __attribute__((packed)) usb_config_packet_t;

static const char *TAG = "usb";

static void usb_enable_update(void) {
	if (settings_get_usb_enable() || usb_is_enable_overridden()) {
		SET_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
	} else {
		CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
	}
}

void usb_init() {
	usb_enable_update();
}

void usb_set_enable(bool enable) {
	settings_set_usb_enable(enable);
	usb_enable_update();
}

bool usb_is_enable_overriden(void) {
	return settings_get_usb_enable_override();
}

void usb_set_enable_override(bool enable) {
	settings_set_usb_enable_override(enable);
	usb_enable_update();
}

void usb_config_rx(const wireless_packet_t *packet) {
	usb_config_packet_t config_packet;
	if (packet->len < sizeof(config_packet)) {
		ESP_LOGD(TAG, "Received short packet, expected %u bytes but got only %u bytes",
		         sizeof(config_packet), packet->len);
		return;
	}
	memcpy(&config_packet, packet->data, sizeof(config_packet));
	if (wireless_is_local_address(config_packet.target_address)) {
		usb_set_enable_override(!!(packet.flags & FLAG_USB_ENABLE_OVERRIDE));
	}
}

void usb_config_tx_override_enable(const uint8_t *address, bool enable) {
	usb_config_packet_t config_packet = {
		.packet_type = WIRELESS_PACKET_TYPE_USB_CONFIG,
		.flags = enable ? FLAG_USB_ENABLE_OVERRIDE : 0
	};
	memcpy(config_packet.target_address, address, sizeof(config_packet.target_address));
	wireless_broadcast((const uint8_t *)&config_packet, sizeof(config_packet));
}
