#include "color_override.h"

#include <esp_log.h>

#include "neighbour.h"
#include "util.h"

#define COLOR_OVERRIDE_MAX_DURATION_MS	10000
#define COLOR_OVERRIDE_MAX_FUTURE_MS	10000

typedef struct color_override_entry {
	int64_t timestamp_start_global_us;
	int64_t timestamp_stop_global_us;
	rgb16_t color;
} color_override_entry_t;

typedef struct color_override_packet {
	uint8_t packet_type;
	wireless_address_t addr;
	color_override_entry_t entry;
} __attribute__((packed)) color_override_packet_t;

typedef struct color_override {
	bool enabled;
	rgb16_t color;
	color_override_entry_t remote_overrides[100];
	unsigned int remote_override_write_idx;
} color_override_t;

static const char *TAG = "color_override";

static color_override_t color_override = { 0 };

void color_override_set_enable(bool enable) {
	color_override.enabled = enable;
}

void color_override_set_color(const rgb16_t *rgb) {
	color_override.color = *rgb;
}

const color_override_entry_t *find_active_remote_override(void) {
	int64_t now_global = neighbour_get_global_clock();
	int64_t start_max = 0;
	color_override_entry_t *most_recent_entry = NULL;
	for (unsigned int i = 0; i < ARRAY_SIZE(color_override.remote_overrides); i++) {
		color_override_entry_t *entry = &color_override.remote_overrides[i];
		if (now_global >= entry->timestamp_start_global_us &&
		    now_global <= entry->timestamp_stop_global_us &&
		    entry->timestamp_start_global_us > start_max) {
			start_max = entry->timestamp_start_global_us;
			most_recent_entry = entry;
		}
	}

	return most_recent_entry;
}

void color_override_apply(rgb16_t *rgb) {
	if (color_override.enabled) {
		*rgb = color_override.color;
	} else {
		const color_override_entry_t *entry = find_active_remote_override();
		if (entry) {
			*rgb = entry->color;
		}
	}
}

static void add_override(const color_override_packet_t *override_packet) {
	int64_t now_global = neighbour_get_global_clock();
	int64_t us_until_start = override_packet->entry.timestamp_start_global_us - now_global;
	int64_t duration_us = override_packet->entry.timestamp_stop_global_us -
			      override_packet->entry.timestamp_start_global_us;
	if (us_until_start / 1000LL <= COLOR_OVERRIDE_MAX_FUTURE_MS &&
	    duration_us > 0 &&
	    duration_us / 1000LL <= COLOR_OVERRIDE_MAX_DURATION_MS) {
		unsigned int override_idx = color_override.remote_override_write_idx;
		color_override.remote_overrides[override_idx++] = override_packet->entry;
		color_override.remote_override_write_idx = override_idx % ARRAY_SIZE(color_override.remote_overrides);
	}
}

void color_override_rx(wireless_packet_t *packet) {
	color_override_packet_t override_packet;
	if (packet->len < sizeof(override_packet)) {
		ESP_LOGD(TAG, "Received short packet! Expecting %u bytes but got only %u bytes",
			 sizeof(override_packet), packet->len);
		return;
	}
	memcpy(&override_packet, packet->data, sizeof(override_packet));
	if (wireless_is_broadcast_address(override_packet.addr) ||
	    wireless_is_local_address(override_packet.addr)) {
		add_override(&override_packet);
	}
}

void color_override_tx(const rgb16_t *color, int64_t time_start_global_us, int64_t time_stop_global_us, const uint8_t *address) {
	color_override_packet_t override_packet = {
		WIRELESS_PACKET_TYPE_COLOR_OVERRIDE,
		.entry = {
			time_start_global_us,
			time_stop_global_us,
			*color
		}
	};
	memcpy(override_packet.addr, address, sizeof(wireless_address_t));

	if (wireless_is_broadcast_address(override_packet.addr) ||
	    wireless_is_local_address(override_packet.addr)) {
		add_override(&override_packet);
	}
	wireless_broadcast((const uint8_t *)&override_packet, sizeof(override_packet));
}

void color_override_broadcast(const rgb16_t *color, int64_t time_start_global_us, int64_t time_stop_global_us) {
	color_override_tx(color, time_start_global_us, time_stop_global_us, wireless_get_broadcast_address());
}
