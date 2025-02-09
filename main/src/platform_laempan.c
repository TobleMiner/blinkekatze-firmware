#include "platform_laempan.h"

#include <stdlib.h>

#include <driver/ledc.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_err.h>
#include <esp_log.h>

#include "color.h"
#include "embedded_files.h"
#include "util.h"

#define GPIO_RED	2
#define GPIO_GREEN	1
#define GPIO_BLUE	3
#define GPIO_WHITE	7

typedef struct brightness_white_packet {
	uint8_t packet_type;
	uint16_t brightness;
	wireless_address_t addr;
} __attribute__((packed)) brightness_white_packet_t;

static const char *TAG = "laempan";

static const rgb16_t *colorcal_table = (const rgb16_t *)EMBEDDED_FILE_PTR(colorcal_laempan_16x16x16_11bit_bin);

#define COLOR_TABLE_SIZE	16UL
#define LOOKUP_DIV		((1 << 16) / COLOR_TABLE_SIZE)

static void lookup_color(const rgb16_t *in, rgb16_t *out) {
	uint32_t idx = ((in->g * COLOR_TABLE_SIZE) + in->b) * COLOR_TABLE_SIZE + in->r;
	*out = colorcal_table[idx];
}

static void apply_color_correction_per_channel(const rgb16_t *in, rgb16_t *out) {
	rgb16_t color_min_lookup = { in->r / LOOKUP_DIV, in->g / LOOKUP_DIV, in->b / LOOKUP_DIV };
	rgb16_t color_ref_lookup = {
		MIN(in->r / LOOKUP_DIV + DIV_ROUND(in->r % LOOKUP_DIV, LOOKUP_DIV), COLOR_TABLE_SIZE - 1),
		MIN(in->g / LOOKUP_DIV + DIV_ROUND(in->g % LOOKUP_DIV, LOOKUP_DIV), COLOR_TABLE_SIZE - 1),
		MIN(in->b / LOOKUP_DIV + DIV_ROUND(in->b % LOOKUP_DIV, LOOKUP_DIV), COLOR_TABLE_SIZE - 1),
	};
	rgb16_t color_lookup_r = {
		MIN(MAX((int)color_ref_lookup.r + (color_ref_lookup.r == color_min_lookup.r ? 1 : -1), 0), COLOR_TABLE_SIZE - 1),
		color_ref_lookup.g,
		color_ref_lookup.b
	};
	rgb16_t color_lookup_g = {
		color_ref_lookup.r,
		MIN(MAX((int)color_ref_lookup.g + (color_ref_lookup.g == color_min_lookup.g ? 1 : -1), 0), COLOR_TABLE_SIZE - 1),
		color_ref_lookup.b
	};
	rgb16_t color_lookup_b = {
		color_ref_lookup.r,
		color_ref_lookup.g,
		MIN(MAX((int)color_ref_lookup.b + (color_ref_lookup.b == color_min_lookup.b ? 1 : -1), 0), COLOR_TABLE_SIZE - 1)
	};
	rgb16_t color_ref = {
		color_ref_lookup.r * LOOKUP_DIV,
		color_ref_lookup.g * LOOKUP_DIV,
		color_ref_lookup.b * LOOKUP_DIV
	};
	rgb16_t color_max = {
		color_lookup_r.r * LOOKUP_DIV,
		color_lookup_g.g * LOOKUP_DIV,
		color_lookup_b.b * LOOKUP_DIV
	};
	rgb16_t color_corrected_ref;
	rgb16_t color_corrected_r;
	rgb16_t color_corrected_g;
	rgb16_t color_corrected_b;
	lookup_color(&color_ref_lookup, &color_corrected_ref);
	lookup_color(&color_lookup_r, &color_corrected_r);
	lookup_color(&color_lookup_g, &color_corrected_g);
	lookup_color(&color_lookup_b, &color_corrected_b);

	int32_t delta_in_r = (int32_t)in->r - (int32_t)color_ref.r;
	int32_t delta_in_g = (int32_t)in->g - (int32_t)color_ref.g;
	int32_t delta_in_b = (int32_t)in->b - (int32_t)color_ref.b;
	int32_t delta_ref_r = (int32_t)color_max.r - (int32_t)color_ref.r;
	int32_t delta_ref_g = (int32_t)color_max.g - (int32_t)color_ref.g;
	int32_t delta_ref_b = (int32_t)color_max.b - (int32_t)color_ref.b;
	rgb16_t color_out = color_corrected_ref;

	if (delta_ref_r) {
		color_out.r += DIV_ROUND(((int32_t)color_corrected_r.r - (int32_t)color_corrected_ref.r) * delta_in_r, delta_ref_r);
		color_out.g += DIV_ROUND(((int32_t)color_corrected_r.g - (int32_t)color_corrected_ref.g) * delta_in_r, delta_ref_r);
		color_out.b += DIV_ROUND(((int32_t)color_corrected_r.b - (int32_t)color_corrected_ref.b) * delta_in_r, delta_ref_r);
	}
	if (delta_ref_g) {
		color_out.r += DIV_ROUND(((int32_t)color_corrected_g.r - (int32_t)color_corrected_ref.r) * delta_in_g, delta_ref_g);
		color_out.g += DIV_ROUND(((int32_t)color_corrected_g.g - (int32_t)color_corrected_ref.g) * delta_in_g, delta_ref_g);
		color_out.b += DIV_ROUND(((int32_t)color_corrected_g.b - (int32_t)color_corrected_ref.b) * delta_in_g, delta_ref_g);
	}
	if (delta_ref_b) {
		color_out.r += DIV_ROUND(((int32_t)color_corrected_b.r - (int32_t)color_corrected_ref.r) * delta_in_b, delta_ref_b);
		color_out.g += DIV_ROUND(((int32_t)color_corrected_b.g - (int32_t)color_corrected_ref.g) * delta_in_b, delta_ref_b);
		color_out.b += DIV_ROUND(((int32_t)color_corrected_b.b - (int32_t)color_corrected_ref.b) * delta_in_b, delta_ref_b);
	}

	*out = color_out;
}

#define CLAMP_ADD(a, b, maxval) \
	((maxval) - (a) >= (b) ? ((a) + (b)) : (maxval))

static void set_rgb_led_color(platform_t *platform, uint16_t r, uint16_t g, uint16_t b) {
	platform_laempan_t *laempan = container_of(platform, platform_laempan_t, base);
	const rgb16_t raw = { r, g, b };
	rgb16_t corrected;


	apply_color_correction_per_channel(&raw, &corrected);

	if (corrected.r) {
		corrected.r = CLAMP_ADD(corrected.r, laempan->color_channel_offsets[0], 65535U >> 5);
	}
	if (corrected.g) {
		corrected.g = CLAMP_ADD(corrected.g, laempan->color_channel_offsets[1], 65535U >> 5);
	}
	if (corrected.b) {
		corrected.b = CLAMP_ADD(corrected.b, laempan->color_channel_offsets[2], 65535U >> 5);
	}

	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, corrected.r);
	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, corrected.g);
	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, corrected.b);
	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, laempan->white_brightness);

	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

static esp_err_t set_color_channel_offset(platform_t *platform, unsigned int channel, unsigned int offset) {
	platform_laempan_t *laempan = container_of(platform, platform_laempan_t, base);

	return color_channel_set_offset(&laempan->cc_offset, channel, offset);
}

static void handle_brightness_white_packet(platform_laempan_t *laempan, const wireless_packet_t *packet) {
	if (packet->len < sizeof(brightness_white_packet_t)) {
		ESP_LOGE(TAG, "Short packet received. Expected %zu bytes but got %u bytes", sizeof(brightness_white_packet_t), packet->len);
		return;
	}

	brightness_white_packet_t bright_packet;
	memcpy(&bright_packet, packet->data, sizeof(bright_packet));
	if (wireless_is_broadcast_address(bright_packet.addr) ||
	    wireless_is_local_address(bright_packet.addr)) {
		laempan->white_brightness = bright_packet.brightness;
	}
}

static bool handle_packet(platform_t *platform, uint8_t packet_type, const wireless_packet_t *packet) {
	platform_laempan_t *laempan = container_of(platform, platform_laempan_t, base);

	if (packet_type == WIRELESS_PACKET_TYPE_COLOR_CHANNEL_ZERO_OFFSET) {
		color_channel_offset_rx(&laempan->cc_offset, packet);
		return true;
	}

	if (packet_type == WIRELESS_PACKET_TYPE_BRIGHTNESS_WHITE) {
		handle_brightness_white_packet(laempan, packet);
		return true;
	}

	return false;
}

static void set_brightness_white(platform_t *platform, uint16_t bright) {
	platform_laempan_t *laempan = container_of(platform, platform_laempan_t, base);

	laempan->white_brightness = bright;
}

static const platform_ops_t laempan_ops = {
	.set_rgb_led_color = set_rgb_led_color,
	.set_color_channel_offset = set_color_channel_offset,
	.handle_packet = handle_packet,
	.set_brightness_white = set_brightness_white
};

void configure_ledc_channel(ledc_channel_t chan, int gpio) {
	ledc_channel_config_t cfg = {
		.gpio_num = gpio,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = chan,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = LEDC_TIMER_0,
		.duty = 0,
		.hpoint = 0,
		.flags = {
			.output_invert = 0
		}
	};
	ESP_ERROR_CHECK(ledc_channel_config(&cfg));
}

esp_err_t platform_laempan_probe(platform_t **ret) {
	adc_oneshot_unit_handle_t adc1_handle;
	adc_oneshot_unit_init_cfg_t init_config1 = {
		.unit_id = ADC_UNIT_1,
	};

	esp_err_t err = adc_oneshot_new_unit(&init_config1, &adc1_handle);
	if (err) {
		return err;
	}

	adc_oneshot_chan_cfg_t config = {
		.bitwidth = ADC_BITWIDTH_DEFAULT,
		.atten = ADC_ATTEN_DB_11,
	};
	err = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config);
	if (err) {
		return err;
	}

	err = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config);
	if (err) {
		return err;
	}

	adc_cali_handle_t handle = NULL;
	adc_cali_curve_fitting_config_t cali_config = {
		.unit_id = ADC_UNIT_1,
		.atten = ADC_ATTEN_DB_11,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
	err = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
	if (err) {
		return err;
	}

	int raw_ntc;
	int raw_vbus;
	err = adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &raw_ntc);
	if (err) {
		return err;
	}

	err = adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &raw_vbus);
	if (err) {
		return err;
	}

	int ntc_voltage_mv;
	int vbus_voltage_mv;
	err = adc_cali_raw_to_voltage(handle, raw_ntc, &ntc_voltage_mv);
	if (err) {
		return err;
	}
	err = adc_cali_raw_to_voltage(handle, raw_vbus, &vbus_voltage_mv);
	if (err) {
		return err;
	}
	// VBUS is divided by 11 for ADC
	vbus_voltage_mv *= 11;

	ESP_LOGI(TAG, "NTC voltage: %d", ntc_voltage_mv);
	ESP_LOGI(TAG, "VBUS voltage: %d", vbus_voltage_mv);

	/*
	 * NTC voltage divider is broken in first revision.
	 * Instead of a 47k pull-up a 4.7k pull-up is populated, turning this into
	 * a signal pulled high strongly.
	 */
	if (ntc_voltage_mv < 1650) {
		return -1;
	}

	if (vbus_voltage_mv < 4000 || vbus_voltage_mv > 16000) {
		return -1;
	}

	platform_laempan_t *laempan = calloc(1, sizeof(platform_laempan_t));
	if (!laempan) {
		return ESP_ERR_NO_MEM;
	}

	color_channel_offset_init(&laempan->cc_offset, laempan->color_channel_offsets, ARRAY_SIZE(laempan->color_channel_offsets));

	const ledc_timer_config_t timer_cfg = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = LEDC_TIMER_11_BIT,
		.timer_num = LEDC_TIMER_0,
		.freq_hz = 19500,
		.clk_cfg = LEDC_AUTO_CLK
	};
	ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
	configure_ledc_channel(LEDC_CHANNEL_0, GPIO_RED);
	configure_ledc_channel(LEDC_CHANNEL_1, GPIO_GREEN);
	configure_ledc_channel(LEDC_CHANNEL_2, GPIO_BLUE);
	configure_ledc_channel(LEDC_CHANNEL_3, GPIO_WHITE);

	platform_init(&laempan->base, &laempan_ops, "laempan");
	laempan->base.default_brightness = HSV_VAL_MAX / 3;
	laempan->white_brightness = 400;
	*ret = &laempan->base;

	return 0;
}

bool platform_is_laempan(const platform_t *platform) {
	return platform_is(platform, "laempan");
}

void platform_brightness_white_tx(platform_t *platform, uint16_t brightness, const uint8_t *address) {
	brightness_white_packet_t bright_packet = {
		WIRELESS_PACKET_TYPE_BRIGHTNESS_WHITE,
		brightness,
		{ }
	};
	memcpy(bright_packet.addr, address, sizeof(wireless_address_t));

	if (wireless_is_broadcast_address(bright_packet.addr) ||
	    wireless_is_local_address(bright_packet.addr)) {
		platform_set_brightness_white(platform, brightness);
	}
	wireless_broadcast((const uint8_t *)&bright_packet, sizeof(bright_packet));
}
