#include "squish.h"

#include <string.h>

#include <esp_log.h>
#include <esp_timer.h>

#define NUM_PRESSURE_SAMPLES_DISCARD	 5
#define NUM_PRESSURE_SAMPLES_INIT	20

#define MAX_SQUISHEDNESS		25000
#define SQUISH_FACTOR			2
#define SQUISH_RECOVERY_INTERVAL_MS	3000

static const char *TAG = "squish";

void squish_init(squish_t *squish, spl06_t *baro) {
	memset(squish, 0, sizeof(*squish));
	squish->baro = baro;
}

esp_err_t squish_update(squish_t *squish) {
	esp_err_t err = spl06_update(squish->baro);
	if (err) {
		ESP_LOGE(TAG, "Failed to update barometer: %d", err);
		return err;
	}

	int64_t now = esp_timer_get_time();
	int64_t delta_us = now - squish->timestamp_last_update_us;
	if (squish->squishedness) {
		uint32_t squish_decay = (uint32_t)MAX_SQUISHEDNESS * delta_us / (uint32_t)SQUISH_RECOVERY_INTERVAL_MS / 1000UL;

		if (squish_decay > squish->squishedness) {
			squish->squishedness = 0;
		} else {
			squish->squishedness -= squish_decay;
		}
	}

	int32_t pressure = spl06_get_pressure(squish->baro);
	if (squish->num_pressure_samples < NUM_PRESSURE_SAMPLES_INIT) {
		if (squish->num_pressure_samples == NUM_PRESSURE_SAMPLES_DISCARD) {
			squish->pressure_at_rest = pressure;
		}
		if (squish->num_pressure_samples > NUM_PRESSURE_SAMPLES_DISCARD) {
			squish->pressure_at_rest =
				(squish->pressure_at_rest * 8 + pressure * 2) / 10;
		}
		squish->num_pressure_samples++;
	} else {
		int32_t delta = squish->pressure_at_rest - pressure;

		if (delta > 0) {
			uint32_t additional_squish = delta * SQUISH_FACTOR * (delta_us / 1000) / 8192;
			squish->squishedness = MIN(squish->squishedness + additional_squish, MAX_SQUISHEDNESS);
		}
	}
	squish->timestamp_last_update_us = now;
	return ESP_OK;
}

void squish_apply(const squish_t *squish, color_hsv_t *color) {
	/*
	 * Squishing is a three step process:
	 *  1. Desaturate current color to white (0 - 1/5 squish)
	 *  2. Change color from white to red and saturate (1/5 - 3/5 squish)
	 *  3. Change color from red to blue (3/5 - full squish)
	 */
//	const int32_t mid_hue = 40 << 5;
	const int32_t mid_hue = HSV_HUE_MAX;
	const int32_t mid_saturation = 220 << 8;
//	const int32_t mid_saturation = 0;
	const int32_t final_hue = 845 << 5;
	const int32_t final_saturation = 200 << 8;
//	const int32_t final_saturation = HSV_SAT_MAX;
	int32_t bend = squish->squishedness;
	const int32_t scale1 = MAX_SQUISHEDNESS / 5;
	const int32_t scale2 = MAX_SQUISHEDNESS * 2 / 5;
	const int32_t scale3 = MAX_SQUISHEDNESS * 2 / 5;
	if (bend <= scale1) {
		/* Desaturate to white */
		color->s -= (int32_t)color->s * bend  / scale1;
	} else if (bend <= scale1 + scale2) {
		/* Stay white for a while */
		color->s = 0;


		/* Force to red and saturate */
/*
		int32_t local_bend = bend - scale1;
		color->h = mid_hue;
		color->s = mid_saturation * local_bend / scale2;
*/
	} else {
		/* Force to red and saturate */
		int32_t local_bend = bend - scale1 - scale2;
		color->h = mid_hue;
		color->s = mid_saturation * local_bend / scale3;

		/* Morph to blue */
/*
		int32_t local_bend = bend - scale1 - scale2;
		color->h = mid_hue + (final_hue - mid_hue) * local_bend / scale3;
		color->s = mid_saturation + (final_saturation - mid_saturation) * local_bend / scale2;
		ESP_LOGI(TAG, "Saturation: %u, local_bend: %ld, scale2: %ld", color->s, local_bend, scale2);
*/
	}
}
