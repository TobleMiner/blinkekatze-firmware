#include "debounce.h"

static void debounce_reset(debounce_t *debounce) {
	debounce->count = 0;
}

static void debounce_init(debounce_t *debounce, unsigned int threshold) {
	debounce->threshold = threshold;
	debounce_reset(debounce);
}

static bool debounce_is_debounced(const debounce_t *debounce) {
	return debounce->count >= debounce->threshold;
}

static void debounce_inc(debounce_t *debounce) {
	if (!debounce_is_debounced(debounce)) {
		debounce->count++;
	}
}

void debounce_bool_init(debounce_bool_t *debounce, unsigned int threshold) {
	debounce_init(&debounce->core, threshold);
}

bool debounce_bool_is_debounced(debounce_bool_t *debounce) {
	return debounce_is_debounced(&debounce->core);
}

debounce_value_t debounce_bool_get_value(debounce_bool_t *debounce) {
	if (debounce_is_debounced(&debounce->core)) {
		return debounce->last_value ? DEBOUNCE_TRUE : DEBOUNCE_FALSE;
	}
	return DEBOUNCE_BOUNCY;
}

bool debounce_bool_get_raw_value(debounce_bool_t *debounce) {
	return debounce->last_value;
}

bool debounce_bool_update(debounce_bool_t *debounce, bool value) {
	if (value == debounce->last_value) {
		debounce_inc(&debounce->core);
	} else {
		debounce_reset(&debounce->core);
		debounce->last_value = value;
	}
	return debounce_is_debounced(&debounce->core);
}

void debounce_bool_reset(debounce_bool_t *debounce) {
	debounce_reset(&debounce->core);
}
