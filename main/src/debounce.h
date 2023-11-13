#pragma once

#include <stdbool.h>

typedef struct debounce {
	unsigned int threshold;
	unsigned int count;
} debounce_t;

typedef struct debounce_bool {
	debounce_t core;
	bool last_value;
} debounce_bool_t;

typedef enum debounce_value {
	DEBOUNCE_BOUNCY,
	DEBOUNCE_TRUE,
	DEBOUNCE_FALSE
} debounce_value_t;

void debounce_bool_init(debounce_bool_t *debounce, unsigned int threshold);
bool debounce_bool_is_debounced(debounce_bool_t *debounce);
bool debounce_bool_get_raw_value(debounce_bool_t *debounce);
debounce_value_t debounce_bool_get_value(debounce_bool_t *debounce);
bool debounce_bool_update(debounce_bool_t *debounce, bool value);
void debounce_bool_reset(debounce_bool_t *debounce);
