#pragma once

#include <stddef.h>
#include <stdint.h>

#define CHACHA20_KEY_SIZE	32

typedef struct chacha20_ctx {
	uint32_t keystream32[16];
	size_t position;

	uint8_t key[32];
	uint8_t nonce[12];
	uint64_t counter;

	uint32_t state[16];
} chacha20_ctx_t;

void chacha20_init(chacha20_ctx_t *ctx, const uint8_t key[32], const uint8_t nonce[12], uint64_t counter);
void chacha20_xor(chacha20_ctx_t *ctx, uint8_t *dst, const uint8_t *src, size_t n_bytes);

static inline void chacha20_xor_inplace(chacha20_ctx_t *ctx, uint8_t *data, size_t n_bytes) {
	chacha20_xor(ctx, data, data, n_bytes);
}
