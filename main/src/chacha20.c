/*
 * Tiny ChaCha20 implementation based on public domain code
 */

#include <string.h>

#include "chacha20.h"

static uint32_t rotl32(uint32_t x, int n)
{
	return (x << n) | (x >> (32 - n));
}

static uint32_t pack4(const uint8_t *a)
{
	uint32_t res = 0;

	res |= (uint32_t)a[0] << 0 * 8;
	res |= (uint32_t)a[1] << 1 * 8;
	res |= (uint32_t)a[2] << 2 * 8;
	res |= (uint32_t)a[3] << 3 * 8;
	return res;
}

static void chacha20_init_block(chacha20_ctx_t *ctx, const uint8_t key[32], const uint8_t nonce[12])
{
	const uint8_t *magic_constant = (uint8_t*)"expand 32-byte k";

	memcpy(ctx->key, key, sizeof(ctx->key));
	memcpy(ctx->nonce, nonce, sizeof(ctx->nonce));

	ctx->state[0] = pack4(magic_constant + 0 * 4);
	ctx->state[1] = pack4(magic_constant + 1 * 4);
	ctx->state[2] = pack4(magic_constant + 2 * 4);
	ctx->state[3] = pack4(magic_constant + 3 * 4);
	ctx->state[4] = pack4(key + 0 * 4);
	ctx->state[5] = pack4(key + 1 * 4);
	ctx->state[6] = pack4(key + 2 * 4);
	ctx->state[7] = pack4(key + 3 * 4);
	ctx->state[8] = pack4(key + 4 * 4);
	ctx->state[9] = pack4(key + 5 * 4);
	ctx->state[10] = pack4(key + 6 * 4);
	ctx->state[11] = pack4(key + 7 * 4);
	// 64 bit counter initialized to zero by default.
	ctx->state[12] = 0;
	ctx->state[13] = pack4(nonce + 0 * 4);
	ctx->state[14] = pack4(nonce + 1 * 4);
	ctx->state[15] = pack4(nonce + 2 * 4);

	memcpy(ctx->nonce, nonce, sizeof(ctx->nonce));
}

static void chacha20_block_set_counter(chacha20_ctx_t *ctx, uint64_t counter)
{
	ctx->state[12] = (uint32_t)counter;
	ctx->state[13] = pack4(ctx->nonce + 0 * 4) + (uint32_t)(counter >> 32);
}

static void chacha20_quaterround(uint32_t *keystream, unsigned int a, unsigned int b, unsigned int c, unsigned int d) {
	keystream[a] += keystream[b]; keystream[d] = rotl32(keystream[d] ^ keystream[a], 16);
	keystream[c] += keystream[d]; keystream[b] = rotl32(keystream[b] ^ keystream[c], 12);
	keystream[a] += keystream[b]; keystream[d] = rotl32(keystream[d] ^ keystream[a], 8);
	keystream[c] += keystream[d]; keystream[b] = rotl32(keystream[b] ^ keystream[c], 7);
}

static void chacha20_block_next(chacha20_ctx_t *ctx)
{
	int i;
	uint32_t *counter;

	// This is where the crazy voodoo magic happens.
	// Mix the bytes a lot and hope that nobody finds out how to undo it.
	for (i = 0; i < 16; i++) ctx->keystream32[i] = ctx->state[i];

	for (i = 0; i < 10; i++) {
		chacha20_quaterround(ctx->keystream32, 0, 4, 8, 12);
		chacha20_quaterround(ctx->keystream32, 1, 5, 9, 13);
		chacha20_quaterround(ctx->keystream32, 2, 6, 10, 14);
		chacha20_quaterround(ctx->keystream32, 3, 7, 11, 15);
		chacha20_quaterround(ctx->keystream32, 0, 5, 10, 15);
		chacha20_quaterround(ctx->keystream32, 1, 6, 11, 12);
		chacha20_quaterround(ctx->keystream32, 2, 7, 8, 13);
		chacha20_quaterround(ctx->keystream32, 3, 4, 9, 14);
	}

	for (i = 0; i < 16; i++) ctx->keystream32[i] += ctx->state[i];

	counter = ctx->state + 12;
	// increment counter
	counter[0]++;
	if (!counter[0]) {
		// wrap around occured, increment higher 32 bits of counter
		counter[1]++;
		// Limited to 2^64 blocks of 64 bytes each.
		// If you want to process more than 1180591620717411303424 bytes
		// you have other problems.
		// We could keep counting with counter[2] and counter[3] (nonce),
		// but then we risk reusing the nonce which is very bad.
//		assert(0 != counter[1]); Ignore that for now, won't happen
	}
}

void chacha20_init(chacha20_ctx_t *ctx, const uint8_t key[32], const uint8_t nonce[12], uint64_t counter)
{
	memset(ctx, 0, sizeof(chacha20_ctx_t));

	chacha20_init_block(ctx, key, nonce);
	chacha20_block_set_counter(ctx, counter);

	ctx->counter = counter;
	ctx->position = 64;
}

void chacha20_xor(chacha20_ctx_t *ctx, uint8_t *dst, const uint8_t *src, size_t n_bytes)
{
	size_t i;
	uint8_t *keystream8 = (uint8_t*)ctx->keystream32;

	for (i = 0; i < n_bytes; i++) {
		if (ctx->position >= 64) {
			chacha20_block_next(ctx);
			ctx->position = 0;
		}
		dst[i] = src[i] ^ keystream8[ctx->position];
		ctx->position++;
	}
}
