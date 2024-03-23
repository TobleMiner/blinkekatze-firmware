#include <stdbool.h>

#include "fp32.h"

#if FIXEDPOINT_BASE == 12
#define TRIG_TABLE_SIZE 128

const fp_t sine_table[TRIG_TABLE_SIZE] = {
	   0,   50,  100,  150,  200,  251,  301,  351,  401,  451,  501,  551,  601,  650,  700,  749,
	 799,  848,  897,  946,  995, 1043, 1092, 1140, 1189, 1237, 1284, 1332, 1379, 1427, 1474, 1520,
	1567, 1613, 1659, 1705, 1751, 1796, 1841, 1886, 1930, 1975, 2018, 2062, 2105, 2148, 2191, 2233,
	2275, 2317, 2358, 2399, 2439, 2480, 2519, 2559, 2598, 2637, 2675, 2713, 2750, 2787, 2824, 2860,
	2896, 2931, 2966, 3000, 3034, 3068, 3101, 3134, 3166, 3197, 3229, 3259, 3289, 3319, 3348, 3377,
	3405, 3433, 3460, 3487, 3513, 3538, 3563, 3588, 3612, 3635, 3658, 3680, 3702, 3723, 3744, 3764,
	3784, 3803, 3821, 3839, 3856, 3873, 3889, 3904, 3919, 3933, 3947, 3960, 3973, 3985, 3996, 4007,
	4017, 4026, 4035, 4043, 4051, 4058, 4065, 4071, 4076, 4080, 4084, 4088, 4091, 4093, 4094, 4095,
};
#else
#error "Unsupported fixedpoint base"
#endif

/* Fast clz, stolen from hackers's delight */
static int32_t clz(uint32_t x)
{
	int32_t	n;

	if (x == 0) {
		return 32;
	}

	n = 0;
	if (x <= 0x0000FFFF) {n = n + 16; x = x << 16;}
	if (x <= 0x00FFFFFF) {n = n + 8; x = x << 8;}
	if (x <= 0x0FFFFFFF) {n = n + 4; x = x << 4;}
	if (x <= 0x3FFFFFFF) {n = n + 2; x = x << 2;}
	if (x <= 0x7FFFFFFF) {n = n + 1;}

	return n;
}

fp_t fp_div(fp_t num, fp_t div)
{
	int64_t lnum = num;

	return (lnum * (1LL << FIXEDPOINT_BASE)) / div;
}

fp_t fp_log(fp_t val)
{
	int32_t fracv, intv, y, ysq, fracr, bitpos;
#if FIXEDPOINT_BASE == 10
	const int32_t ilog2e = 710;
#elif FIXEDPOINT_BASE == 11
	const int32_t ilog2e = 1420;
#elif FIXEDPOINT_BASE == 12
	const int32_t ilog2e = 2839;
#elif FIXEDPOINT_BASE == 13
	const int32_t ilog2e = 5678;
#elif FIXEDPOINT_BASE == 14
	const int32_t ilog2e = 11357;
#elif FIXEDPOINT_BASE == 15
	const int32_t ilog2e = 22713;
#elif FIXEDPOINT_BASE == 16
	const int32_t ilog2e = 45426;
#else
#error "Unsupported fixedpoint base"
#endif
	const int32_t ln_denoms[] = {
		(1 << FIXEDPOINT_BASE) /  1,
		(1 << FIXEDPOINT_BASE) /  3,
		(1 << FIXEDPOINT_BASE) /  5,
		(1 << FIXEDPOINT_BASE) /  7,
		(1 << FIXEDPOINT_BASE) /  9,
		(1 << FIXEDPOINT_BASE) / 11,
		(1 << FIXEDPOINT_BASE) / 13,
		(1 << FIXEDPOINT_BASE) / 15,
		(1 << FIXEDPOINT_BASE) / 17,
		(1 << FIXEDPOINT_BASE) / 19,
		(1 << FIXEDPOINT_BASE) / 21,
	};

	bitpos = (32 - (FIXEDPOINT_BASE - 1)) - clz(val);
	if (bitpos >= 0) {
		bitpos++;
		fracv = val >> bitpos;
	} else if (bitpos < 0) {
		bitpos++;
		fracv = val << (-bitpos);
	}

	intv = bitpos * ilog2e;
	y = ((int64_t)(fracv - (1 << FIXEDPOINT_BASE)) << FIXEDPOINT_BASE) / (fracv + (1 << FIXEDPOINT_BASE));
	ysq = (y * y) >> FIXEDPOINT_BASE;

	fracr = ln_denoms[10];
	fracr = (((int64_t)fracr * ysq) >> FIXEDPOINT_BASE) + ln_denoms[9];
	fracr = (((int64_t)fracr * ysq) >> FIXEDPOINT_BASE) + ln_denoms[8];
	fracr = (((int64_t)fracr * ysq) >> FIXEDPOINT_BASE) + ln_denoms[7];
	fracr = (((int64_t)fracr * ysq) >> FIXEDPOINT_BASE) + ln_denoms[6];
	fracr = (((int64_t)fracr * ysq) >> FIXEDPOINT_BASE) + ln_denoms[5];
	fracr = (((int64_t)fracr * ysq) >> FIXEDPOINT_BASE) + ln_denoms[4];
	fracr = (((int64_t)fracr * ysq) >> FIXEDPOINT_BASE) + ln_denoms[3];
	fracr = (((int64_t)fracr * ysq) >> FIXEDPOINT_BASE) + ln_denoms[2];
	fracr = (((int64_t)fracr * ysq) >> FIXEDPOINT_BASE) + ln_denoms[1];
	fracr = (((int64_t)fracr * ysq) >> FIXEDPOINT_BASE) + ln_denoms[0];
	fracr =	 ((int64_t)fracr * (y << 1)) >> FIXEDPOINT_BASE;

	return intv + fracr;
}

fp_t fp_exp(fp_t val)
{
	fp_t x = val;

	if (!val) {
		return 1 << FIXEDPOINT_BASE;
	}

	x -= (((int64_t)x * (fp_log(x) - val)) >> FIXEDPOINT_BASE);
	x -= (((int64_t)x * (fp_log(x) - val)) >> FIXEDPOINT_BASE);
	x -= (((int64_t)x * (fp_log(x) - val)) >> FIXEDPOINT_BASE);
	x -= (((int64_t)x * (fp_log(x) - val)) >> FIXEDPOINT_BASE);
#ifdef FIXEDPOINT_PRECISE_EXP
	x -= (((int64_t)x * (fp_log(x) - val)) >> FIXEDPOINT_BASE);
	x -= (((int64_t)x * (fp_log(x) - val)) >> FIXEDPOINT_BASE);
#endif
	return x;
}

fp_t fp_pow(fp_t base, fp_t exponent)
{
	return fp_exp(((int64_t)exponent * fp_log(base)) >> FIXEDPOINT_BASE);
}

fp_t fp_sqrt(fp_t val)
{
	uint32_t x;
	int32_t bitpos;
	uint64_t v;

	if (!val) {
		return 0;
	}

	bitpos = (32 - FIXEDPOINT_BASE) - clz(val);

	if (bitpos > 0) {
		x = (1u << FIXEDPOINT_BASE) << (bitpos >> 1);
	} else if(bitpos < 0) {
		x = (1u << FIXEDPOINT_BASE) << ((uint32_t)(-bitpos) << 1);
	} else {
		x = (1u << FIXEDPOINT_BASE);
	}

	v = (uint64_t)val << (FIXEDPOINT_BASE - 1);

	x = (x >> 1) + v / x;
	x = (x >> 1) + v / x;
	x = (x >> 1) + v / x;
	x = (x >> 1) + v / x;
	return x;
}

fp_t fp_mul(fp_t x, fp_t y)
{
	int64_t res = ((int64_t)x * (int64_t)y);
	return (res >> FIXEDPOINT_BASE) + ((res >> (FIXEDPOINT_BASE - 1)) & 1);
}

fp_t fp_sin(fp_t val) {
	unsigned int bin;
	bool invert_x = false;
	bool invert_y = false;
	fp_t res;

	if (val < 0) {
		val = -val;
		val %= FP_TAU;
		val = FP_TAU - val;
	} else {
		val %= FP_TAU;
	}

	invert_x = (val > FP_PI_2 && val < FP_PI) ||
		   (val >= FP_PI + FP_PI_2);
	invert_y = val >= FP_PI;

	if (invert_x) {
		val -= FP_PI_2;
	}

	if (invert_y) {
		val -= FP_PI;
	}

	bin = val * (TRIG_TABLE_SIZE - 1) / FP_PI_2;

	if (invert_x) {
		bin = TRIG_TABLE_SIZE - bin - 1;
	}

	res = sine_table[bin];

	if (invert_y) {
		res = -res;
	}

	return res;
}

float fp_to_float(fp_t val)
{
	int32_t integer = val >> FIXEDPOINT_BASE;
	float fval = integer;
	uint32_t frac = val & ((((fp_t)1) << FIXEDPOINT_BASE) - 1);

	fval += (float)frac / (1 << FIXEDPOINT_BASE);
	return fval;
}

#define ABS(x) ((x) < 0 ? -(x) : (x))

fp_t float_to_fp(float val)
{
	int32_t num = ABS(val);
	uint32_t frac;
	fp_t posval;

	frac = (ABS(val) - num) * (1 << FIXEDPOINT_BASE);
	posval = ((fp_t)num << FIXEDPOINT_BASE) | (fp_t)frac;
	return val < 0 ? -posval : posval;
}

uint32_t fp_num_to_int(fp_t val) {
	if (val & (1UL << 31)) {
		val = ~val;
	}
	return val >> FIXEDPOINT_BASE;
}

static const uint32_t frac_bit_lookups[] = {
        500000000UL,
        250000000UL,
        125000000UL,
         62500000UL,
         31250000UL,
         15625000UL,
          7812500UL,
          3906250UL,
          1953125UL,
           976563UL,
           488281UL,
           244141UL,
           122070UL,
            61035UL,
            30518UL,
            15259UL
};

uint32_t fp_frac_to_int(fp_t val) {
	uint32_t frac = 0;
	for (unsigned int i = 0; i < FIXEDPOINT_BASE; i++) {
		if (val & (1UL << (FIXEDPOINT_BASE - 1 - i))) {
			frac += frac_bit_lookups[i];
		}
	}
	if (val & (1UL << 31)) {
		frac = 1000000000UL - frac;
	}
	while (!(frac % 10) && frac) frac /= 10;
	return frac;
}
