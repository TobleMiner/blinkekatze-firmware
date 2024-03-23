#pragma once

#include <stdint.h>

#define FIXEDPOINT_BASE 12
#define FIXEDPOINT_PRECISE_EXP

#if FIXEDPOINT_BASE == 12
#define FP_E	11134
#define FP_PI	12868
#define FP_PI_2	 6434
#define FP_PI_4	 3217
#define FP_TAU	25736
#else
#error "Unsupported fixedpoint base"
#endif

#define FPSTR "%c%lu.%lu"
#define FP2STR(x_) ((x_) & (1UL << 31) ? '-' : '+'), (unsigned long)fp_num_to_int(x_), (unsigned long)fp_frac_to_int(x_)

typedef int32_t fp_t;

fp_t fp_div(fp_t num, fp_t div);
fp_t fp_log(fp_t val);
fp_t fp_exp(fp_t val);
fp_t fp_pow(fp_t base, fp_t exponent);
fp_t fp_sqrt(fp_t val);
fp_t fp_mul(fp_t x, fp_t y);
fp_t fp_sin(fp_t val);
float fp_to_float(fp_t val);
fp_t float_to_fp(float val);

uint32_t fp_num_to_int(fp_t val);
uint32_t fp_frac_to_int(fp_t val);

static inline fp_t int_to_fp(int val) {
	return (fp_t)val << FIXEDPOINT_BASE;
}

static inline fp_t fp_cos(fp_t val) {
	return fp_sin(val + FP_PI_2);
}

static inline fp_t fp_mod(fp_t x, fp_t y) {
	return x % y;
}

static inline fp_t fp_abs(fp_t x) {
	return x < 0 ? -x : x;
}
