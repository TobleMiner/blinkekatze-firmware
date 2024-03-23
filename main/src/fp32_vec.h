#pragma once

#include "fp32.h"

#define FPVEC3STR "{ "FPSTR", "FPSTR", "FPSTR" }"
#define FPVEC32STR(vec_) FP2STR((vec_).x), FP2STR((vec_).y), FP2STR((vec_).z)

typedef struct fp_vec3 {
	fp_t x;
	fp_t y;
	fp_t z;
} fp_vec3_t;

void fp_vec3_add(fp_vec3_t *a, const fp_vec3_t *b);
void fp_vec3_sub(fp_vec3_t *a, const fp_vec3_t *b);
void fp_vec3_mul(fp_vec3_t *a, fp_t scale);
void fp_vec3_div(fp_vec3_t *a, fp_t scale);
fp_t fp_vec3_mag(const fp_vec3_t *a);
