#include "fp32_vec.h"

void fp_vec3_add(fp_vec3_t *a, const fp_vec3_t *b) {
	a->x += b->x;
	a->y += b->y;
	a->z += b->z;
}

void fp_vec3_sub(fp_vec3_t *a, const fp_vec3_t *b) {
	a->x -= b->x;
	a->y -= b->y;
	a->z -= b->z;
}

void fp_vec3_mul(fp_vec3_t *a, fp_t scale) {
	a->x = fp_mul(a->x, scale);
	a->y = fp_mul(a->y, scale);
	a->z = fp_mul(a->z, scale);
}

void fp_vec3_div(fp_vec3_t *a, fp_t scale) {
	a->x = fp_div(a->x, scale);
	a->y = fp_div(a->y, scale);
	a->z = fp_div(a->z, scale);
}

fp_t fp_vec3_mag(const fp_vec3_t *a) {
	fp_t x2 = fp_mul(a->x, a->x);
	fp_t y2 = fp_mul(a->y, a->y);
	fp_t z2 = fp_mul(a->z, a->z);
	return fp_sqrt(x2 + y2 + z2);
}
