/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016  B. Stultiens
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#ifndef __HSV_FAST_HSV2RGB_H__
#define __HSV_FAST_HSV2RGB_H__

#include <stdint.h>

#define HSV_HUE_SEXTANT		8192
#define HSV_HUE_STEPS		(6 * HSV_HUE_SEXTANT)

#define HSV_HUE_MIN		0
#define HSV_HUE_MAX		(HSV_HUE_STEPS - 1)
#define HSV_SAT_MIN		0
#define HSV_SAT_MAX		65535
#define HSV_VAL_MIN		0
#define HSV_VAL_MAX		65535

/* Options: */
#define HSV_USE_SEXTANT_TEST	/* Limit the hue to 0...360 degrees */
#define HSV_USE_ASSEMBLY	/* Optimize code using assembly */

#ifdef __cplusplus
extern "C" {
#endif

void fast_hsv2rgb_32bit(uint16_t h, uint16_t s, uint16_t v, uint16_t *r, uint16_t *g , uint16_t *b);

#ifdef __cplusplus
}
#endif


/*
 * Macros that are common to all implementations
 */
#define HSV_MONOCHROMATIC_TEST(s,v,r,g,b) \
  do { \
    if(!(s)) { \
       *(r) = *(g) = *(b) = (v); \
      return; \
    } \
  } while(0)

#ifdef HSV_USE_SEXTANT_TEST
#define HSV_SEXTANT_TEST(sextant) \
  do { \
    if((sextant) > 5) { \
      (sextant) = 5; \
    } \
  } while(0)
#else
#define HSV_SEXTANT_TEST(sextant) do { ; } while(0)
#endif

/*
 * Pointer swapping:
 * 	sext.	r g b	r<>b	g<>b	r <> g	result
 *	0 0 0	v u c			!u v c	u v c
 *	0 0 1	d v c				d v c
 *	0 1 0	c v u	u v c			u v c
 *	0 1 1	c d v	v d c		d v c	d v c
 *	1 0 0	u c v		u v c		u v c
 *	1 0 1	v c d		v d c	d v c	d v c
 *
 * if(sextant & 2)
 * 	r <-> b
 *
 * if(sextant & 4)
 * 	g <-> b
 *
 * if(!(sextant & 6) {
 * 	if(!(sextant & 1))
 * 		r <-> g
 * } else {
 * 	if(sextant & 1)
 * 		r <-> g
 * }
 */
#define HSV_SWAPPTR(a,b)	do { uint16_t *tmp = (a); (a) = (b); (b) = tmp; } while(0)
#define HSV_POINTER_SWAP(sextant,r,g,b) \
  do { \
    if((sextant) & 2) { \
      HSV_SWAPPTR((r), (b)); \
    } \
    if((sextant) & 4) { \
      HSV_SWAPPTR((g), (b)); \
    } \
    if(!((sextant) & 6)) { \
      if(!((sextant) & 1)) { \
        HSV_SWAPPTR((r), (g)); \
      } \
    } else { \
      if((sextant) & 1) { \
        HSV_SWAPPTR((r), (g)); \
      } \
    } \
  } while(0)
#endif
