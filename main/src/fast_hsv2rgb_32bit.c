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
#include "fast_hsv2rgb.h"

void fast_hsv2rgb_32bit(uint16_t h, uint16_t s, uint16_t v, uint16_t *r, uint16_t *g , uint16_t *b)
{
  HSV_MONOCHROMATIC_TEST(s, v, r, g, b);// Exit with grayscale if s == 0

  uint8_t sextant = h >> 13;		// Sextant is stored in upper 3 bits

  HSV_SEXTANT_TEST(sextant);		// Optional: Limit hue sextants to defined space

  HSV_POINTER_SWAP(sextant, r, g, b);	// Swap pointers depending which sextant we are in

  *g = v;		// Top level

  // Perform actual calculations

  /*
   * Bottom level: v * (1.0 - s)
   * --> (v * (255 - s) + error_corr + 1) / 256
   */
  uint32_t ww;		// Intermediate result
  ww = v * (HSV_SAT_MAX - s);	// We don't use ~s to prevent size-promotion side effects
  ww += HSV_SAT_MAX;		// Ensure correct rounding in following division
  *b = ww >> 16;		// Divide by scale of s

  uint16_t h_fraction = h & 0x1fff;	// 0...8191
  uint64_t d;			// Intermediate result

  if(!(sextant & 1)) {
    // *r = ...slope_up...;
    d = (uint64_t)v * (uint64_t)((8191UL << 16) - ((uint32_t)s * (8191UL - h_fraction)));
  } else {
    // *r = ...slope_down...;
    d = (uint64_t)v * (uint64_t)((8191UL << 16) - ((uint32_t)s * h_fraction));
  }
  d += (1ULL << 29) - 1;	// Error correction
  *r = d >> 29;
}
