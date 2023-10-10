#pragma once

#include <stdint.h>

#define STR(s) #s

#define EMBEDDED_FILE_PTR(name_) \
	binary_ ## name_ ## _start

#define EMBEDDED_FILE_PTR_END(name_) \
	binary_ ## name_ ## _end

#define EMBEDDED_FILE_PTRS(name_) \
	EMBEDDED_FILE_PTR(name_), EMBEDDED_FILE_PTR_END(name_)

#define EMBEDDED_FILE_PTRS_SIZE(name_) \
	EMBEDDED_FILE_PTR(name_), (EMBEDDED_FILE_PTR_END(name_) - EMBEDDED_FILE_PTR(name_))

#define DECLARE_EMBEDDED_FILE(name_) \
	extern const uint8_t binary_ ## name_ ## _start[] asm("_binary_"STR(name_)"_start"); \
	extern const uint8_t binary_ ## name_ ## _end[] asm("_binary_"STR(name_)"_end")

DECLARE_EMBEDDED_FILE(colorcal_16x16x16_12bit_bin);
DECLARE_EMBEDDED_FILE(colorcal_32x32x32_12bit_bin);
