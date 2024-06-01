#include <errno.h>
#include <sys/types.h>

#include "util.h"

void strntr(char* str, size_t len, char a, char b) {
  while(len-- > 0) {
    if(*str == a) {
      *str = b;
    }
    str++;
  }
}

ssize_t hex_decode(uint8_t *outptr, size_t outlen, const char *inptr, size_t inlen) {
	uint8_t *dst = outptr;
	size_t i;

	if (inlen % 2) {
		return -EINVAL;
	}

	if (outlen < inlen / 2) {
		return -EINVAL;
	}

	for (i = 0; i < inlen; i += 2) {
		*outptr++ = hex_to_byte(inptr);
		inptr += 2;
	}

	return inlen / 2;
}

ssize_t hex_decode_inplace(uint8_t *ptr, size_t len) {
	uint8_t *dst = ptr;
	size_t i;

	if (len % 2) {
		return -EINVAL;
	}

	for (i = 0; i < len; i++) {
		*dst++ = hex_to_byte(ptr);
		ptr += 2;
	}

	return len / 2;
}

ssize_t hex_encode(const uint8_t *src, size_t src_len, char *dst, size_t dst_len) {
	if (dst_len % 2) {
		return -EINVAL;
	}

	size_t bytes_out = 0;
	while (dst_len >= 2 && src_len) {
		uint8_t byt = *src++;
		src_len--;
		*dst++ = nibble_to_hex(byt >> 4);
		*dst++ = nibble_to_hex(byt & 0xf);
		dst_len -= 2;
		bytes_out += 2;
	}

	return bytes_out;
}
