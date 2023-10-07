#include "strutil.h"

#include <string.h>

bool str_starts_with(const char *str, const char *start) {
	return strstr(str, start) == str;
}
