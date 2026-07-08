#include "util.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

bool parse_int_arg(char *token, int *value)
{
	char *end;
	long parsed;

	if (token == NULL || token[0] == '\0') {
		return false;
	}

	errno = 0;
	parsed = strtol(token, &end, 10);
	if (errno != 0 || *end != '\0') {
		return false;
	}

	*value = (int)parsed;
	return true;
}

bool parse_float_arg(char *token, float *value)
{
	char *end;
	float parsed;

	if (token == NULL || token[0] == '\0') {
		return false;
	}

	errno = 0;
	parsed = strtof(token, &end);
	if (errno != 0 || *end != '\0') {
		return false;
	}

	*value = parsed;
	return true;
}
