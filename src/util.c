#include "util.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>

#define PI 3.14159265358979323846

int64_t time_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

double deg_to_rad(double deg)
{
	return deg * (PI / 180.0);
}

double rad_to_deg(double rad)
{
	return rad * (180.0 / PI);
}

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
