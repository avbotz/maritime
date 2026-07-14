#ifndef _MARITIME_UTIL_H
#define _MARITIME_UTIL_H

#include <stdbool.h>
#include <stdint.h>

bool parse_int_arg(char *token, int *value);
bool parse_float_arg(char *token, float *value);

/* Microseconds since boot */
int64_t time_us(void);

double deg_to_rad(double deg);
double rad_to_deg(double rad);

#endif /* _MARITIME_UTIL_H */
