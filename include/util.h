#ifndef _MARITIME_UTIL_H
#define _MARITIME_UTIL_H

#include <stdbool.h>

bool parse_int_arg(char *token, int *value);
bool parse_float_arg(char *token, float *value);

float clamp(float val, float low, float high);

#endif /* _MARITIME_UTIL_H */
