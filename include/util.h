#ifndef _MARITIME_UTIL_H
#define _MARITIME_UTIL_H

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <stdint.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

int parse_int(char *delim, char **save_ptr);
float parse_float(char *delim, char **save_ptr);
float rad_to_deg(float rad);
float deg_to_rad(float deg);

uint32_t time_us();

#endif
