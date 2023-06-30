#include <zephyr/kernel.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "util.h"
#include "strtok.h"

const float DEG_TO_RAD = M_PI / 180.0f;
const float RAD_TO_DEG = 180.0f / M_PI;

uint32_t time_us() { return k_cyc_to_us_ceil32(k_cycle_get_32()); }

int parse_int(char *delim, char **save_ptr) {
    char *token = strtok_r(NULL, delim, save_ptr);
    return atoi(token);
}

float parse_float(char *delim, char **save_ptr) {
    char *token = strtok_r(NULL, delim, save_ptr);
    return atof(token);
}

float rad_to_deg(float rad) {
    return rad * RAD_TO_DEG;
}

float deg_to_rad(float deg) {
    return deg * DEG_TO_RAD;
}

