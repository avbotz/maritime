#ifndef _MARITIME_UTIL_H
#define _MARITIME_UTIL_H

int parse_int(char *delim, char **save_ptr);

float parse_float(char *delim, char **save_ptr);

float rad_to_deg(float rad);

float deg_to_rad(float deg);

#endif