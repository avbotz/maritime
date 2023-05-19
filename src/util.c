/*
 * Various helper functions to help do stuff
 */
#include <stdio.h>
#include <math.h>
#include "util.h"

#define PI 3.141592653


/* 
 * Parse a token of a string pointer received over UART as an int.
 * Assumes we have already called strtok(), and we are passing 
 * that pointer in
 */
int parse_int(char *ptr, char *delim)
{
    ptr = strtok(NULL, delim);
    return atoi(ptr);
}

/*
 * Parse a token of a string pointer as a float
 */
float parse_float(char *ptr, char *delim)
{
    ptr = strtok(NULL, delim);
    return atof(ptr);
}

/*
 * Conversions (todo: put these in a util.c file)
 */
float rad_to_deg(float rad)
{
    return rad / PI * 180.;
}

float deg_to_rad(float deg)
{
    return deg / 180. * PI;
}