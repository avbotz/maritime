/*
 * Various helper functions to help do stuff
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "maritime/util.h"

#define PI 3.141592653


/* 
 * Parse a token of a string pointer received over UART as an int.
 * Assumes we have already called strtok_r(), and we are passing 
 * that save_ptr (the save state of the string) pointer in
 */
int parse_int(char *delim, char **save_ptr)
{
    char *token = strtok_r(NULL, delim, save_ptr);
    return atoi(token);
}

/*
 * Parse a token of a string pointer as a float
 */
float parse_float(char *delim, char **save_ptr)
{
    char *token = strtok_r(NULL, delim, save_ptr);
    return atof(token);
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