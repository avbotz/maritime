#ifndef _MARITIME_PRESSURE_H
#define _MARITIME_PRESSURE_H

#include <zephyr/kernel.h>

int setup_pressure(void);
/* Takes the current reading as the surface (depth 0) */
int reset_reference(void);
/*
 * Takes the current reading as the surface, but only if it is within limit_m of
 * the surface already (or there is no surface reading yet), so that a kill switch
 * release underwater does not move the surface. -ERANGE if it is not.
 */
int rezero_near_surface(float limit_m);
int get_raw_pressure(void);
/* Depth below the surface reading [m, positive down]; -EIO if the sensor cannot be read */
int get_depth_meters(float *depth_m);

#endif
