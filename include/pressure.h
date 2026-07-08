#ifndef _MARITIME_PRESSURE_H
#define _MARITIME_PRESSURE_H

#include <zephyr/kernel.h>

int setup_pressure(void);
int get_raw_pressure(void);
float get_depth_meters(void);

#endif
