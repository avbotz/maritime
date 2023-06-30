#ifndef _MARITIME_PRESSURE_H
#define _MARITIME_PRESSURE_H

#include <zephyr/kernel.h>

int setup_pressure(void);
extern struct k_msgq pressure_data_msgq;

struct pressure_data_s {
    float depth;
};

#endif

