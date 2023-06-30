#ifndef _MARITIME_AHRS_H
#define _MARITIME_AHRS_H

#include <zephyr/kernel.h>

#define AHRS_OUTPUT_RATE 30 

int setup_ahrs(void);
extern struct k_msgq ahrs_data_msgq;

struct ahrs_data_s {
    float yaw;
    float pitch;
    float roll;
    float ang_vel_yaw;
    float ang_vel_roll;
    float ang_vel_pitch;
};

#endif

