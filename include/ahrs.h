#ifndef _MARITIME_AHRS_H
#define _MARITIME_AHRS_H

#include <zephyr/kernel.h>

#include <stdint.h>

// AHRS output rate in Hz
#define AHRS_OUTPUT_RATE 50

// Total bytes the message can store
#define MSG_SZ 512

int setup_ahrs(void);
extern struct k_msgq ahrs_data_msgq;

struct ahrs_data_s {
    uint32_t ts_us;
    float yaw;
    float pitch;
    float roll;
    float ang_vel_yaw;
    float ang_vel_roll;
    float ang_vel_pitch;
};

#endif

