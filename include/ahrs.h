#ifndef _MARITIME_AHRS_H
#define _MARITIME_AHRS_H

#include <zephyr/kernel.h>

/*
 * AHRS sample published to ahrs_data_msgq every 20 msec: NED Euler angles in
 * radians (from the WT901's onboard fusion) and their finite-difference
 * angular velocities in rad/s.
 */
struct ahrs_data_s {
	float yaw;
	float pitch;
	float roll;
	float ang_vel_yaw;
	float ang_vel_pitch;
	float ang_vel_roll;
};

extern struct k_msgq ahrs_data_msgq;

int setup_ahrs(void);

#endif /* _MARITIME_AHRS_H */
