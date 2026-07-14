#ifndef _MARITIME_AHRS_H
#define _MARITIME_AHRS_H

int setup_ahrs(void);

/* Orientation quaternion, q = {w, x, y, z} */
void get_quaternion(float q[4]);

/*
 * Euler angles in degrees (roll about x, pitch about y, yaw about z).
 * 6-DOF fusion (no magnetometer), so yaw is relative to startup heading
 * and slowly drifts with gyro bias.
 */
void get_rpy(float *roll, float *pitch, float *yaw);

#endif /* _MARITIME_AHRS_H */
