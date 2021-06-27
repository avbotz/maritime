#ifndef _MARITIME_AHRS_H
#define _MARITIME_AHRS_H

#include <zephyr.h>
#include <device.h>

// #define MPU9250_I2C_NODE DT_INST(0, invensense_mpu6050)
// // #if DT_NODE_HAS_STATUS(MPU9250_I2C_NODE, okay)
// #define MPU9250_I2C_LABEL DT_LABEL(MPU9250_I2C_NODE)
// // #else
// // #error "Compatible MPU9250 IMU node not found."
// // #endif
//
// #define AK8963_I2C_NODE DT_INST(0, asahi-kasei_ak8975)
// // #if DT_NODE_HAS_STATUS(AK8963_MAG_NODE, okay)
// #define AK8963_I2C_LABEL DT_LABEL(AK8963_I2C_NODE)
// // #else
// // #error "Compatible AK8963 Magnetometer node not found."
// // #endif
#define IMU_NODE DT_ALIAS(imu)
#if DT_NODE_HAS_STATUS(IMU_NODE, okay)
#define IMU_LABEL DT_LABEL(IMU_NODE)
#else
#error "Unsupported board."
#endif

#define MAG_NODE DT_ALIAS(mag)
#if DT_NODE_HAS_STATUS(MAG_NODE, okay)
#define MAG_LABEL DT_LABEL(MAG_NODE)
#else
#error "Unsupported board."
#endif


#endif // _MARITIME_AHRS_H
