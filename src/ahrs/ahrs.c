/*

 * Implements data input, estimation and fusion from
 * the magnetometer and IMU for attitude estimation.
 *
 * @author Vincent Wang <vwangsf@gmail.com>
 */

#include <logging/log.h>

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>

#include <pubsub/pubsub.h>

#include <stdio.h>

#include "ahrs.h"

/*LOG_MODULE_REGISTER(ahrs, LOG_LEVEL_DBG)*/;
LOG_MODULE_REGISTER(ahrs, LOG_LEVEL_ERR);

#define AHRS_THREAD_SIZE 1024
#define AHRS_PRIORITY 1

static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
		 h, min, s, ms);
	return buf;
}

static int process_ak8963(const struct device *dev)
{
	int rc = sensor_sample_fetch(dev);
    struct sensor_value magn[3];
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, magn);
	}

    if (rc == 0) {
		printf("[imu] [%s]\n"
		       "      mag %f %f %f gauss\n",
               now_str(),
               sensor_value_to_double(&magn[0]),
               sensor_value_to_double(&magn[1]),
               sensor_value_to_double(&magn[2]));
    } else {
        printf("[mag] sample fetch/get failed: %d\n", rc);
    }

    return rc;
}

static int process_mpu9250(const struct device *dev)
{
	struct sensor_value temperature;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
					gyro);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP,
					&temperature);
	}
	if (rc == 0) {
		printf("[imu] [%s]:%g Cel\n"
		       "      accel %f %f %f m/s/s\n"
		       "      gyro  %f %f %f rad/s\n",
		       now_str(),
		       sensor_value_to_double(&temperature),
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]),
		       sensor_value_to_double(&gyro[0]),
		       sensor_value_to_double(&gyro[1]),
		       sensor_value_to_double(&gyro[2]));
	} else {
		printf("[imu] sample fetch/get failed: %d\n", rc);
	}

	return rc;
}

static void ahrs_thread(void *unused1, void *unused2, void *unused3)
{
    ARG_UNUSED(unused1);
    ARG_UNUSED(unused2);
    ARG_UNUSED(unused3);

    LOG_INF("Initializing AHRS thread");

    // TODO: PR MPU9250 SPI support into upstream
    const struct device *mpu9250_imu = device_get_binding(IMU_LABEL);
    if (mpu9250_imu == NULL) {
        LOG_ERR("Unable to bind IMU! Make sure it's plugged in and working.");
        return;
    }

    const struct device *ak8963_mag = device_get_binding(MAG_LABEL);
    if (ak8963_mag == NULL) {
        LOG_ERR("Unable to bind magnetometer! Make sure it's plugged in and working.");
        return;
    }

    while (1) {
        int rc = process_mpu9250(mpu9250_imu);
        if (rc != 0) {
            break;
        }

        rc = process_ak8963(ak8963_mag);
        if (rc != 0) {
            break;
        }

        k_sleep(K_MSEC(10));
    }
}

K_THREAD_DEFINE(ahrs_thread_id, AHRS_THREAD_SIZE,
                ahrs_thread, NULL, NULL, NULL,
                AHRS_PRIORITY, 0, 0);
