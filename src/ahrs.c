#include <stdio.h>

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

#include <logging/log.h>
#include "threads.h"

#include "ahrs.h"

LOG_MODULE_REGISTER(ahrs, LOG_LEVEL_DBG);

//Function to sample data directly from imu
int sample_imu(const struct device *dev, struct imu_sample *imu_sample)
{
	int64_t = time_now = k_uptime_get();
	struct sensor_value accel, gyro, temperature;

	int ret;
	ret = sensor_sample_fetch(dev);
	if (ret != 0) goto end;

	ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, &accel);
	if (ret != 0) goto end;
	ret = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, &gyro);
	if (ret != 0) goto end;
  ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
	if (ret != 0) goto end;

  imu_sample->timestamp = time_now;
	imu_sample->gyro[0] = sensor_value_to_double(&gyro[0]);
	imu_sample->gyro[1] = sensor_value_to_double(&gyro[1]);
	imu_sample->gyro[2] = sensor_value_to_double(&gyro[2]);
	imu_sample->accel[0] = sensor_value_to_double(&accel[0]);
	imu_sample->accel[1] = sensor_value_to_double(&accel[1]);
	imu_sample->accel[2] = sensor_value_to_double(&accel[2]);
  imu_sample->temperature = sensor_value_to_double(&temperature);

end:
	return ret;
}

//Function to sample data directly from magnetometer
int sample_magnetometer(const struct device *dev, struct magnetometer_sample *magnetometer_sample)
{
	int64_t = time_now = k_uptime_get();
	struct sensor_value magn;

	int ret;
	ret = sensor_sample_fetch(dev);
	if (ret != 0) goto end;

  ret = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, &magn);
	if (ret != 0) goto end;

  magnetometer_sample->timestamp = time_now;
  magnetometer_sample->magn[0] = sensor_value_to_double(&magn[0]);
	magnetometer_sample->magn[1] = sensor_value_to_double(&magn[1]);
	magnetometer_sample->magn[2] = sensor_value_to_double(&magn[2]);

end:
	return ret;
}

#ifdef AHRS_POLL_THREAD

/* IMU+magnetometer poll loop for when trigger is not enabled */
void ahrs_poll_thread_entry(void *arg1, void *arg2, void *arg3)
{
	LOG_DBG("Initializing AHRS poll thread");

	const struct device *dev = (struct device *) arg1;

  //Sets up addresses to queue messages
	struct k_msgq *imu_msgq = (struct k_msgq *) arg2;
  struct k_msgq *magn_msgq = (struct k_msgq *) arg3;

	struct imu_sample imu_sample;
  struct magnetometer_sample magn_sample;

	while (1) {
    //Draw samples from imu
		int imu_ret = sample_imu(dev, &imu_sample);
		if (imu_ret != 0) {
			LOG_ERR("Error sampling IMU: %d", imu_ret);
		}

		while (k_msgq_put(imu_msgq, &imu_sample, K_NO_WAIT) != 0) {
			LOG_ERR("Dropping IMU samples");
			k_msgq_purge(imu_msgq);
		}

    //Draw samples from magnetometer
    int magn_ret = sample_imu(dev, &magn_sample);
		if (magn_ret != 0) {
			LOG_ERR("Error sampling magnetometer: %d", magn_ret);
		}

		while (k_msgq_put(magn_msgq, &magn_sample, K_NO_WAIT) != 0) {
			LOG_ERR("Dropping magnetometer samples");
			k_msgq_purge(magn_msgq);
		}

		k_sleep(K_MSEC(1)); /* 1Khz poll rate */
	}
}
#endif /* AHRS_POLL_THREAD */
