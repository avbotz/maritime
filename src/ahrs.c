#include <stdio.h>

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

#include <logging/log.h>
#include "threads.h"

#include "ahrs.h"

LOG_MODULE_REGISTER(ahrs, LOG_LEVEL_DBG);

//Function to sample data directly from ahrs
int sample_ahrs(const struct device *dev, struct ahrs_sample *ahrs_sample)
{
	int64_t = time_now = k_uptime_get();
	struct sensor_value accel, gyro, temperature, magn, att;

	int ret;
	ret = sensor_sample_fetch(dev);
	if (ret != 0) goto end;

	ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, &accel);
	if (ret != 0) goto end;
	ret = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, &gyro);
	if (ret != 0) goto end;
  ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
	if (ret != 0) goto end;
	ret = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, &magn);
	if (ret != 0) goto end;
	ret = sensor_channel_get(dev, SENSOR_CHAN_ROTATION, &att);
	if (ret != 0) goto end;

  ahrs_sample->timestamp = time_now;
	ahrs_sample->gyro[0] = sensor_value_to_double(&gyro[0]);
	ahrs_sample->gyro[1] = sensor_value_to_double(&gyro[1]);
	ahrs_sample->gyro[2] = sensor_value_to_double(&gyro[2]);
	ahrs_sample->accel[0] = sensor_value_to_double(&accel[0]);
	ahrs_sample->accel[1] = sensor_value_to_double(&accel[1]);
	ahrs_sample->accel[2] = sensor_value_to_double(&accel[2]);
	ahrs_sample->magn[0] = sensor_value_to_double(&magn[0]);
	ahrs_sample->magn[1] = sensor_value_to_double(&magn[1]);
	ahrs_sample->magn[2] = sensor_value_to_double(&magn[2]);
	ahrs_sample->rotation = sensor_value_to_double(&att);
  ahrs_sample->temperature = sensor_value_to_double(&temperature);

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
	struct k_msgq *ahrs_msgq = (struct k_msgq *) arg2;
	struct ahrs_msgq ahrs_sample;

	while (1) {
    //Draw samples from imu
		int imu_ret = sample_ahrs(dev, &imu_sample);
		if (imu_ret != 0) {
			LOG_ERR("Error sampling AHRS: %d", imu_ret);
		}

		while (k_msgq_put(ahrs_msgq, &ahrs_sample, K_NO_WAIT) != 0) {
			LOG_ERR("Dropping AHRS samples");
			k_msgq_purge(attitude_msgq);
		}
}
#endif /* AHRS_POLL_THREAD */
