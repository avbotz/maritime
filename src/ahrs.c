// #include <stdio.h>

// #include <zephyr.h>
// #include <device.h>
// #include <drivers/gpio.h>
// #include <drivers/uart.h>

// #include <logging/log.h>
// #include "threads.h"

// #include "ahrs.h"

// LOG_MODULE_REGISTER(ahrs, LOG_LEVEL_DBG);

// //Function to sample data directly from imu
// int sample_imu(const struct device *dev, struct imu_sample *imu_sample)
// {
// 	int64_t = time_now = k_uptime_get();
// 	struct sensor_value accel, gyro, temperature, magn, att;

// 	int ret;
// 	ret = sensor_sample_fetch(dev);
// 	if (ret != 0) goto end;

// 	ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, &accel);
// 	if (ret != 0) goto end;
// 	ret = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, &gyro);
// 	if (ret != 0) goto end;
//   ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
// 	if (ret != 0) goto end;

//   imu_sample->timestamp = time_now;
// 	imu_sample->gyro[0] = sensor_value_to_double(&gyro[0]);
// 	imu_sample->gyro[1] = sensor_value_to_double(&gyro[1]);
// 	imu_sample->gyro[2] = sensor_value_to_double(&gyro[2]);
// 	imu_sample->accel[0] = sensor_value_to_double(&accel[0]);
// 	imu_sample->accel[1] = sensor_value_to_double(&accel[1]);
// 	imu_sample->accel[2] = sensor_value_to_double(&accel[2]);
// 	imu_sample->temperature = sensor_value_to_double(&temperature);

// end:
// 	return ret;
// }

// //Function to sample data directly from magnetometer
// int sample_mag(const struct device *dev, struct mag_sample *mag_sample){
// 	int64_t = time_now = k_uptime_get();
// 	struct sensor_value mag;

// 	int ret;
// 	ret = sensor_sample_fetch(dev);
// 	if (ret != 0) goto end;

// 	ret = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, &magn);
// 	if (ret != 0) goto end;

//   imu_sample->timestamp = time_now;
// 	imu_sample->magn[0] = sensor_value_to_double(&magn[0]);
// 	imu_sample->magn[1] = sensor_value_to_double(&magn[1]);
// 	imu_sample->magn[2] = sensor_value_to_double(&magn[2]);
// }

// static const struct device *get_icm20689_device(void)
// {
// 	const struct device *const dev = GPIO_DT_SPEC_GET(IMU_NODE, gpios);

// 	if (dev == NULL) {
// 		/* No such node, or the node does not have status "okay". */
// 		printk("\nError: no device found.\n");
// 		return NULL;
// 	}

// 	if (!device_is_ready(dev)) {
// 		printk("\nError: IMU \"%s\" is not ready; "
// 		       "check the driver initialization logs for errors.\n",
// 		       dev->name);
// 		return NULL;
// 	}

// 	printk("Found IMU \"%s\", getting sensor data\n", dev->name);
// 	return dev;
// }

// static const struct device *get_rm3100_device(void)
// {
// 	const struct device *const dev = GPIO_DT_SPEC_GET(MAG_NODE, gpios);

// 	if (dev == NULL) {
// 		/* No such node, or the node does not have status "okay". */
// 		printk("\nError: no device found.\n");
// 		return NULL;
// 	}

// 	if (!device_is_ready(dev)) {
// 		printk("\nError: magnetometer \"%s\" is not ready; "
// 		       "check the driver initialization logs for errors.\n",
// 		       dev->name);
// 		return NULL;
// 	}

// 	printk("Found magnetometer \"%s\", getting sensor data\n", dev->name);
// 	return dev;
// }
