/* @author: Kush Nayak */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <math.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

struct sensor_value oversampling_rate = { 8192, 0 };
const struct device *const dev = DEVICE_DT_GET_ANY(meas_ms5837);

void init_pressure()
{
	/* Initialize communication with the pressure sensor */
	// if (dev == NULL) {
	// 	LOG_ERR("Could not find MS5837 device, aborting test.");
	// 	return;
	// }
	// if (!device_is_ready(dev)) {
	// 	LOG_ERR("MS5837 device %s is not ready, aborting test.",
	// 		dev->name);
	// 	return;
	// }

	// if (sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_OVERSAMPLING,
	// 			&oversampling_rate) != 0) {
	// 	LOG_ERR("Could not set oversampling rate of %d "
	// 		"on MS5837 device, aborting test.",
	// 		oversampling_rate.val1);
	// 	return;
	// }

	sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_OVERSAMPLING,
		&oversampling_rate);
}

float pressure_get_depth()
{
	/* Calculates current depth based on pressure reading */
	struct sensor_value press;

	sensor_sample_fetch(dev);
	sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);

	float pressure_Pa = press.val1 + press.val2 * pow(10, -6);
	float depth = (pressure_Pa - 101325) / 9806.38;
	return depth;
}
