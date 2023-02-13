#include <stdio.h>
#include <math.h>

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#include "threads.h"
#include "pressure_sensor.h"

LOG_MODULE_REGISTER(pressure_sensor, LOG_LEVEL_DBG);

int sample_pressure_sensor(const struct device *dev, struct pressure_sensor_sample *pressure_sensor_sample)
{
	int64_t time_now;
	struct sensor_value pressure;
	time_now = k_uptime_get();

	int ret;
	ret = sensor_sample_fetch(dev);
	if (ret != 0) goto end;

	ret = sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure);
	if (ret != 0) goto end;

	pressure_sensor_sample->timestamp = time_now;
	pressure_sensor_sample->pressureKPA = sensor_value_to_double(&pressure);

end:
	return ret;
}

static const struct device *get_pressure_sensor_device(void)
{
	const struct device *const dev = GPIO_DT_SPEC_GET(PRESSURE_SENSOR_NODE, gpios);

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: pressure sensor \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found pressure sensor \"%s\", getting sensor data\n", dev->name);
	return dev;
}

#endif /* PRESSURE_POLL_THREAD */
