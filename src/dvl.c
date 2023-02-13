#include <stdio.h>
#include <math.h>

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#include "threads.h"
#include "dvl.h"

LOG_MODULE_REGISTER(dvl, LOG_LEVEL_DBG);

int sample_dvl(const struct device *dev, struct dvl_sample *dvl_sample)
{
	int64_t time_now;
	struct sensor_value posChangeX, posChangeY, posChangeZ;
	time_now = k_uptime_get();

	int ret;
	ret = sensor_sample_fetch(dev);
	if (ret != 0) goto end;

	ret = sensor_channel_get(dev, SENSOR_CHAN_POS_DX, &posChangeX);
	if (ret != 0) goto end;
	ret = sensor_channel_get(dev, SENSOR_CHAN_POS_DY, &posChangeY);
	if (ret != 0) goto end;
	ret = sensor_channel_get(dev, SENSOR_CHAN_POS_DZ, &posChangeZ);
	if (ret != 0) goto end;

	dvl_sample->timestamp = time_now;
	dvl_sample->changeX = sensor_value_to_double(&posChangeX);
	dvl_sample->changeY = sensor_value_to_double(&posChangeY);
	dvl_sample->changeZ = sensor_value_to_double(&posChangeZ);

end:
	return ret;
}

static const struct device *get_DVL_device(void)
{
	const struct device *const dev = GPIO_DT_SPEC_GET(DVL_NODE, gpios);

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: DVL \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found DVL \"%s\", getting sensor data\n", dev->name);
	return dev;
}

#endif /* DVL_POLL_THREAD */
