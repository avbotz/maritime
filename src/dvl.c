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
	ret = sensor_channel_get(dev, SENSOR_CHAN_POS_DX, &posChangeY);
	if (ret != 0) goto end;
	ret = sensor_channel_get(dev, SENSOR_CHAN_POS_DX, &posChangeZ);
	if (ret != 0) goto end;

	dvl_sample->timestamp = time_now;
	dvl_sample->changeX = sensor_value_to_double(&posChangeX);
	dvl_sample->changeY = sensor_value_to_double(&posChangeY);
	dvl_sample->changeZ = sensor_value_to_double(&posChangeZ);

end:
	return ret;
}

/* DVL poll loop for when trigger is not enabled */
void dvl_poll_thread_entry(void *arg1, void *arg2, void *unused3)
{
	LOG_DBG("Initializing DVL poll thread");

	const struct device *dev = (struct device *) arg1;

  //Sets up addresses to queue messages
	struct k_msgq *dvl_msgq = (struct k_msgq *) arg2;

	struct dvl_sample dvl_sample;

	while (1) {
    //Draw samples from dvl
		int dvl_ret = sample_imu(dev, &dvl_sample);
		if (dvl_ret != 0) {
			LOG_ERR("Error sampling DVL: %d", dvl_ret);
		}

		while (k_msgq_put(dvl_msgq, &dvl_sample, K_NO_WAIT) != 0) {
			LOG_ERR("Dropping DVL samples");
			k_msgq_purge(dvl_msgq);
		}

		k_sleep(K_MSEC(1)); /* 1Khz poll rate */
	}
}
#endif /* DVL_POLL_THREAD */
