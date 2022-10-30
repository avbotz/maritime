#include <stdio.h>
#include <math.h>

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#include "threads.h"
#include "pressure_sensor.h"

LOG_MODULE_REGISTER(dvl, LOG_LEVEL_DBG);

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

	dvl_sample->timestamp = time_now;
	dvl_sample->pressureKPA = sensor_value_to_double(&pressure);

end:
	return ret;
}

/* Pressure sensor poll loop for when trigger is not enabled */
void pressure_sensor_poll_thread_entry(void *arg1, void *arg2, void *unused3)
{
	LOG_DBG("Initializing pressure sensor poll thread");

	const struct device *dev = (struct device *) arg1;

  //Sets up addresses to queue messages
	struct k_msgq *pressure_msgq = (struct k_msgq *) arg2;

	struct pressure_sample pressure_sample;

	while (1) {
    //Draw samples from pressure sensor
		int pressure_sensor_ret = sample_pressure_sensor(dev, &pressure_sample);
		if (pressure_sensor_ret != 0) {
			LOG_ERR("Error sampling pressure sensor: %d", pressure_sensor_ret);
		}

		while (k_msgq_put(pressure_msgq, &pressure_sample, K_NO_WAIT) != 0) {
			LOG_ERR("Dropping pressure sensor samples");
			k_msgq_purge(pressure_msgq);
		}

		k_sleep(K_MSEC(1)); /* 1Khz poll rate */
	}
}
#endif /* PRESSURE_POLL_THREAD */
