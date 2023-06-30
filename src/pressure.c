#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <math.h>
#include <stdio.h>

#include "pressure.h"

K_MSGQ_DEFINE(pressure_data_msgq, sizeof(struct pressure_data_s), 1, 4);
LOG_MODULE_REGISTER(test_pressure, LOG_LEVEL_INF);

static const struct device *const dev = DEVICE_DT_GET_ANY(meas_ms5837);

int setup_pressure() {
/*    static struct sensor_value oversampling_rate = { 8192, 0 };
    LOG_DBG("Setting up pressure");
    if (dev == NULL) {
        LOG_ERR("Could not find MS5837 device, aborting test.");
        return -1;
    }
    if (!device_is_ready(dev)) {
        LOG_ERR("MS5837 device %s is not ready, aborting test.",
                dev->name);
        return -1;
    }
    
    if (sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_OVERSAMPLING,
                &oversampling_rate) != 0) {
        LOG_ERR("Could not set oversampling rate of %d "
                "on MS5837 device, aborting test.",
                oversampling_rate.val1);
        return -1;
    }
*/    return 0;
}

void pressure_thread(void *arg1, void *arg2, void *arg3){
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    LOG_DBG("Started pressure thread");
    static const struct device *const dev = DEVICE_DT_GET_ANY(meas_ms5837);
    LOG_DBG("Got pressure dev");
    if (dev == NULL) {
        LOG_ERR("Could not find MS5837 device, aborting test.");
        return;
    }
    if (!device_is_ready(dev)) {
        LOG_ERR("MS5837 device %s is not ready, aborting test.",
                dev->name);
        return;
    }

/*    if (sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_OVERSAMPLING,
                &oversampling_rate) != 0) {
        LOG_ERR("Could not set oversampling rate of %d "
                "on MS5837 device, aborting test.",
                oversampling_rate.val1);
        return;
    }*/


    while (1) {
        LOG_DBG("Entered pressure thread");
        struct sensor_value press;

        int ret = sensor_sample_fetch(dev);
        if (ret != 0) {
            LOG_DBG("Fetch error %d", ret);
            continue;
        }
        ret = sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
        if (ret != 0) {
            LOG_DBG("Sensor get error %d", ret);
            continue;
        }

	float pressure_sample = press.val1 + press.val2 * pow(10, -6);

        struct pressure_data_s pressure_data;
	pressure_data.depth = (pressure_sample - 5.123) * 1.997;
        while (k_msgq_put(&pressure_data_msgq, &pressure_data, K_NO_WAIT) != 0) {
            k_msgq_put(&pressure_data_msgq, &pressure_data, K_NO_WAIT);
        }
        LOG_DBG("Pressure reading: %f", pressure_sample);
        // k_yield();
        // LOG_DBG("Back in pressure thread");
    }
}

/*
K_THREAD_DEFINE(pressure_thread_id, 2048,
                pressure_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
*/
