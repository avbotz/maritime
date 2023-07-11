#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <math.h>
#include <stdio.h>

#include "pressure.h"

K_MSGQ_DEFINE(pressure_data_msgq, sizeof(struct pressure_data_s), 1, 4);
LOG_MODULE_REGISTER(test_pressure, LOG_LEVEL_INF);
// const struct device *const dev = DEVICE_DT_GET_ANY(meas_ms5837);

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
    ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
                 DT_SPEC_AND_COMMA)
};

static const struct adc_dt_spec adc_channel = adc_channels[0];

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
    if (adc_channel_setup_dt(&adc_channel) < 0) {
        // Handle ADC channel setup error
        LOG_DBG("Error setting up adc");
        return;
    }

    // Use a rolling mean estimator to filter out noise from the adc
    float sample_weight = 0.001;
    float mean_sample = -999;
    struct pressure_data_s pressure_data;

    while (1) 
    {
	for (int i = 0; i < 500; i++)
	{
            // Read integer (no units) that is proportional with pressure
            uint16_t buf;
            struct adc_sequence sequence = {
                .buffer = &buf,
                /* buffer size in bytes, not number of samples */
                .buffer_size = sizeof(buf),
            };

            adc_sequence_init_dt(&adc_channel, &sequence);

            adc_read(adc_channel.dev, &sequence);

            // Initialize the first sample
            if (mean_sample < 0)
            {
                mean_sample = buf;
            }
            else
            {
                mean_sample += sample_weight * (buf - mean_sample);
            }
	}

        // Calibrated with linear regression
        float depth = 0.000136 * (mean_sample - 23569.6);

        pressure_data.depth = depth;
	while (k_msgq_put(&pressure_data_msgq, &pressure_data, K_NO_WAIT) != 0) {
            k_msgq_put(&pressure_data_msgq, &pressure_data, K_NO_WAIT);
        }

        // LOG_DBG("Pressure reading: %i", buf);
        LOG_DBG("Rolling mean estimate: %f", mean_sample);
        k_yield();
	// k_sleep(K_MSEC(1));
    }

    return;
}

K_THREAD_DEFINE(pressure_thread_id, 4096,
                pressure_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
