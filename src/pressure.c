/* @author: Kush Nayak */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <math.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

static const struct adc_dt_spec adc_channel = adc_channels[0];

LOG_MODULE_REGISTER(main);

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

	// sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_OVERSAMPLING,
	// 	&oversampling_rate);

	if (adc_channel_setup_dt(&adc_channel) < 0) {
	    // Handle ADC channel setup error
	    return;
	}
}

uint16_t pressure_get_reading()
{
	// Return integer that is proportional with pressure
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	adc_sequence_init_dt(&adc_channel, &sequence);

	adc_read(adc_channel.dev, &sequence);

	return buf;
}

int main() {
	while (1) {
		printk("%d\n", pressure_get_reading());
		k_sleep(K_MSEC(500));
	}
	
}