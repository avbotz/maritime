#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "pressure.h"

LOG_MODULE_REGISTER(pressure, LOG_LEVEL_DBG);

static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

static int16_t sample_buf;
static struct adc_sequence sequence = {
	.buffer = &sample_buf,
	.buffer_size = sizeof(sample_buf),
};

/* Raw ADC reading taken at startup, assumed to be at the surface */
static int initial_sample;

int get_raw_pressure()
{
	int ret = adc_read_dt(&adc_channel, &sequence);
	if (ret != 0) {
		LOG_ERR("Failed to read ADC: %d", ret);
		return ret;
	}

	return sample_buf;
}

int get_raw_averaged_pressure()
{
	int sum = 0;
	for (int i = 0; i < 10; i++) {
		int raw = get_raw_pressure();
		if (raw <= 0) {
			i--;
			continue;
		}
		sum += raw;
	}
	return sum / 10;
}

int setup_pressure()
{
	if (!adc_is_ready_dt(&adc_channel)) {
		LOG_ERR("ADC controller %s not ready", adc_channel.dev->name);
		return -ENODEV;
	}

	int ret = adc_channel_setup_dt(&adc_channel);
	if (ret != 0) {
		LOG_ERR("Failed to setup ADC channel: %d", ret);
		return ret;
	}

	ret = adc_sequence_init_dt(&adc_channel, &sequence);
	if (ret != 0) {
		LOG_ERR("Failed to init ADC sequence: %d", ret);
		return ret;
	}

	initial_sample = get_raw_averaged_pressure();
	if (initial_sample < 0) {
		return -1;
	}

	LOG_DBG("Surface pressure: %d", initial_sample);

	return 0;
}

int reset_reference()
{
	initial_sample = get_raw_averaged_pressure();
	if (initial_sample < 0) {
		return -1;
	}
	LOG_DBG("Surface pressure: %d", initial_sample);
	return 0;
}

float raw_pressure_to_depth(int raw_pressure, int initial_sample)
{
	return (raw_pressure - initial_sample) / 365.0;
}

float get_depth_meters()
{
	int raw = get_raw_averaged_pressure();

	LOG_DBG("Raw pressure: %d", raw);

	if (raw > 0) {
		return raw_pressure_to_depth(raw, initial_sample);
	}

	return -1.0f;
}
