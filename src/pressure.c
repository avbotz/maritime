#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "pressure.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>

LOG_MODULE_REGISTER(pressure, LOG_LEVEL_DBG);

static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

static int16_t sample_buf;
static struct adc_sequence sequence = {
	.buffer = &sample_buf,
	.buffer_size = sizeof(sample_buf),
};

/* Raw ADC reading at the surface: taken at startup, and again at each kill switch release
 * near the surface */
static int initial_sample;
static bool have_initial_sample;

/* Raw readings averaged into one, and how many tries they get */
#define PRESSURE_SAMPLES   10
#define PRESSURE_MAX_READS 30

/* ADC counts per metre of water, from a linear fit of readings against measured depth */
#define COUNTS_PER_METER 365.0f

int get_raw_pressure()
{
	int ret = adc_read_dt(&adc_channel, &sequence);
	if (ret != 0) {
		LOG_ERR("Failed to read ADC: %d", ret);
		return ret;
	}

	return sample_buf;
}

/* The mean of PRESSURE_SAMPLES good readings, or -EIO if the ADC does not give that many
 * in PRESSURE_MAX_READS tries (a disconnected sensor reads 0). Bounded, so a dead sensor
 * cannot hang the main loop and with it the thruster watchdog. */
static int get_raw_averaged_pressure()
{
	int sum = 0;
	int good = 0;
	for (int i = 0; i < PRESSURE_MAX_READS && good < PRESSURE_SAMPLES; i++) {
		int raw = get_raw_pressure();
		if (raw > 0) {
			sum += raw;
			good++;
		}
	}
	return good == PRESSURE_SAMPLES ? sum / good : -EIO;
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

	return reset_reference();
}

static float raw_to_depth(int raw)
{
	return (float)(raw - initial_sample) / COUNTS_PER_METER;
}

int reset_reference()
{
	return rezero_near_surface(INFINITY);
}

int rezero_near_surface(float limit_m)
{
	int sample = get_raw_averaged_pressure();
	if (sample < 0) {
		return sample;
	}
	if (have_initial_sample && fabsf(raw_to_depth(sample)) > limit_m) {
		return -ERANGE;
	}
	initial_sample = sample;
	have_initial_sample = true;
	LOG_DBG("Surface pressure: %d", initial_sample);
	return 0;
}

int get_depth_meters(float *depth_m)
{
	if (!have_initial_sample) {
		return -EIO;
	}
	int raw = get_raw_averaged_pressure();
	if (raw < 0) {
		return -EIO;
	}
	*depth_m = raw_to_depth(raw);
	return 0;
}
