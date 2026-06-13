#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>

#include "thruster.h"

LOG_MODULE_REGISTER(thrusters, LOG_LEVEL_DBG);

#define THRUSTER_COUNT 8
#define THRUSTER_PWM_SETTLE_MS 20

static const struct pwm_dt_spec thruster0 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster0));
static const struct pwm_dt_spec thruster1 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster1));
static const struct pwm_dt_spec thruster2 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster2));
static const struct pwm_dt_spec thruster3 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster3));
static const struct pwm_dt_spec thruster4 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster4));
static const struct pwm_dt_spec thruster5 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster5));
static const struct pwm_dt_spec thruster6 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster6));
static const struct pwm_dt_spec thruster7 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster7));

static const struct pwm_dt_spec *thruster_devices[THRUSTER_COUNT] = {
	&thruster0, &thruster1, &thruster2, &thruster3,
	&thruster4, &thruster5, &thruster6, &thruster7
};

static const uint32_t thruster_min_pulse = 1100;
static const uint32_t thruster_max_pulse = 1900;

int setup_thrusters()
{
	for (int i = 0; i < THRUSTER_COUNT; i++) {
		if (!device_is_ready(thruster_devices[i]->dev)) {
			LOG_DBG("Thruster %d device not ready", i);
			// return -1;
		}
	}
	return 0;
};

int send_thrusts(int idx, float power)
{
	if ((idx < 0) || (idx >= THRUSTER_COUNT)) {
		return -EINVAL;
	}

	if (power < -1.0f) {
		power = -1.0f;
	}
	if (power > 1.0f) {
		power = 1.0f;
	}

	uint32_t pulse_width =
		(uint32_t)((power + 1.0f) * (thruster_max_pulse - thruster_min_pulse) / 2 +
			   thruster_min_pulse);
	printf("Setting thruster %d to pulse width %u us\n", idx, pulse_width);

	int ret = pwm_set_pulse_dt(thruster_devices[idx], PWM_USEC(pulse_width));
	if (ret < 0) {
		LOG_DBG("Failed to set pulse for thruster %d, error code %i", idx, ret);
	}

	k_sleep(K_MSEC(THRUSTER_PWM_SETTLE_MS));

	return ret;
};
