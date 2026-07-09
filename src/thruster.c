#include "thruster.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

#include <stdbool.h>
#include <stdint.h>

LOG_MODULE_REGISTER(thrusters, LOG_LEVEL_DBG);

static const struct pwm_dt_spec thruster0 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster0));
static const struct pwm_dt_spec thruster1 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster1));
static const struct pwm_dt_spec thruster2 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster2));
static const struct pwm_dt_spec thruster3 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster3));
static const struct pwm_dt_spec thruster4 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster4));
static const struct pwm_dt_spec thruster5 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster5));
static const struct pwm_dt_spec thruster6 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster6));
static const struct pwm_dt_spec thruster7 = PWM_DT_SPEC_GET(DT_NODELABEL(thruster7));

static const struct pwm_dt_spec *thruster_devices[8] = {&thruster0, &thruster1, &thruster2,
							&thruster3, &thruster4, &thruster5,
							&thruster6, &thruster7};

static const uint32_t THRUSTER_MIN_PULSE = 1100;
static const uint32_t THRUSTER_MAX_PULSE = 1900;
static const float MAX_POWER = 0.6;

int setup_thrusters()
{
	for (int i = 0; i < 8; i++) {
		if (!device_is_ready(thruster_devices[i]->dev)) {
			LOG_DBG("Thruster %d device not ready", i);
			return -1;
		}
	}
	return 0;
};

void send_thrusts(float thrusts[8])
{
	for (int i = 0; i < 8; i++) {
		send_thrust(i, thrusts[i]);
	}
};

void send_thrust(int thruster, float thrust)
{
	if (thruster < 0 || thruster >= 8) {
		return;
	}

	thrust = CLAMP(thrust, -MAX_POWER, MAX_POWER);

	uint32_t pulse_width =
		(uint32_t)((thrust + 1.0f) * (THRUSTER_MAX_PULSE - THRUSTER_MIN_PULSE) / 2 +
			   THRUSTER_MIN_PULSE);

	int ret = pwm_set_pulse_dt(thruster_devices[thruster], PWM_USEC(pulse_width));
	if (ret < 0) {
		LOG_DBG("Failed to set pulse for thruster %d, error code %i", thruster, ret);
	}
}
