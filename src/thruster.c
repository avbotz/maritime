#include "thruster.h"

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <math.h>
#include <stdint.h>

LOG_MODULE_REGISTER(thrusters, LOG_LEVEL_DBG);

static const struct pwm_dt_spec thruster_devices[NUM_THRUSTERS] = {
	PWM_DT_SPEC_GET(DT_NODELABEL(thruster0)), PWM_DT_SPEC_GET(DT_NODELABEL(thruster1)),
	PWM_DT_SPEC_GET(DT_NODELABEL(thruster2)), PWM_DT_SPEC_GET(DT_NODELABEL(thruster3)),
	PWM_DT_SPEC_GET(DT_NODELABEL(thruster4)), PWM_DT_SPEC_GET(DT_NODELABEL(thruster5)),
	PWM_DT_SPEC_GET(DT_NODELABEL(thruster6)), PWM_DT_SPEC_GET(DT_NODELABEL(thruster7)),
};

static const uint32_t THRUSTER_MIN_PULSE = 1100;
static const uint32_t THRUSTER_MAX_PULSE = 1900;
/* Maximum of |thrust|. Corresponds to 1260-1740 us to the Basic ESCs. */
static const float MAX_POWER = 0.6f;

int setup_thrusters(void)
{
	for (int i = 0; i < NUM_THRUSTERS; i++) {
		if (!device_is_ready(thruster_devices[i].dev)) {
			LOG_ERR("Thruster %d device not ready", i);
			return -ENODEV;
		}
	}
	return 0;
};

void send_thrusts(const float thrusts[NUM_THRUSTERS])
{
	for (int i = 0; i < NUM_THRUSTERS; i++) {
		send_thrust(i, thrusts[i]);
	}
};

void send_all_thrusts(float thrust)
{
	for (int i = 0; i < NUM_THRUSTERS; i++) {
		send_thrust(i, thrust);
	}
}

void stop_thrusters(void)
{
	send_all_thrusts(0.0f);
}

void send_thrust(int thruster, float thrust)
{
	if (thruster < 0 || thruster >= NUM_THRUSTERS) {
		return;
	}

	/* CLAMP would pass NaN through to the pulse width, and turn inf into full power */
	if (!isfinite(thrust)) {
		thrust = 0.0f;
	}
	thrust = CLAMP(thrust, -MAX_POWER, MAX_POWER);

	uint32_t pulse_width =
		(uint32_t)((thrust + 1.0f) * (THRUSTER_MAX_PULSE - THRUSTER_MIN_PULSE) / 2 +
			   THRUSTER_MIN_PULSE);

	int ret = pwm_set_pulse_dt(&thruster_devices[thruster], PWM_USEC(pulse_width));

	if (ret < 0) {
		LOG_DBG("Failed to set pulse for thruster %d, error code %i", thruster, ret);
	}
}
