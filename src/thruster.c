#include "thruster.h"

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <math.h>
#include <stdint.h>

LOG_MODULE_REGISTER(thrusters, LOG_LEVEL_DBG);

/* Not DT_FOREACH_CHILD_SEP: clang-format turns its (,) into (, ), which checkpatch rejects */
#define THRUSTER_PWM_SPEC(node_id) PWM_DT_SPEC_GET(node_id),

static const struct pwm_dt_spec thruster_devices[] = {
	DT_FOREACH_CHILD(DT_PATH(thrusters), THRUSTER_PWM_SPEC)};
BUILD_ASSERT(ARRAY_SIZE(thruster_devices) == NUM_THRUSTERS);

static const uint32_t THRUSTER_MIN_PULSE = DT_PROP(DT_PATH(thrusters), min_pulse);
static const uint32_t THRUSTER_MAX_PULSE = DT_PROP(DT_PATH(thrusters), max_pulse);
static const float MAX_POWER = DT_PROP(DT_PATH(thrusters), max_power) / 100.0f;

int thruster_init(void)
{
	for (int i = 0; i < NUM_THRUSTERS; i++) {
		if (!device_is_ready(thruster_devices[i].dev)) {
			LOG_ERR("Thruster %d device not ready", i);
			return -ENODEV;
		}
	}
	return 0;
};

void thruster_send_all(const float thrusts[NUM_THRUSTERS])
{
	for (int i = 0; i < NUM_THRUSTERS; i++) {
		thruster_send(i, thrusts[i]);
	}
};

void thruster_send_all_uniform(float thrust)
{
	for (int i = 0; i < NUM_THRUSTERS; i++) {
		thruster_send(i, thrust);
	}
}

void thruster_stop(void)
{
	thruster_send_all_uniform(0.0f);
}

void thruster_send(int thruster, float thrust)
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

	int ret = pwm_set_pulse_dt(&thruster_devices[thruster], pulse_width);

	if (ret < 0) {
		LOG_ERR("Failed to set pulse for thruster %d, error code %i", thruster, ret);
	}
}
