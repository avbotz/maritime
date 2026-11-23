#include "thruster.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/math/interpolation.h>

#include <math.h>
LOG_MODULE_REGISTER(thrusters, LOG_LEVEL_DBG);

#define THRUSTERS DT_PATH(thrusters)

#define THRUSTER_BIT(node_id)   BIT(DT_PROP(node_id, thruster_index))
#define THRUSTER_ENTRY(node_id) [DT_PROP(node_id, thruster_index)] = PWM_DT_SPEC_GET(node_id)

/*
 * Disable clang-format to resolve conflicts with checkpatch. Per Zephyr style guide, checkpatch is
 * the authoritative source for formatting.
 */
/* clang-format off */
static const struct pwm_dt_spec thruster_devices[NUM_THRUSTERS] = {DT_FOREACH_CHILD_SEP(THRUSTERS, THRUSTER_ENTRY, (,))};
/* clang-format on */
BUILD_ASSERT(DT_CHILD_NUM(THRUSTERS) == NUM_THRUSTERS, "wrong number of thrusters");
BUILD_ASSERT((DT_FOREACH_CHILD_SEP(THRUSTERS, THRUSTER_BIT, (|))) == BIT_MASK(NUM_THRUSTERS),
	     "thruster-index values must be 0-7, each used once");

static const uint32_t THRUSTER_MIN_PULSE = DT_PROP(THRUSTERS, min_pulse);
static const uint32_t THRUSTER_MAX_PULSE = DT_PROP(THRUSTERS, max_pulse);

static const float MAX_THROTTLE = DT_PROP(THRUSTERS, max_throttle) / 100.0f;
static const uint32_t MAX_CURRENT = DT_PROP(THRUSTERS, max_current);

static const int32_t CURRENT_PULSES[] = DT_PROP(THRUSTERS, current_pulses);
static const int32_t CURRENT_MILLIAMPS[] = DT_PROP(THRUSTERS, current_milliamps);

BUILD_ASSERT(ARRAY_SIZE(CURRENT_PULSES) == ARRAY_SIZE(CURRENT_MILLIAMPS),
	     "current-pulses and current-milliamps must be the same length");
BUILD_ASSERT(DT_PROP_BY_IDX(THRUSTERS, current_pulses, 0) <= DT_PROP(THRUSTERS, min_pulse),
	     "current-pulses must start at or below min-pulse");
BUILD_ASSERT(DT_PROP_LAST(THRUSTERS, current_pulses) >= DT_PROP(THRUSTERS, max_pulse),
	     "current-pulses must end at or above max-pulse");

#define ASSERT_ASCENDING(node_id, prop, i)                                                         \
	IF_ENABLED(DT_PROP_HAS_IDX(node_id, prop, UTIL_INC(i)),                                    \
	(BUILD_ASSERT(DT_PROP_BY_IDX(node_id, prop, i) < DT_PROP_BY_IDX(node_id, prop, UTIL_INC(i)), "current-pulses must ascend");))
DT_FOREACH_PROP_ELEM(THRUSTERS, current_pulses, ASSERT_ASCENDING)

int thruster_init(void)
{
	for (int i = 0; i < NUM_THRUSTERS; i++) {
		if (!device_is_ready(thruster_devices[i].dev)) {
			LOG_ERR("Thruster %d device not ready", i);
			return -ENODEV;
		}
	}
	return 0;
}

static uint32_t throttle_to_pulse(float throttle)
{
	/* Normalize throttle (-1.0 to 1.0) to 0.0 to 1.0 */
	float throttle_norm = (throttle + 1.0f) / 2;

	return THRUSTER_MIN_PULSE +
	       (uint32_t)(throttle_norm * (THRUSTER_MAX_PULSE - THRUSTER_MIN_PULSE));
}

static float pulse_to_current(uint32_t pulse)
{
	return linear_interpolate(CURRENT_PULSES, CURRENT_MILLIAMPS, ARRAY_SIZE(CURRENT_PULSES),
				  pulse);
}

static float total_current(const float throttles[NUM_THRUSTERS], float scale)
{
	float total = 0.0f;

	for (int i = 0; i < NUM_THRUSTERS; i++) {
		total += pulse_to_current(throttle_to_pulse(scale * throttles[i]));
	}
	return total;
}

static void write_pulse(int thruster, float throttle)
{
	int ret = pwm_set_pulse_dt(&thruster_devices[thruster], throttle_to_pulse(throttle));

	if (ret < 0) {
		LOG_ERR("Failed to set pulse for thruster %d, error code %i", thruster, ret);
	}
}

void thruster_send_all(const float throttles[NUM_THRUSTERS])
{
	float limited[NUM_THRUSTERS];
	float scale = 1.0f;

	for (int i = 0; i < NUM_THRUSTERS; i++) {
		if (!isfinite(throttles[i])) {
			limited[i] = 0.0f;
			continue;
		}

		limited[i] = CLAMP(throttles[i], -MAX_THROTTLE, MAX_THROTTLE);
	}

	if (total_current(limited, 1.0f) > MAX_CURRENT) {
		float fits = 0.0f;
		float over = 1.0f;

		/* binary search to find largest scale that keeps total current under MAX_CURRENT */
		for (int i = 0; i < 10; i++) {
			float middle = (fits + over) / 2;

			if (total_current(limited, middle) > MAX_CURRENT) {
				over = middle;
			} else {
				fits = middle;
			}
		}

		scale = fits;
	}

	for (int i = 0; i < NUM_THRUSTERS; i++) {
		write_pulse(i, scale * limited[i]);
	}
}

void thruster_stop(void)
{
	for (int i = 0; i < NUM_THRUSTERS; i++) {
		write_pulse(i, 0.0f);
	}
}
