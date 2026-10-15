/* Servos control the ball dropper, torpedo shooter, grabber */
#include "servo.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

#include <errno.h>
#include <stdint.h>

LOG_MODULE_REGISTER(servos, LOG_LEVEL_DBG);

/* The grabber is built only when the overlay has its node */
#define HAVE_GRABBER DT_NODE_EXISTS(DT_NODELABEL(grabber_servo))

static const struct pwm_dt_spec dropper_servo = PWM_DT_SPEC_GET(DT_NODELABEL(dropper_servo));
static const struct pwm_dt_spec shooter_servo = PWM_DT_SPEC_GET(DT_NODELABEL(shooter_servo));
#if HAVE_GRABBER
static const struct pwm_dt_spec grabber_servo = PWM_DT_SPEC_GET(DT_NODELABEL(grabber_servo));
#endif

static const uint32_t dropper_min_pulse = DT_PROP(DT_NODELABEL(dropper_servo), min_pulse);
static const uint32_t dropper_max_pulse = DT_PROP(DT_NODELABEL(dropper_servo), max_pulse);
static const uint32_t dropper_mid_pulse = (dropper_min_pulse + dropper_max_pulse) / 2;

static const uint32_t shooter_min_pulse = DT_PROP(DT_NODELABEL(shooter_servo), min_pulse);
static const uint32_t shooter_mid_pulse = DT_PROP(DT_NODELABEL(shooter_servo), mid_pulse);
static const uint32_t shooter_max_pulse = DT_PROP(DT_NODELABEL(shooter_servo), max_pulse);

#if HAVE_GRABBER
/* Grabber only uses only part of the servo's range, from 37% to 66% of servo's range */
static const uint32_t grabber_min_pulse = DT_PROP(DT_NODELABEL(grabber_servo), min_pulse);
static const uint32_t grabber_max_pulse = DT_PROP(DT_NODELABEL(grabber_servo), max_pulse);
#endif

/*
 * min pulse = min angle = left hole
 * max pulse = max angle = right hole
 */

void setup_servos()
{
	/* Set all of the servos to go to the middle (neutral) position */
	if (!device_is_ready(dropper_servo.dev)) {
		LOG_ERR("Dropper device is not ready");
	}

	if (!device_is_ready(shooter_servo.dev)) {
		LOG_ERR("Shooter device is not ready");
	}

	int ret = pwm_set_pulse_dt(&dropper_servo, dropper_mid_pulse);
	if (ret < 0) {
		LOG_ERR("Failed to set pwm pulse for dropper servo, error code %i", ret);
	}
	ret = pwm_set_pulse_dt(&shooter_servo, shooter_min_pulse);
	if (ret < 0) {
		LOG_ERR("Failed to set pwm pulse for shooter servo, error code %i", ret);
	}

#if HAVE_GRABBER
	if (!device_is_ready(grabber_servo.dev)) {
		LOG_ERR("Grabber device is not ready");
	}
	ret = pwm_set_pulse_dt(&grabber_servo, grabber_min_pulse);
	if (ret < 0) {
		LOG_ERR("Failed to set pwm pulse for grabber servo, error code %i", ret);
	}
#endif
}

/* pulse_us clamped to [min_ns, max_ns], as nanoseconds. Clamped in us so the host cannot
 * overflow the conversion. */
static uint32_t clamp_pulse(int pulse_us, uint32_t min_ns, uint32_t max_ns)
{
	const int min_us = min_ns / NSEC_PER_USEC;
	const int max_us = max_ns / NSEC_PER_USEC;

	return (uint32_t)CLAMP(pulse_us, min_us, max_us) * NSEC_PER_USEC;
}

int set_pulse(enum servo_type type, int pulse_us)
{
	const struct pwm_dt_spec *servo;
	uint32_t pulse_ns;

	switch (type) {
	case SERVO_DROPPER:
		servo = &dropper_servo;
		pulse_ns = clamp_pulse(pulse_us, dropper_min_pulse, dropper_max_pulse);
		break;
	case SERVO_SHOOTER:
		servo = &shooter_servo;
		pulse_ns = clamp_pulse(pulse_us, shooter_min_pulse, shooter_max_pulse);
		break;
#if HAVE_GRABBER
	case SERVO_GRABBER:
		servo = &grabber_servo;
		pulse_ns = clamp_pulse(pulse_us, grabber_min_pulse, grabber_max_pulse);
		break;
#endif
	default:
		LOG_WRN("No servo %d in the overlay", type);
		return -ENODEV;
	}

	int ret = pwm_set_pulse_dt(servo, pulse_ns);
	if (ret < 0) {
		LOG_ERR("Failed to set pulse width for servo %d, error code %i", type, ret);
	}
	return ret;
}

void drop(int idx, int value)
{
	LOG_DBG("Drop for %i %i", idx, value);

	int ret = 0;

	// If requested, set servo to neutral position
	if (value == 0) {
		ret = pwm_set_pulse_dt(&dropper_servo, dropper_mid_pulse);
	} else if (value == 1) {
		// Set servo to drop the 0th (right) ball
		if (idx == 0) {
			ret = pwm_set_pulse_dt(&dropper_servo, dropper_max_pulse);
		}
		// Set servo to drop the 1st (left) ball
		else if (idx == 1) {
			ret = pwm_set_pulse_dt(&dropper_servo, dropper_min_pulse);
		}
	}

	if (ret < 0) {
		LOG_ERR("Failed to set pulse width for dropper servo, error code %i", ret);
	}
}

// void grab(float value)
//{
//	/*
//	 * value = the fraction of closed you grabber to be (0 to 1).
//	 * Eg. value = 0.6 = 60% closed grabber.
//	 */
//	LOG_DBG("Opened grab for %f", value);
//	uint32_t pulse_width =
//		(uint32_t)(grabber_min_pulse + value * (grabber_max_pulse - grabber_min_pulse));
//	int ret = pwm_set_pulse_dt(&grabber_servo, pulse_width);
//	// int ret = pwm_set_pulse_dt(&shooter_servo, value);
//	LOG_DBG("Pulse width %i", pulse_width);
//	if (ret < 0) {
//		LOG_ERR("Failed to set pulse width for grabber, error code %i", ret);
//	}
// }

void shoot(int idx, int value)
{
	LOG_DBG("Opened shoot for %i %i", idx, value);
	int ret = 0;
	// If requested, set servo to neutral position (closes both holes)
	if (value == 0) {
		ret = pwm_set_pulse_dt(&shooter_servo, shooter_min_pulse);
	} else if (value == 1) {
		// Set servo to open the 0th (right) hole
		if (idx == 0) {
			ret = pwm_set_pulse_dt(&shooter_servo, shooter_mid_pulse);
		}
		// Set servo to open the 1st (left) hole
		else if (idx == 1) {
			ret = pwm_set_pulse_dt(&shooter_servo, shooter_max_pulse);
		}
	}
	if (ret < 0) {
		LOG_ERR("Failed to shooter pwm pulse, error code %i", ret);
	}
}
