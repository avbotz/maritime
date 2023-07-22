/* Servos control the ball dropper, torpedo shooter, grabber */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

#include "servo.h"

LOG_MODULE_REGISTER(servos, LOG_LEVEL_INF);

static const struct pwm_dt_spec dropper_servo = PWM_DT_SPEC_GET(DT_NODELABEL(dropper_servo));
static const struct pwm_dt_spec grabber_servo = PWM_DT_SPEC_GET(DT_NODELABEL(grabber_servo));
static const struct pwm_dt_spec shooter_servo = PWM_DT_SPEC_GET(DT_NODELABEL(shooter_servo));

static const uint32_t dropper_min_pulse = DT_PROP(DT_NODELABEL(dropper_servo), min_pulse);
static const uint32_t dropper_max_pulse = DT_PROP(DT_NODELABEL(dropper_servo), max_pulse);
static const uint32_t dropper_mid_pulse = (dropper_min_pulse + dropper_max_pulse) / 2;

/* Grabber only uses only part of the servo's range, from 37% to 66% of servo's range */
static const uint32_t grabber_min_pulse = DT_PROP(DT_NODELABEL(grabber_servo), min_pulse);
static const uint32_t grabber_max_pulse = DT_PROP(DT_NODELABEL(grabber_servo), max_pulse);
static const uint32_t grabber_range_min_pulse = 1150000; 
static const uint32_t grabber_range_max_pulse = 1460000;

static const uint32_t shooter_min_pulse = DT_PROP(DT_NODELABEL(shooter_servo), min_pulse);
static const uint32_t shooter_max_pulse = DT_PROP(DT_NODELABEL(shooter_servo), max_pulse);
static const uint32_t shooter_mid_pulse = (shooter_min_pulse + shooter_max_pulse) / 2;

/*
 * min pulse = min angle = left hole
 * max pulse = max angle = right hole
 */

void setup_servos()
{
	/* Set all of the servos to go to the middle (neutral) position */
        if (!device_is_ready(dropper_servo.dev))
            LOG_DBG("Dropper device is not ready");
        if (!device_is_ready(grabber_servo.dev))
            LOG_DBG("Grabber device is not ready");
        if (!device_is_ready(shooter_servo.dev))
            LOG_DBG("Shooter device is not ready");

        int ret;
	ret = pwm_set_pulse_dt(&dropper_servo, dropper_mid_pulse);
	if (ret < 0)
            LOG_DBG("Failed to set pwm pulse for dropper servo, error code %i", ret);
        ret = pwm_set_pulse_dt(&grabber_servo, grabber_range_min_pulse);
	if (ret < 0)
            LOG_DBG("Failed to set pwm pulse for grabber servo, error code %i", ret);
        ret = pwm_set_pulse_dt(&shooter_servo, shooter_mid_pulse);
        if (ret < 0)
            LOG_DBG("Failed to set pwm pulse for shooter servo, error code %i", ret);
}

void drop(int idx, int value)
{
        LOG_DBG("Drop for %i %i", idx, value);
	// If requested, set servo to neutral position
	if (value == 0)
	{
		pwm_set_pulse_dt(&dropper_servo, dropper_mid_pulse);
	}
	else if (value == 1)
	{
		// Set servo to drop the 0th (right) ball
		if (idx == 0)
		{
			pwm_set_pulse_dt(&dropper_servo, dropper_max_pulse);
		}
		// Set servo to drop the 1st (left) ball
		else if (idx == 1)
		{
			pwm_set_pulse_dt(&dropper_servo, dropper_min_pulse);
		}
	}
}

void grab(float value)
{
	/* 
	 * value = the fraction of closed you grabber to be (0 to 1).
	 * Eg. value = 0.6 = 60% closed grabber.
	 */
        LOG_DBG("Opened grab for %f", value);
	uint32_t pulse_width = (uint32_t) (grabber_range_min_pulse + 
		value * (grabber_range_max_pulse - grabber_range_min_pulse));
	int ret = pwm_set_pulse_dt(&grabber_servo, pulse_width);
        if (ret < 0)
            LOG_DBG("Failed to set pulse width for grabber, error code %i", ret);
}

void shoot(int idx, int value)
{
        LOG_DBG("Opened shoot for %i %i", idx, value);
        int ret;
	// If requested, set servo to neutral position (closes both holes)
	if (value == 0)
	{
		ret = pwm_set_pulse_dt(&shooter_servo, shooter_mid_pulse);
	}
	else if (value == 1)
	{
		// Set servo to open the 0th (right) hole
		if (idx == 0)
		{
			ret = pwm_set_pulse_dt(&shooter_servo, shooter_max_pulse);
		}
		// Set servo to open the 1st (left) hole
		else if (idx == 1)
		{
			ret = pwm_set_pulse_dt(&shooter_servo, shooter_min_pulse);
		}
	}
        if (ret < 0)
            LOG_DBG("Failed to shooter pwm pulse, error code %i", ret);
}
