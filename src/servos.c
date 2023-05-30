/* Servos control the ball dropper, torpedo shooter, grabber */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

#define MINPULSEWIDTH 850
#define MIDDLEPULSEWIDTH 1600
#define MAXPULSEWIDTH 2350

static const struct pwm_dt_spec dropper_servo = 
	PWM_DT_SPEC_GET(DT_NODELABEL(servo1));
static const struct pwm_dt_spec grabber_servo = 
	PWM_DT_SPEC_GET(DT_NODELABEL(servo2));
static const struct pwm_dt_spec shooter_servo = 
	PWM_DT_SPEC_GET(DT_NODELABEL(servo3));

/*
 * min pulse = min angle = left hole
 * max pulse = max angle = right hole
 */

void init_servos()
{
	/* Set all of the servos to go to the middle (neutral) position */
	pwm_set_pulse_dt(&dropper_servo, MIDDLEPULSEWIDTH);
	pwm_set_pulse_dt(&grabber_servo, MIDDLEPULSEWIDTH);
	pwm_set_pulse_dt(&shooter_servo, MIDDLEPULSEWIDTH);
}

void drop(int idx, int value)
{
	// If requested, set servo to neutral position
	if (value == 0)
	{
		pwm_set_pulse_dt(&dropper_servo, MIDDLEPULSEWIDTH);
	}
	else if (value == 1)
	{
		// Set servo to drop the 0th (right) ball
		if (idx == 0)
		{
			pwm_set_pulse_dt(&dropper_servo, MAXPULSEWIDTH);
		}
		// Set servo to drop the 1st (left) ball
		else if (idx == 1)
		{
			pwm_set_pulse_dt(&dropper_servo, MINPULSEWIDTH);
		}
	}
}

void grab(int value)
{
	// If requested, open up the grabber
	if (value == 0)
	{
		pwm_set_pulse_dt(&grabber_servo, MINPULSEWIDTH);
	}

	// If requested, close the grabber (grab)
	else if (value == 1)
	{
		pwm_set_pulse_dt(&grabber_servo, MAXPULSEWIDTH);
	}
}

void shoot(int idx, int value)
{
	// If requested, set servo to neutral position (closes both holes)
	if (value == 0)
	{
		pwm_set_pulse_dt(&shooter_servo, MIDDLEPULSEWIDTH);
	}
	else if (value == 1)
	{
		// Set servo to open the 0th (right) hole
		if (idx == 0)
		{
			pwm_set_pulse_dt(&shooter_servo, MAXPULSEWIDTH);
		}
		// Set servo to open the 1st (left) hole
		else if (idx == 1)
		{
			pwm_set_pulse_dt(&shooter_servo, MINPULSEWIDTH);
		}
	}
}