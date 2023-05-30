/* Servos control the ball dropper, torpedo shooter, grabber */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

/* Pulse width definitions for servos */
#define SERVO_MINPULSEWIDTH 850
#define SERVO_MIDDLEPULSEWIDTH 1600
#define SERVO_MAXPULSEWIDTH 2350

static const struct pwm_dt_spec dropper_servo = 
	PWM_DT_SPEC_GET(DT_NODELABEL(pwm1));
static const struct pwm_dt_spec grabber_servo = 
	PWM_DT_SPEC_GET(DT_NODELABEL(pwm2));
static const struct pwm_dt_spec shooter_servo = 
	PWM_DT_SPEC_GET(DT_NODELABEL(pwm3));

// Convention: min pulse = min angle = left hole, max pulse = max angle = right hole

void init_servos()
{
	// Set servo devices to neutral positions
	pwm_set_pulse_dt(dropper_servo, SERVO_MIDDLEPULSEWIDTH);
	pwm_set_pulse_dt(grabber_servo, SERVO_MIDDLEPULSEWIDTH);
	pwm_set_pulse_dt(shooter_servo, SERVO_MIDDLEPULSEWIDTH);
}

void drop(int idx, int value)
{
	// If requested, set servo to neutral position
	if (value == 0)
	{
		pwm_set_pulse_dt(dropper_servo, SERVO_MIDDLEPULSEWIDTH);
	}
	else if (value == 1)
	{
		// Set servo to drop the 0th (right) ball
		if (idx == 0)
		{
			pwm_set_pulse_dt(dropper_servo, SERVO_MAXPULSEWIDTH);
		}
		// Set servo to drop the 1st (left) ball
		else if (idx == 1)
		{
			pwm_set_pulse_dt(dropper_servo, SERVO_MINPULSEWIDTH);
		}
	}
}

void grab(int value)
{
	// If requested, open up the grabber
	if (value == 0)
	{
		pwm_set_pulse_dt(grabber_servo, SERVO_MINPULSEWIDTH);
	}

	// If requested, close the grabber (grab)
	else if (value == 1)
	{
		pwm_set_pulse_dt(grabber_servo, SERVO_MAXPULSEWIDTH);
	}
}

void shoot(int idx, int value)
{
	// If requested, set servo to neutral position (closes both holes)
	if (value == 0)
	{
		pwm_set_pulse_dt(shooter_servo, SERVO_MIDDLEPULSEWIDTH);
	}
	else if (value == 1)
	{
		// Set servo to open the 0th (right) hole
		if (idx == 0)
		{
			pwm_set_pulse_dt(shooter_servo, SERVO_MAXPULSEWIDTH);
		}
		// Set servo to open the 1st (left) hole
		else if (idx == 1)
		{
			pwm_set_pulse_dt(shooter_servo, SERVO_MINPULSEWIDTH);
		}
	}
}