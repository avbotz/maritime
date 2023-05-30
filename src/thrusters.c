/* Send pwm signals to the escs that control the thrusters */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

/* Pulse width definitions for bluerobotics t200 thrusters */
#define THRUSTER_MINPULSEWIDTH 1100
#define THRUSTER_MIDDLEPULSEWIDTH 1500
#define THRUSTER_MAXPULSEWIDTH 1900

static const struct pwm_dt_spec thruster_escs[NUM_THRUSTERS] = {
	PWM_DT_SPEC_GET(DT_NODELABEL(pwm4)),
	PWM_DT_SPEC_GET(DT_NODELABEL(pwm5)),
	PWM_DT_SPEC_GET(DT_NODELABEL(pwm6)),
	PWM_DT_SPEC_GET(DT_NODELABEL(pwm7)),
	PWM_DT_SPEC_GET(DT_NODELABEL(pwm8)),
	PWM_DT_SPEC_GET(DT_NODELABEL(pwm9)),
	PWM_DT_SPEC_GET(DT_NODELABEL(pwm10)),
	PWM_DT_SPEC_GET(DT_NODELABEL(pwm11))
};

void init_thrusters()
{
	/* 
	 * Set the thrusters to not move. 
	 * Pulse lengths: 1100 = max reverse, 1500 = nothing, 1900 = max forward
	 */
	for (int i = 0; i < NUM_THRUSTERS; i++)
		pwm_set_pulse_dt(thruster_escs[i], THRUSTER_MIDDLEPULSEWIDTH);
}

void send_thrusts(float *thrusts)
{
	/* 
	 * Input: Array of NUM_THRUSTERS floats. Each float is -1 to 1,
	 *        where -1 means max reverse, and 1 means max forward.
	 * Sends thrusts to each thruster by sending a pwm signal to
	 * each thruster's esc. 
	 */
	float pwm_radius = THRUSTER_MAXPULSEWIDTH - THRUSTER_MIDDLEPULSEWIDTH;
	for (int i = 0; i < NUM_THRUSTERS; i++)
	{
		float pwm_value = (uint32_t) (THRUSTER_MIDDLEPULSEWIDTH +
			thrusts[i] * pwm_radius);
		pwm_set_pulse_dt(thruster_escs[i], pwm_value);
	}
}