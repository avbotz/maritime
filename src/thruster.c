/* Servos control the ball dropper, torpedo shooter, grabber */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

#include "thruster.h"

LOG_MODULE_REGISTER(thrusters, LOG_LEVEL_DBG);

static const struct pwm_dt_spec thrusters[] = {
    PWM_DT_SPEC_GET(DT_NODELABEL(thr0_esc)),
    PWM_DT_SPEC_GET(DT_NODELABEL(thr1_esc)),
    PWM_DT_SPEC_GET(DT_NODELABEL(thr2_esc)),
    PWM_DT_SPEC_GET(DT_NODELABEL(thr3_esc)),
    PWM_DT_SPEC_GET(DT_NODELABEL(thr4_esc)),
    PWM_DT_SPEC_GET(DT_NODELABEL(thr5_esc)),
    PWM_DT_SPEC_GET(DT_NODELABEL(thr6_esc)),
    PWM_DT_SPEC_GET(DT_NODELABEL(thr7_esc))
};

static const float esc_range_min_pulse_ns = 1100000;
static const float esc_range_mid_pulse_ns = 1500000;
static const float esc_range_max_pulse_ns = 1900000;

static const int NUM_THRUSTERS = 8;
/*
 * min pulse = hela reverse
 * max pulse = hela forward
 */

int setup_thrusters(void)
{
    /* Set all of the thrusters to go to the middle (no speed/thrust) */
    for (int i = 0; i < NUM_THRUSTERS; i++)
    {
        if (!device_is_ready(thrusters[i].dev))
            LOG_DBG("Thruster %i's electronic speed controller device is not ready", i);
    }

    int ret;
    for (int i = 0; i < NUM_THRUSTERS; i++)
    {
        ret = pwm_set_pulse_dt(&thrusters[i], esc_range_mid_pulse_ns);
        if (ret < 0)
            LOG_DBG("Failed to set pwm pulse for thruster %i's esc, error code %i", i, ret);
    }

    return 0;
}

void send_thrusts(float thrusts[8])
{
    /* Convert thrust value from [-1, 1] to [1100000, 1900000] */
    float pulse_widths[8];
    for (int i = 0; i < NUM_THRUSTERS; i++)
    {
        pulse_widths[i] = esc_range_mid_pulse_ns + 
            thrusts[i] * (esc_range_max_pulse_ns - esc_range_min_pulse_ns);

        // Check for nan
        if (pulse_widths[i] != pulse_widths[i])
        {
            LOG_DBG("Nan pulse width, discarding thrust packet");
            return;
        }

        int ret = pwm_set_pulse_dt(&thrusters[i], pulse_widths[i]);
        if (ret < 0)
        {
            LOG_DBG("Failed to send pwm pulse %f nanoseconds for thruster %i", pulse_widths[i], i);
        }
    }

    LOG_DBG("Successfully updated pwm pulses for thrusters: %f, %f, %f, %f, %f, %f, %f, %f,",
        pulse_widths[0], pulse_widths[1], pulse_widths[2], pulse_widths[3],
        pulse_widths[4], pulse_widths[5], pulse_widths[6], pulse_widths[7]
    );
}