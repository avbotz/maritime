/* 
 * Reads sensor data and readjusts power to motors on each loop.
 * Receives setpoint destinations from pc/jetson and can send 
 * back state (location) info to the pc/jetson when requested.
 */
#include <zephyr.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys_clock.h>
#include <drivers/gpio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "ahrs.h"
#include "dvl.h"
#include "thruster.h"
#include "pressure_sensor.h"
#include "util.h"
#include "killswitch.h"

#include "mec/control.h"
#include "mec/estimation.h"
#include "mec/util.h"


/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 4096
#define PAUSE_TIME 5000.0

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static const struct device *const dvl = get_DVL_device();
static const struct device *const mag = get_rm3100_device();
static const struct device *const imu = get_icm20689_device();
static const struct gpio_dt_spec *const ks = get_KILLSWITCH_device();


/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_dev)) {
        return;
    }

    while (uart_irq_rx_ready(uart_dev)) {

        uart_fifo_read(uart_dev, &c, 1);

        if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
            /* terminate string */
            rx_buf[rx_buf_pos] = '\0';

            /* if queue is full, message is silently dropped */
            k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

            /* reset the buffer (it was copied to the msgq) */
            rx_buf_pos = 0;
        } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
            rx_buf[rx_buf_pos++] = c;
        }
        /* else: characters beyond buffer size are dropped */
    }
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
    int msg_len = strlen(buf);

    for (int i = 0; i < msg_len; i++) {
        uart_poll_out(uart_dev, buf[i]);
    }
}

bool alive()
{
    // Todo: read the kill switch pin to return if we are alive or not
    uint8_t current_state = gpio_pin_get_dt(ks);
    // SUB is ALIVE when state is 1
    return current_state == 1;
}

void main(void)
{
    // Initialize a bunch of variables
    bool alive_state = alive();
    bool alive_state_prev = alive_state;
    bool pause = false;
    uint32_t pause_time;

    uint32_t motor_time = k_uptime_get_32();
    bool velocity_override = false;
    bool angvel_override = false;
    float power = 0;
    float INITIAL_YAW;

    Matrix<float, 6, 8> mix(nemo_mix_data);
    struct att_controller attitude_controller;
    struct angvel_controller angular_velocity_controller;
    struct position_controller pos_controller;
    struct velocity_controller vel_controller;
    struct mec_torque_setpoint torque_out;
    struct mec_force_setpoint force_out;
    struct mec_vehicle_position position;
    struct mec_vehicle_position position_sp;
    struct mec_vehicle_velocity_body velocity_body;
    struct mec_vehicle_velocity_body velocity_body_sp;
    struct mec_vehicle_velocity velocity_sp;
    struct mec_vehicle_attitude attitude;
    struct mec_vehicle_attitude att_sp;
    struct mec_vehicle_angvel angvel;
    struct mec_vehicle_angvel angvel_sp;

    att_controller_init(&attitude_controller);
    angvel_controller_init(&angular_velocity_controller);
    position_controller_init(&pos_controller);
    velocity_controller_init(&vel_controller);
    mec_vehicle_position_init(&position);

    position_sp.north = 0;
    position_sp.east = 0;
    position_sp.down = 0;
    position_controller_update_sp(&pos_controller, &position_sp);

    att_sp.yaw = 0;
    att_sp.pitch = 0;
    att_sp.roll = 0;
    att_controller_update_sp(&attitude_controller, &att_sp);

    pos_controller.use_floor_altitude = false;

    char msg[MSG_SIZE];
    char delim[] = " ";

    if (!device_is_ready(uart_dev)) {
        printk("UART device not found!");
        return;
    }

    /* configure interrupt and callback to receive data */
    uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    struct ahrs_sample ahrs_sample;
    struct imu_sample imu_sample;
    struct mag_sample mag_sample;

    // TODO: here, set INITIAL_YAW = the initial ahrs yaw we sample

    // Begin motor control loop
    while (true)
    {
        // TODO: finish dvl interfacing and sample dvl here
        int dvl_ret = sample_dvl(dev, &dvl_sample);
        if (dvl_ret != 0) {
            LOG_ERR("Error sampling DVL: %d", dvl_ret);
        }
        // TODO: update velocity_body with that fresh dvl data
        /*
        velocity_body.forward_m_s = newest_dvl_forward_m_s
        velocity_body.right_m_s = newest_dvl_right_m_s
        velocity_body.down_m_s = newest_dvl_down_m_s
        */

        // Parse message coming from computer if there is one
        if (k_msgq_get(&uart_msgq, &msg, K_NO_WAIT) == 0)
        {
            // Get the first character 
            // (we have a code of what characters mean)
            char *ptr = strtok(msg, delim);
            char c = ptr[0];

            char output[MSG_SIZE];
            
            // Todo: Change this protocol to be more readable
            if (c == 'a')
            {
                snprintf(output, sizeof(output), "%d\n", alive());
                print_uart(output);
            }
            else if (c == 'b')
            {
                // Set altitude setpoint (which overrides depth) if it's valid
                velocity_override = false;
                float value = parse_float(ptr, delim);
                if (value == -1)
                    pos_controller.use_floor_altitude = false;
                else
                {
                    pos_controller.use_floor_altitude = true;
                    position_sp.altitude = value;
                }
                position_controller_update_sp(&pos_controller, &position_sp);
            }
            else if (c == 'c')
            {
                snprintf(output, sizeof(output),
                    "%f %f %f %f %f %f\n",
                    position.north,
                    position.east,
                    position.down,
                    rad_to_deg(attitude.yaw),
                    rad_to_deg(attitude.pitch),
                    rad_to_deg(attitude.roll));
                print_uart(output);
            }
            else if (c == 'm')
            {
                snprintf(output, sizeof(output),
                    "%f %f %f\n",
                    velocity_body.forward_m_s,
                    velocity_body.right_m_s,
                    velocity_body.down_m_s);
                print_uart(output);
            }
            else if (c == 'p')
            {
                // Ranges from 0-1, 1 = 100% thruster power
                power = parse_float(ptr, delim);
            }
            else if (c == 's')
            {
                // Only save z setpoint if it's valid
                velocity_override = false;
                angvel_override = false;

                position_sp.north = parse_float(ptr, delim);
                position_sp.east = parse_float(ptr, delim);

                // Only save z setpoint if it's valid
                float value = parse_float(ptr, delim);
                if (value != -1)
                    position_sp.down = value;

                att_sp.yaw = deg_to_rad(parse_float(ptr, delim));

                // For now, keep pitch and roll at 0
                // att_sp.pitch = deg_to_rad(parse_float(ptr, delim));
                // att_sp.roll = deg_to_rad(parse_float(ptr, delim));

                position_controller_update_sp(&pos_controller, &position_sp);
                att_controller_update_sp(&attitude_controller, &att_sp);
            }
            else if (c == 'v')
            {
                velocity_override = true;

                velocity_body_sp.forward_m_s = parse_float(ptr, delim);
                velocity_body_sp.right_m_s = parse_float(ptr, delim);
                velocity_body_sp.down_m_s = parse_float(ptr, delim);

                velocity_controller_update_sp(&vel_controller, &velocity_body_sp);
            }
            else if (c == 'n')
            {
                // Only write the angle setpoint
                angvel_override = false;

                att_sp.yaw = deg_to_rad(parse_float(ptr, delim));
                att_sp.pitch = deg_to_rad(parse_float(ptr, delim));
                att_sp.roll = deg_to_rad(parse_float(ptr, delim));

                att_controller_update_sp(&attitude_controller, &att_sp);
            }
            else if (c == 't')
            {
                angvel_override = true;

                angvel_sp.yaw_rad_s = deg_to_rad(parse_float(ptr, delim));
                angvel_sp.pitch_rad_s = deg_to_rad(parse_float(ptr, delim));
                angvel_sp.roll_rad_s = deg_to_rad(parse_float(ptr, delim));

                angvel_controller_update_sp(&angular_velocity_controller, &angvel_sp);
            }
            else if (c == 'z')
            {
                velocity_override = false;
                position_sp.down = parse_float(ptr, delim);
                position_controller_update_sp(&pos_controller, &position_sp);
            }
            else if (c == 'w')
            {
                // Send current altitude
                snprintf(output, sizeof(output), "%f\n", position.altitude);
                print_uart(output);
            }
            else if (c == 'r')
            {
                velocity_override = false;
                angvel_override = false;
                position_sp.north = position.north;
                position_sp.east = position.east;
                position_sp.down = position.down;
                att_sp.yaw = attitude.yaw;
                att_sp.pitch = attitude.pitch;
                att_sp.roll = attitude.roll;

                float offsets[3];
                // TODO: make BODY_DOF = 3 (readable constant)
                for (int i = 0; i < 3; i++)
                    offsets[i] = parse_float(ptr, delim);
                float angle[3] = {
                    deg_to_rad(attitude.yaw), 
                    deg_to_rad(attitude.pitch),
                    deg_to_rad(attitude.roll)
                };
                float absolute_offsets[3];
                offsets_to_frame(offsets, angle, absolute_offsets);

                position_sp.north += absolute_offsets[0];
                position_sp.east += absolute_offsets[1];
                position_sp.down += absolute_offsets[2];

                att_sp.yaw = angle_add(attitude.yaw, this->Serial.parseFloat() * D2R);
                att_sp.pitch = angle_add(attitude.pitch, this->Serial.parseFloat() * D2R);
                att_sp.roll = angle_add(attitude.roll, this->Serial.parseFloat() * D2R);

                position_controller_update_sp(&pos_controller, &position_sp);
                att_controller_update_sp(&attitude_controller, &att_sp);
            }
            else if (c == 'x')
            {
                position.north = 0;
                position.east = 0;

                INITIAL_YAW = attitude.yaw;
                attitude.yaw = 0;

                position_sp.north = 0;
                position_sp.east = 0;
                att_sp.yaw = 0;

                position_controller_update_sp(&pos_controller, &position_sp);
                att_controller_update_sp(&attitude_controller, &att_sp);
            }
            else if (c == 'g')
            {
                int idx = parse_int(ptr, delim);
                int val = parse_int(ptr, delim);
                // TODO: interface with ball dropper
                // this->drop(idx, val);
            }
            else if (c == 'u')
            {
                // Expects message of 
                // u <int representing degree of freedom> <0 for position, 1 for velocity> <gain1> <gain2> <gain3>
                int dof = parse_int(ptr, delim);
                int position_or_vel = parse_int(ptr, delim);
                float gains[3];
                for (int i = 0; i < 3; i++)
                    gains[i] = parse_float(ptr, delim);

                // Adjust the x, y, or z gain
                if (dof <= 2)
                {
                    if (position_or_vel == 0)
                        pid_set_gains(&pos_controller.pid[dof], gains[0], gains[1], gains[2]);
                    else if (position_or_vel == 1)
                        pid_set_gains(&vel_controller.pid[dof], gains[0], gains[1], gains[2]);
                }

                // Adjust the roll, pitch, or yaw gain
                else if (dof >= 3)
                {
                    // Start the dof at 0 for indexing the pids
                    dof -= 3; 
                    if (position_or_vel == 0)
                        pid_set_gains(&attitude_controller.pid[dof], gains[0], gains[1], gains[2]);
                    else if (position_or_vel == 1)
                        pid_set_gains(&angular_velocity_controller.pid[dof], gains[0], gains[1], gains[2]);
                }
            }
            else if (c == 'f')
            {
                int val = parse_int(ptr, delim);
                // TODO: interface with grabber
                // this->grab(val);
            }
            else if (c == 'o')
            {
                int idx = parse_int(ptr, delim);
                int val = parse_int(ptr, delim);
                // TODO: interface with torp shooter
                // this->shoot(idx, val);
            }
        }

        alive_state_prev = alive_state;
        alive_state = alive();

        // Enough time has elapsed for motors to start up. Don't forget to reset
        // time so the time difference for the first set of velocities from the
        // DVL are correct.
        if (pause && k_uptime_get_32() - pause_time > PAUSE_TIME)
        {
            pause = false;
            motor_time = k_uptime_get_32();
        }

        // Kill switch has just been switched from alive to dead. Pause motor
        // communications. Until the sub has been set back to alive, none of
        // these "if" statements will run.
        if (alive_state_prev && !alive_state)
        {
            // TODO: send thrust values of 0 here in case 
            // the power doesn't shut off?
        }

        // Kill switch has just been switched from dead to alive. Reset all
        // values, including states and initial headings. Pause so motors have
        // time to start up. 
        if (!alive_state_prev && alive_state)
        { 
            position.north = 0.;
            position.east = 0.;
            position_sp.north = 0.;
            position_sp.east = 0.;
            position_sp.down = 0.;

            attitude.yaw = 0.;
            att_sp.yaw = 0.;

            // TODO: set INITIAL_YAW = current ahrs yaw reading
            // INITIAL_YAW = 

            pos_controller.use_floor_altitude = false;

            pause = true;
            pause_time = k_uptime_get_32();

            position_controller_update_sp(&pos_controller, &position_sp);
            att_controller_update_sp(&attitude_controller, &att_sp);
        }

        // Motors are done starting up and the sub is alive. Run the sub as
        // intended.
        if (!pause && alive_state)
        {
            // TODO: interface with the pressure sensor, ahrs, dvl to 
            // get the latest sensor readings
            /*
            position.down = get_depth_from_pressure_sensor()

            //Sample IMU on AHRS
            int imu_ret = sample_imu(dev, &ahrs_sample);
            if (imu_ret != 0) {
                LOG_ERR("Error sampling IMU: %d", imu_ret);

            // TODO: do (current yaw/pitch/roll - previous yaw/pitch/roll) / dt
            // inside of ahrs interfacing implementation to get the angular
            // velocity
            angvel.roll_rad_s = ahrs_get_roll_rad_s
            angvel.pitch_rad_s = ahrs_get_pitch_rad_s
            angvel.yaw_rad_s = ahrs_get_yaw_rad_s

            attitude.yaw = ahrs_get_yaw() - INITIAL_YAW
            attitude.pitch = ahrs_get_pitch()
            attitude.roll = ahrs_get_roll()
            position.altitude = dvl_get_range_to_bottom()
            */
          
            // Handle angle overflow/underflow.
            attitude.yaw += (attitude.yaw > M_PI) ? -2*M_PI : (attitude.yaw < -M_PI) ? 2*M_PI : 0;
            attitude.pitch += (attitude.pitch > M_PI) ? -2*M_PI : (attitude.pitch < -M_PI) ? 2*M_PI : 0;
            attitude.roll += (attitude.roll > M_PI) ? -2*M_PI : (attitude.roll < -M_PI) ? 2*M_PI : 0;

            // Update our position based on velocity and time elapsed
            uint32_t t = k_uptime_get_32();
            float dt = (t - this->motor_time) / 1000.;
            motor_time = t;

            mec_vehicle_position_update(&velocity_body,
                position.altitude, &position, &attitude, dt);

            // Compute thruster outputs from error in current state to state setpoint
            if (!angvel_override)
            {
                att_controller_update(
                    &attitude_controller,
                    &attitude,
                    &angvel_sp,
                    dt
                    );
                angvel_controller_update_sp(
                    &angular_velocity_controller,
                    &angvel_sp
                    );
            }
            angvel_controller_update(
                &angular_velocity_controller,
                &angvel,
                &torque_out,
                dt
                );

            if (!velocity_override)
            {
                position_controller_update(
                    &pos_controller,
                    &position,
                    &attitude,
                    &velocity_body_sp,
                    dt
                    );
                velocity_controller_update_sp(
                    &vel_controller,
                    &velocity_body_sp
                    );
            }
            velocity_controller_update(
                &vel_controller,
                &velocity_body,
                &force_out,
                dt
                );

            // Map forces and torques to thruster outputs
            float thruster_outputs[8];
            mec_mix(&force_out, &torque_out, mix, power, thruster_outputs);

            // TODO: interface with thrusters to send them the thruster_outputs
            // send_thrusts(thruster_outputs) // for example
        }
    }
}
