/* 
 * Reads sensor data and readjusts power to motors on each loop.
 * Receives setpoint destinations from pc/jetson and can send 
 * back state (location) info to the pc/jetson when requested.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys_clock.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "dvl.h"
#include "ahrs.h"
#include "pressure.h"
#include "thruster.h"
#include "servo.h"
#include "killswitch.h"
#include "util.h"

#include "mec/control.h"
#include "mec/estimation.h"
#include "mec/pid_controller.h"
#include "mec/util.h"

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 256
#define PAUSE_TIME 5000.0

LOG_MODULE_REGISTER(LOG_LEVEL_INF);

/* queue to store up to 3 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 3, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue. We use this to read messages coming in 
 * from the onboard computer.
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

int main(void)
{
    // Initialize a bunch of variables
    struct dvl_data_s dvl_data;
    struct ahrs_data_s ahrs_data;
    struct pressure_data_s pressure_data;

    uint32_t motor_time = time_us();
    bool velocity_override = false;
    bool angvel_override = false;
    float power = 0;
    float INITIAL_YAW;

    float mix[8][6];
    memcpy(mix, sub_mix_data, sizeof(float) * NUM_THRUSTERS * NUM_DOF);
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

    vel_controller.pid[0].integral = 0;
    vel_controller.pid[1].integral = 0;
    vel_controller.pid[2].integral = 0;

    angular_velocity_controller.pid[0].integral = 0;
    angular_velocity_controller.pid[1].integral = 0;
    angular_velocity_controller.pid[2].integral = 0;

    position_sp.north = 0;
    position_sp.east = 0;
    position_sp.down = 0;
    position_controller_update_sp(&pos_controller, &position_sp);

    attitude.yaw = 0;
    attitude.pitch = 0;
    attitude.roll = 0;
    att_sp.yaw = 0;
    att_sp.pitch = 0;
    att_sp.roll = 0;
    att_controller_update_sp(&attitude_controller, &att_sp);

    pos_controller.use_floor_altitude = false;

    char msg[MSG_SIZE];
    char delim[] = " ";

    uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    setup_thrusters();
    setup_dvl();
    setup_ahrs();
    setup_pressure();
    setup_servos();
    setup_killswitch();

    bool alive_state = alive();
    bool alive_state_prev = alive_state;
    bool pause = false;
    uint32_t pause_time;

    bool dvl_updated = false;
    bool ahrs_updated = false;
    bool pressure_updated = false;

    while (true) 
    {
        int ret;
        ret = k_msgq_get(&ahrs_data_msgq, &ahrs_data, K_NO_WAIT);
        ret = k_msgq_get(&dvl_data_msgq, &dvl_data, K_NO_WAIT);
        ret = k_msgq_get(&pressure_data_msgq, &pressure_data, K_NO_WAIT);

        // Parse message coming from computer if there is one
        if (k_msgq_get(&uart_msgq, &msg, K_NO_WAIT) == 0)
        {
            // Get the first character 
            // (we have a code of what characters mean)
            char *save_ptr;
            char *token = strtok_r(msg, delim, &save_ptr);
            char c = token[0];
            
            // Todo: Change this protocol to be more readable
            if (c == 'a')
            {
                bool is_alive = alive();
                printk("%d\n", is_alive);
            }
            else if (c == 'b')
            {
                // Set altitude setpoint (which overrides depth) if it's valid
                velocity_override = false;
                float value = parse_float(delim, &save_ptr);
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
                printk("%f %f %f %f %f %f\n",
                    position.north,
                    position.east,
                    position.down,
                    rad_to_deg(attitude.yaw),
                    rad_to_deg(attitude.pitch),
                    rad_to_deg(attitude.roll));
            }
            else if (c == 'm')
            {
                printk("%f %f %f\n",
                    velocity_body.forward_m_s,
                    velocity_body.right_m_s,
                    velocity_body.down_m_s);
            }
            else if (c == 'p')
            {
                // Ranges from 0-1, 1 = 100% thruster power
                power = parse_float(delim, &save_ptr);
            }
            else if (c == 's')
            {
                // Only save z setpoint if it's valid
                velocity_override = false;
                angvel_override = false;

                position_sp.north = parse_float(delim, &save_ptr);
                position_sp.east = parse_float(delim, &save_ptr);

                // Only save z setpoint if it's valid
                float value = parse_float(delim, &save_ptr);
                if (value != -1)
                    position_sp.down = value;

                att_sp.yaw = deg_to_rad(parse_float(delim, &save_ptr));

                // For now, keep pitch and roll at 0
                // att_sp.pitch = deg_to_rad(parse_float(delim, &save_ptr));
                // att_sp.roll = deg_to_rad(parse_float(delim, &save_ptr));

                position_controller_update_sp(&pos_controller, &position_sp);
                att_controller_update_sp(&attitude_controller, &att_sp);
            }
            else if (c == 'v')
            {
                velocity_override = true;

                velocity_body_sp.forward_m_s = parse_float(delim, &save_ptr);
                velocity_body_sp.right_m_s = parse_float(delim, &save_ptr);
                velocity_body_sp.down_m_s = parse_float(delim, &save_ptr);

                velocity_controller_update_sp(&vel_controller, &velocity_body_sp);
            }
            else if (c == 'n')
            {
                // Only write the angle setpoint
                angvel_override = false;

                att_sp.yaw = deg_to_rad(parse_float(delim, &save_ptr));
                att_sp.pitch = deg_to_rad(parse_float(delim, &save_ptr));
                att_sp.roll = deg_to_rad(parse_float(delim, &save_ptr));

                att_controller_update_sp(&attitude_controller, &att_sp);
            }
            else if (c == 't')
            {
                angvel_override = true;

                angvel_sp.yaw_rad_s = deg_to_rad(parse_float(delim, &save_ptr));
                angvel_sp.pitch_rad_s = deg_to_rad(parse_float(delim, &save_ptr));
                angvel_sp.roll_rad_s = deg_to_rad(parse_float(delim, &save_ptr));

                angvel_controller_update_sp(&angular_velocity_controller, &angvel_sp);
            }
            else if (c == 'z')
            {
                velocity_override = false;
                position_sp.down = parse_float(delim, &save_ptr);
                position_controller_update_sp(&pos_controller, &position_sp);
            }
            else if (c == 'w')
            {
                // Send current altitude
                printk("%f\n", position.altitude);
            }
            else if (c == 'r')
            {
                velocity_override = false;
                angvel_override = false;
                position_sp.north = position.north;
                position_sp.east = position.east;
                position_sp.down = position.down;
                att_sp.yaw = attitude.yaw;
                // att_sp.pitch = attitude.pitch;
                // att_sp.roll = attitude.roll;

                float offsets[3];
                // TODO: make BODY_DOF = 3 (readable constant)
                for (int i = 0; i < 3; i++)
                    offsets[i] = parse_float(delim, &save_ptr);
                float angle[3] = {
                    attitude.roll, 
                    attitude.pitch,
                    attitude.yaw
                };
                float absolute_offsets[3];
                offsets_to_frame(offsets, angle, absolute_offsets);

                position_sp.north += absolute_offsets[0];
                position_sp.east += absolute_offsets[1];
                position_sp.down += absolute_offsets[2];

                att_sp.yaw = angle_add(attitude.yaw, 
                    deg_to_rad(parse_float(delim, &save_ptr)));

                /*
                att_sp.pitch = angle_add(attitude.pitch, 
                    deg_to_rad(parse_float(delim, &save_ptr)));
                att_sp.roll = angle_add(attitude.roll, 
                    deg_to_rad(parse_float(delim, &save_ptr)));
                */

                position_controller_update_sp(&pos_controller, &position_sp);
                att_controller_update_sp(&attitude_controller, &att_sp);
            }
            else if (c == 'h')
            {
                printk("%f %f %f\n", 
                rad_to_deg(ahrs_data.yaw),
                rad_to_deg(ahrs_data.pitch),
                rad_to_deg(ahrs_data.roll));
            }
            else if (c == 'd')
            {
                // Todo: implement ahrs sampling
                printk("%f %f %f %f\n", 
                    dvl_data.velocity_x, 
                    dvl_data.velocity_y, 
                    dvl_data.velocity_z, 
                    dvl_data.altitude);
                //printk("%f\n", rad_to_deg(ahrs_data.yaw));
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
                int idx = parse_int(delim, &save_ptr);
                int val = parse_int(delim, &save_ptr);
                drop(idx, val);
            }
            else if (c == 'u')
            {
                // Expects message of 
                // u <int representing degree of freedom> <0 for position, 1 for velocity> <gain1> <gain2> <gain3>
                int dof = parse_int(delim, &save_ptr);
                int position_or_vel = parse_int(delim, &save_ptr);
                float gains[3];
                for (int i = 0; i < 3; i++)
                    gains[i] = parse_float(delim, &save_ptr);

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
                float val = parse_float(delim, &save_ptr);
                grab(val);
            }
            else if (c == 'o')
            {
                int idx = parse_int(delim, &save_ptr);
                int val = parse_int(delim, &save_ptr);
                shoot(idx, val);
            }
            else if (c == '#')
            {
                printk("%f\n", pressure_data.depth);
            }
            // Debug characters to test the dvl switch
            else if (c == 'q')
            {
                start_dvl_uart();
            }
            else if (c == 'j')
            {
                stop_dvl_uart();
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
            motor_time = time_us();
        }

        // Kill switch has just been switched from alive to dead. Pause motor
        // communications. Until the sub has been set back to alive, none of
        // these "if" statements will run.
        if (alive_state_prev && !alive_state)
        {
            stop_dvl_uart();
            // TODO: send thrust values of 0 here in case 
            // the power doesn't shut off?
        }

        // Kill switch has just been switched from dead to alive. Reset all
        // values, including states and initial headings. Pause so motors have
        // time to start up. 
        if (!alive_state_prev && alive_state)
        { 
            start_dvl_uart();

            position.north = 0.;
            position.east = 0.;
            position_sp.north = 0.;
            position_sp.east = 0.;
            position_sp.down = 0.;

            attitude.yaw = 0.;
            att_sp.yaw = 0.;

            angvel_sp.yaw_rad_s = 0;
            angvel_sp.pitch_rad_s = 0;
            angvel_sp.roll_rad_s = 0;

            INITIAL_YAW = ahrs_data.yaw;

            pos_controller.use_floor_altitude = false;

            pause = true;
            pause_time = k_uptime_get_32();

            velocity_override = false;
            angvel_override = false;

            position_controller_update_sp(&pos_controller, &position_sp);
            att_controller_update_sp(&attitude_controller, &att_sp);
        }

        // Motors are done starting up and the sub is alive. Run the sub as
        // intended.
        if (!pause && alive_state)
        {
            position.down = pressure_data.depth;

            angvel.roll_rad_s = ahrs_data.ang_vel_roll;
            angvel.pitch_rad_s = ahrs_data.ang_vel_pitch;
            angvel.yaw_rad_s = ahrs_data.ang_vel_yaw;

            attitude.yaw = ahrs_data.yaw - INITIAL_YAW;
            attitude.pitch = ahrs_data.pitch;
            attitude.roll = ahrs_data.roll;
          
            // Handle angle overflow/underflow.
            attitude.yaw += (attitude.yaw > M_PI) ? -2*M_PI : (attitude.yaw < -M_PI) ? 2*M_PI : 0;
            attitude.pitch += (attitude.pitch > M_PI) ? -2*M_PI : (attitude.pitch < -M_PI) ? 2*M_PI : 0;
            attitude.roll += (attitude.roll > M_PI) ? -2*M_PI : (attitude.roll < -M_PI) ? 2*M_PI : 0;

            if (fabs(dvl_data.velocity_x < 5.))
                velocity_body.forward_m_s = dvl_data.velocity_x;
            else
                velocity_body.forward_m_s = 0.;
            if (fabs(dvl_data.velocity_y < 5.))
                velocity_body.right_m_s = dvl_data.velocity_y;
            else 
                velocity_body.right_m_s = 0.;
            if (fabs(dvl_data.velocity_z < 5.))
                velocity_body.down_m_s = dvl_data.velocity_z;
            else 
                velocity_body.down_m_s = 0.;
            position.altitude = dvl_data.altitude;


            // Update our position based on velocity and time elapsed
            uint32_t t = time_us();
            float dt = (t - motor_time) / 1e6f;
            motor_time = t;

            struct mec_vehicle_velocity rectified_velocity;
            velocity_body_to_ned(&velocity_body, &rectified_velocity, &attitude);

            float abs_dx = rectified_velocity.north_m_s * dt;
            float abs_dy = rectified_velocity.east_m_s * dt;
            if (fabs(abs_dx) < 0.5)
                position.north += abs_dx;
            if (fabs(abs_dy) < 0.5)
                position.east += abs_dy;
            //mec_vehicle_position_update(&velocity_body,
            //    position.altitude, &position, &attitude, dt);

            // Compute thruster outputs from error in current state to state setpoint
            /* Really jank: Seems like the functions don't work sometimes, so pasting their code here instead
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
            */

            // Calculate angvel setpoint
            if (!angvel_override)
            {
                struct mec_vehicle_attitude att_error;

                att_error.roll = angle_difference(att_sp.roll, attitude.roll);
                att_error.pitch = angle_difference(att_sp.pitch, attitude.pitch);
                att_error.yaw = angle_difference(att_sp.yaw, attitude.yaw);

                angvel_sp.roll_rad_s = normalize(
                    pid_calculate(
                        &attitude_controller.pid[0], 
                        att_error.roll, 
                        dt), 
                    -2, 2);
                angvel_sp.pitch_rad_s = normalize(
                    pid_calculate(
                        &attitude_controller.pid[1], 
                        att_error.pitch, 
                        dt), 
                    -2, 2);
                angvel_sp.yaw_rad_s = normalize(
                    pid_calculate(
                        &attitude_controller.pid[2], 
                        att_error.yaw, 
                        dt), 
                    -2, 2);
            }

            // Calculate torque necessary to get to that angvel setpoint
            struct mec_vehicle_angvel angvel_error;

            angvel_error.roll_rad_s = angvel_sp.roll_rad_s - angvel.roll_rad_s;
            angvel_error.pitch_rad_s = angvel_sp.pitch_rad_s - angvel.pitch_rad_s;
            angvel_error.yaw_rad_s = angvel_sp.yaw_rad_s - angvel.yaw_rad_s;

            torque_out.roll = normalize(
                pid_calculate(
                    &angular_velocity_controller.pid[0], 
                    angvel_error.roll_rad_s, 
                    dt), 
                -1, 1);
            torque_out.pitch = normalize(
                pid_calculate(
                    &angular_velocity_controller.pid[1], 
                    angvel_error.pitch_rad_s, 
                    dt), 
                -1, 1);
            torque_out.yaw = normalize(
                pid_calculate(
                    &angular_velocity_controller.pid[2], 
                    angvel_error.yaw_rad_s, 
                    dt), 
                -1, 1);

            // Anti-reset windup
            float limit = 1;
            if (torque_out.roll >= limit || torque_out.roll <= -limit)
            {
                angular_velocity_controller.pid[0].integral -= angvel_error.roll_rad_s * dt;
            }
            if (torque_out.pitch >= limit || torque_out.pitch <= -limit)
            {
                angular_velocity_controller.pid[1].integral -= angvel_error.pitch_rad_s * dt;
            }
            if (torque_out.yaw >= limit || torque_out.yaw <= -limit)
            {
                angular_velocity_controller.pid[2].integral -= angvel_error.yaw_rad_s * dt;
            }

            /*
            printk("Angvel: %f %f %f\nAngvel sp: %f %f %f\nTorque out: %f %f %f\n",
                angvel.roll_rad_s, angvel.pitch_rad_s, angvel.yaw_rad_s,
                angvel_sp.roll_rad_s, angvel_sp.pitch_rad_s, angvel_sp.yaw_rad_s,
                torque_out.roll, torque_out.pitch, torque_out.yaw);
            */

            // Calculate velocity setpoint
            if (!velocity_override)
            {
                struct mec_vehicle_position ned_error;

                ned_error.north = position_sp.north - position.north;
                ned_error.east = position_sp.east - position.east;
                if (pos_controller.use_floor_altitude)
                    ned_error.down = position.altitude - position_sp.altitude;
                else
                    ned_error.down = position_sp.down - position.down;

                struct mec_vehicle_position_body error;
                position_ned_to_body(
                    &error, 
                    &ned_error, 
                    &attitude);

                float max_speed = 1.;

                velocity_body_sp.forward_m_s = normalize(
                    pid_calculate(
                        &pos_controller.pid[0], 
                        error.forward, 
                        dt), 
                    -max_speed, 
                    max_speed);

                velocity_body_sp.right_m_s = normalize(
                    pid_calculate(
                        &pos_controller.pid[1], 
                        error.right, 
                        dt), 
                    -max_speed, 
                    max_speed);

                velocity_body_sp.down_m_s = normalize(
                    pid_calculate(
                        &pos_controller.pid[2], 
                        error.down, 
                        dt), 
                    -max_speed, 
                    max_speed);            
            }

            // Calculate force necessary to get to that velocity setpoint
            struct mec_vehicle_velocity_body velocity_body_error;

            velocity_body_error.forward_m_s = velocity_body_sp.forward_m_s - velocity_body.forward_m_s;
            velocity_body_error.right_m_s = velocity_body_sp.right_m_s - velocity_body.right_m_s;
            velocity_body_error.down_m_s = velocity_body_sp.down_m_s - velocity_body.down_m_s;

            force_out.forward = normalize(pid_calculate(&vel_controller.pid[0], velocity_body_error.forward_m_s, dt), -1, 1);
            force_out.right = normalize(pid_calculate(&vel_controller.pid[1], velocity_body_error.right_m_s, dt), -1, 1);
            force_out.down = normalize(pid_calculate(&vel_controller.pid[2], velocity_body_error.down_m_s, dt), -1, 1);
            // printk("Vel: %f %f %f\nForce: %f %f %f\n", velocity_body_error.forward_m_s, velocity_body_error.right_m_s, velocity_body_error.down_m_s, force_out.forward, force_out.right, force_out.down);
            // ARW
            if (force_out.forward >= 1 || force_out.forward <= -1)
            {
                vel_controller.pid[0].integral -= velocity_body_error.forward_m_s * dt;
            }
            if (force_out.right >= 1 || force_out.right <= -1)
            {
                vel_controller.pid[1].integral -= velocity_body_error.right_m_s * dt;
            }
            if (force_out.down >= 1 || force_out.down <= -1)
            {
                vel_controller.pid[2].integral -= (velocity_body_error.down_m_s * dt);
            }

            // Map forces and torques to thruster outputs
            float thruster_outputs[8];
            mec_mix(&force_out, &torque_out, mix, power, thruster_outputs);
            send_thrusts(thruster_outputs); 
        }
        
        // Sleep so other sensor reading threads can run
        k_sleep(K_MSEC(20));
    }
    return 0;
}

