#include <stdio.h>

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include "io.hpp"

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <uuv_sensor_ros_plugins_msgs/msg/dvl.hpp>
#include <uuv_gazebo_ros_plugins_msgs/msg/float_stamped.hpp>

#include "simulator/commandParser.hpp"
#include "simulator/utils.hpp"
#include "mec/control.h"
#include "mec/estimation.h"
#include "mec/util.h"

using namespace std::chrono_literals;
using namespace std::literals;

bool velocity_override = false;
bool angvel_override = false;
Matrix<float, 6, 8> mix(nemo_mix_data);
struct att_controller attitude_controller;
struct angvel_controller angular_velocity_controller;
struct position_controller pos_controller;
struct velocity_controller vel_controller;

struct mec_torque_setpoint torque_out;
struct mec_force_setpoint force_out;

struct mec_vehicle_position position;

struct mec_vehicle_velocity_body velocity_body;

float vehicle_depth = 0;

struct mec_vehicle_attitude att_sp;
struct mec_vehicle_attitude attitude;
struct mec_vehicle_angvel angvel;
struct mec_vehicle_angvel angvel_sp;
struct mec_vehicle_velocity velocity_sp;
struct mec_vehicle_velocity_body velocity_body_sp;
struct mec_vehicle_position position_sp;
float last_pressure_update_time;
float last_pressure_depth = -999;

#define LED_PORT "GPIOA"
#define LED_PIN 5

void main(void)
{
	return;
	printf("Hello, world! %s\n", CONFIG_BOARD);

	const struct device *gpio_port_dev;
	int ret;

	gpio_port_dev = device_get_binding(LED_PORT);
	if (gpio_port_dev == NULL) {
		printf("Unable to initialize device %s\n", LED_PORT);
		return;
	}

	printf("Initialized GPIO device %s\n", LED_PORT);

	ret = gpio_pin_configure(gpio_port_dev, LED_PIN, GPIO_OUTPUT);
	if (ret < 0) {
		printf("Unable to configure %s pin %d: error %d\n", LED_PORT, LED_PIN, ret);
		return;
	}

	att_controller_init(&attitude_controller);
  angvel_controller_init(&angular_velocity_controller);
  position_controller_init(&pos_controller);
  velocity_controller_init(&vel_controller);
  mec_vehicle_position_init(&position);

  pos_controller.use_floor_depth = false;

  att_sp.roll = 0;
  att_sp.pitch = 0;
  att_sp.yaw = 45 * D2R;
  att_controller_update_sp(&attitude_controller, &att_sp);

  position_sp.down = 2;
  position_controller_update_sp(&pos_controller, &position_sp);

  // Control the simulated sub's thrusters.
  std::cout << "Thrusters initialized!" << std::endl;
  auto mtime = std::chrono::steady_clock::now();

	while (rclcpp::ok())
	{
		rclcpp::spin_some(node);
    auto t = std::chrono::steady_clock::now();
    float dt = ((t - mtime) / 1us) / 1000000.;
    mtime = t;

		mec_vehicle_position_update(&velocity_body,
    														vehicle_depth, &position, &attitude, dt);
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
                &velocity_sp,
                dt
                );
        velocity_ned_to_body(
                &velocity_body_sp,
                &velocity_sp,
                &attitude
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

    //Copied from main.cpp in nautical, changed slightly to accomodate maritime reporting
    if (Serial.available() > 0)
		{
			char c = Serial.read();
			if (c == 'a')
			{
				Serial << attitude.yaw << " " << attitude.pitch << " " << " " << attitude.roll << '\n';
			} else if (c == 'b') {
				Serial << angvel.yaw_rad_s << " " << angvel.pitch_rad_s << " " << " " << angvel.roll_rad_s << '\n';
			} else if (c == 'd') {
				Serial << pos_controller.use_floor_depth << '\n';
			} else if (c == 'p') {
				Serial << position.north << " " << position.east << " " << " " << position.down << '\n';
			} else if (c == 'v'){
				Serial << velocity_body.forward_m_s << " " << velocity_body.right_m_s << " " << " " << velocity_body.down_m_s << '\n';
			}

		  float thruster_outputs[8];
		  mec_mix(&force_out, &torque_out, mix, thruster_outputs);

		//printf("Toggling!\n");
		//gpio_pin_toggle(gpio_port_dev, LED_PIN);
		k_msleep(1000);
		}
	}
}
