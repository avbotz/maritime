//Script to report angular velocity, attitude, state, linear velocity, and depth mode
#include <zephyr.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <drivers/gpio.h>
#include <string.h>

#include <ahrs.h>
#include <dvl.h>
#include <thruster.h>
#include <pressure_sensor.h>

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

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 4096

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static const struct device *const dvl = get_DVL_device();
static const struct device *const mag = get_rm3100_device();
static const struct device *const imu = get_icm20689_device();

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
//For printing doubles, not characters
void print_double_over_uart(double num){
  char arr[sizeof(num)];
  memcpy(arr, &num, sizeof(num));
  for(int i = 0; i < sizeof(arr); i++){
    uart_poll_out(uart_dev, arr[i]);
  }
}

void main(void)
{
	char tx_buf[MSG_SIZE];

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return;
	}

	/* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	uart_irq_rx_enable(uart_dev);

	//print_uart("Hello! I'm your echo bot.\r\n");
	//print_uart("Tell me something and press enter:\r\n");

  struct ahrs_sample ahrs_sample;
  struct imu_sample imu_sample;
	struct mag_sample mag_sample;

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

	/* indefinitely wait for input from the user */
	while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
    //Sample IMU on AHRS
    int imu_ret = sample_imu(dev, &ahrs_sample);
  	if (imu_ret != 0) {
  		LOG_ERR("Error sampling IMU: %d", imu_ret);
  	}

    //Sample DVL
    int dvl_ret = sample_dvl(dev, &dvl_sample);
  	if (dvl_ret != 0) {
  		LOG_ERR("Error sampling DVL: %d", dvl_ret);
  	}

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

    if(tx_buf == 'a'){
      //Print attitude over uart
			print_double_over_uart(ahrs_sample->gyro[0]);
      print_uart(' ');
      print_double_over_uart(ahrs_sample->gyro[1]);
      print_uart(' ');
      print_double_over_uart(ahrs_sample->gyro[2]);
      print_uart('\n');
    } else if(tx_buf == 'b'){
      //Print angular velocity over uart
			print_double_over_uart(velocity_body.forward_m_s);
      print_uart(' ');
      print_double_over_uart(velocity_body.right_m_s);
      print_uart(' ');
      print_double_over_uart(velocity_body.down_m_s);
      print_uart('\n');
    } else if(tx_buf == 'd'){
			print_double_over_uart(pos_controller.use_floor_depth);
      print_uart('\n);
      //Print depth mode over uart
    } else if (tx_buf == 'p'){
      //Print current position over uart
			print_double_over_uart(pos.north);
      print_uart(' ');
      print_double_over_uart(pos.east);
      print_uart(' ');
      print_double_over_uart(pos.down);
      print_uart('\n');
    } else if (tx_buf == 'x'){
      //Print desired position over uart
			print_double_over_uart(pos_controller.north);
      print_uart(' ');
      print_double_over_uart(pos_controller.east);
      print_uart(' ');
      print_double_over_uart(pos_controller.down);
      print_uart('\n');
    } else if (tx_buf == 'v'){
      //Print linear velocity over uart
      print_double_over_uart(dvl_sample->changeX);
      print_uart(' ');
      print_double_over_uart(dvl_sample->changeY);
      print_uart(' ');
      print_double_over_uart(dvl_sample->changeZ);
      print_uart('\n');
    }
	}
}
