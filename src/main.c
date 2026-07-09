// Need the following line to include strtok_r, as picolibc does not include it by default
// Seems to build fine though, though I get editor warnings if the following line is not included
#define _POSIX_C_SOURCE 200809L

#include "killswitch.h"
#include "pressure.h"
#include "thruster.h"
#include "servo.h"
#include "util.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef CONFIG_BOARD_RPI_PICO
#include <pico/bootrom.h>
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

static const struct device *usb_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#define MSG_SIZE 256
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 1);

static char rx_buf[MSG_SIZE];
static size_t rx_pos = 0;



static void uart_rx_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uint8_t c;

	uart_irq_update(dev);
	while (uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			if (uart_fifo_read(dev, &c, 1) != 1) {
				uart_irq_update(dev);
				continue;
			}

			if (c == '\n' || c == '\r') {
				if (rx_pos > 0) {
					rx_buf[rx_pos] = '\0';
					k_msgq_put(&uart_msgq, rx_buf, K_NO_WAIT);
					rx_pos = 0;
				}
			} else if (rx_pos < MSG_SIZE - 1) {
				rx_buf[rx_pos++] = (char)c;
			} else {
				rx_pos = 0;
			}
		}
		uart_irq_update(dev);
	}
}

int main(void)
{
	if (!device_is_ready(usb_device)) {
		LOG_ERR("CDC ACM device not ready");
		return -1;
	}

	uart_irq_callback_user_data_set(usb_device, uart_rx_handler, NULL);
	uart_irq_rx_enable(usb_device);
	setup_killswitch();
	setup_thrusters();
	setup_servos();
	setup_pressure();

	float init_thrusters[8] = {0.0f};

	send_thrusts(init_thrusters);

	// Provide initialize pulse to ESCs
	// https://docs.bluerobotics.com/bluesc/#:~:text=Provide%20a%20%E2%80%9Cstopped%E2%80%9D%20signal%20at%201500%20%CE%BCs%20for%20a%20few%20seconds
	k_sleep(K_SECONDS(3));
	
	char msg[MSG_SIZE];
	bool is_alive = false;
	int64_t prev_alive_time = k_uptime_get();

	while (true) {
		LOG_DBG("Recieved command: %s", msg);

		if (k_msgq_get(&uart_msgq, msg, K_NO_WAIT) == 0) {
			char *save_ptr;
			char *token = strtok_r(msg, " ", &save_ptr);
			if (!token) {
				continue;
			}
			char c = token[0];

			if (c == 'p') {
				char *thruster_token = strtok_r(NULL, " ", &save_ptr);
				char *thrust_token = strtok_r(NULL, " ", &save_ptr);
				int thruster;
				float thrust;

				if (!parse_int_arg(thruster_token, &thruster) ||
				    !parse_float_arg(thrust_token, &thrust)) {
					continue;
				}

				send_thrust(thruster, thrust);

				LOG_DBG("Setting thruster power: p %d %d\n", thruster, (int)(thrust * 1000));
			} else if (c == 'a') {
				char *thrust_token = strtok_r(NULL, " ", &save_ptr);
				float thrust;

				if (!parse_float_arg(thrust_token, &thrust)) {
					continue;
				}

				float thrusts[8] = {thrust, thrust, thrust, thrust, thrust, thrust, thrust, thrust};
				send_thrusts(thrusts);
			} else if (c == 's') {
				// Usage: s <type> <pulse>
				// type: g (grabber), t (shooter), d (dropper)
				// pulse: microseconds (1000 = 1ms)
				char *type_token = strtok_r(NULL, " ", &save_ptr);

				if (!type_token) {
					continue;
				}

				char type = type_token[0];

				char *pulse_token = strtok_r(NULL, " ", &save_ptr);
				int pulse;

				if (!parse_int_arg(pulse_token, &pulse)) {
					continue;
				}

				// convert to nanoseconds (1000 = 1ms)
				pulse = pulse * 1000;

				if (type == 'g') {
					set_pulse(SERVO_GRABBER, pulse);
				} else if (type == 't') {
					set_pulse(SERVO_SHOOTER, pulse);
				} else if (type == 'd') {
					set_pulse(SERVO_DROPPER, pulse);
				}
			} else if (c == 't') {
				char *selector_token = strtok_r(NULL, " ", &save_ptr);
				char *state_token = strtok_r(NULL, " ", &save_ptr);

				if (!selector_token || !state_token) {
					continue;
				}

				int selector = selector_token[0] == '1' ? 1 : 0;
				int state = state_token[0] == '1' ? 1 : 0;

				shoot(selector, state);
			} else if (c == 'd') {
				char *selector_token = strtok_r(NULL, " ", &save_ptr);
				char *state_token = strtok_r(NULL, " ", &save_ptr);

				if (!selector_token || !state_token) {
					continue;
				}

				int selector = selector_token[0] == '1' ? 1 : 0;
				int state = state_token[0] == '1' ? 1 : 0;

				drop(selector, state);
			} 
			#ifdef CONFIG_BOARD_RPI_PICO
			else if (c == 'r') {
				reset_usb_boot(0, 0);
			}
			#endif
		}

		int64_t current_time = k_uptime_get();
		if (current_time - prev_alive_time >= 200) {
			// Kill Switch
			if (!is_alive && alive()) {
				// Provide initialize pulse to ESCs
				k_sleep(K_SECONDS(3));
			}

			is_alive = alive();

			printk(is_alive ? "x 0\n" : "x 1\n");

			if (!is_alive) {
				send_thrusts(init_thrusters);
			}

			// Pressure Sensor
			float depth_m = get_depth_meters();
			// the pico does not support floating point formatting in printk, so we need to convert to integer parts
			int depth_m_int = (int)depth_m;
			int depth_m_frac = (int)((depth_m - depth_m_int) * 1000); 

			printk("d %d.%03d\n", depth_m_int, depth_m_frac);

			prev_alive_time = current_time;
		}
	}

	return 0;
}
