#include "killswitch.h"
#include "thruster.h"
#include "util.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdbool.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

static const struct device *usb_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#define MSG_SIZE 256
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 1);

static char rx_buf[MSG_SIZE];
static size_t rx_pos = 0;

static bool parse_int_arg(char *token, int *value)
{
	char *end;
	long parsed;

	if (token == NULL || token[0] == '\0') {
		return false;
	}

	errno = 0;
	parsed = strtol(token, &end, 10);
	if (errno != 0 || *end != '\0') {
		return false;
	}

	*value = (int)parsed;
	return true;
}

static bool parse_float_arg(char *token, float *value)
{
	char *end;
	float parsed;

	if (token == NULL || token[0] == '\0') {
		return false;
	}

	errno = 0;
	parsed = strtof(token, &end);
	if (errno != 0 || *end != '\0') {
		return false;
	}

	*value = parsed;
	return true;
}

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
	// setup_servos();

	float init_thrusters[8] = {0.0f};

	send_thrusts(init_thrusters);

	char msg[MSG_SIZE];
	int64_t prev_alive_time = k_uptime_get();

	while (true) {
		if (k_msgq_get(&uart_msgq, msg, K_NO_WAIT) == 0) {
			char *save_ptr;
			char *token = strtok_r(msg, " \t", &save_ptr);
			if (!token) {
				continue;
			}
			char c = token[0];

			if (c == 'p') {
				char *thruster_token = strtok_r(NULL, " \t", &save_ptr);
				char *thrust_token = strtok_r(NULL, " \t", &save_ptr);
				int thruster;
				float thrust;

				if (!parse_int_arg(thruster_token, &thruster) ||
				    !parse_float_arg(thrust_token, &thrust)) {
					continue;
				}

				send_thrust(thruster, thrust);

				printk("p %d %d\n", thruster, (int)(thrust * 1000));
			} else if (c == 'a') {
				char *thrust_token = strtok_r(NULL, " \t", &save_ptr);
				float thrust;

				if (!parse_float_arg(thrust_token, &thrust)) {
					continue;
				}

				float thrusts[8] = {thrust, thrust, thrust, thrust, thrust, thrust, thrust, thrust};
				send_thrusts(thrusts);
			}
		}

		int64_t current_time = k_uptime_get();
		if (current_time - prev_alive_time >= 100) {
			// printk(alive() ? "x 0" : "x 1");
			prev_alive_time = current_time;
		}
	}

	return 0;
}
