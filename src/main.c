#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uart_fifo_read(dev, &c, 1);

			if ((c == '\n' || c == '\r') && rx_pos > 0) {
				rx_buf[rx_pos] = '\0';
				k_msgq_put(&uart_msgq, rx_buf, K_NO_WAIT);
				rx_pos = 0;
			} else if (rx_pos < MSG_SIZE - 1) {
				rx_buf[rx_pos++] = (char)c;
			} else {
				rx_pos = 0;
			}
		}
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
	// setup_ahrs();
	setup_thrusters();
	setup_servos();

	char msg[MSG_SIZE];

	while (true) {
		if (k_msgq_get(&uart_msgq, msg, K_FOREVER) == 0) {
			char *save_ptr;
			char *token = strtok_r(msg, " ", &save_ptr);
			if (!token) {
				continue;
			}
			char c = token[0];

			if (c == 'a') {
				printk(alive() ? "alive\n" : "not alive\n");
			} else if (c == 'p') {
				printk("pong\n");
			} else if (c == 't') {
				float thrusts[8];
				for (int i = 0; i < 8; i++) {
					token = strtok_r(NULL, " ", &save_ptr);
					if (token) {
						thrusts[i] = strtof(token, NULL);
					} else {
						thrusts[i] = 0.0f;
					}
					printf("%s_g %f\n", token ? token : "0",
					       (double)thrusts[i]);
				}

				send_thrusts(thrusts);
				printk("sent thrusts\n");
			} else if (c == 'g') {
				// grabber
				grab(strtof(strtok_r(NULL, " ", &save_ptr), NULL));
			} else if (c == 'd') {
				// dropper
				int idx = atoi(strtok_r(NULL, " ", &save_ptr));
				int value = atoi(strtok_r(NULL, " ", &save_ptr));
				drop(idx, value);
			} else if (c == 's') {
				shoot(strtof(strtok_r(NULL, " ", &save_ptr), NULL));
			} else {
				LOG_WRN("Unknown command: %c", c);
			}
		}
	}

	return 0;
}
