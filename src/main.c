#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "ahrs.h"
#include "thruster.h"
#include "servo.h"
#include "killswitch.h"
#include "util.h"

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
#define MSG_SIZE         256

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 1);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(UART_DEVICE_NODE, zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

const struct device *usb_device = DEVICE_DT_GET(UART_DEVICE_NODE);

static char rx_buf[MSG_SIZE];
static size_t rx_pos = 0;

void cdc_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(dev)) {
		return;
	}

	printk("CDC CB Triggered1\n");
	while (uart_irq_rx_ready(dev)) {
		printk("CDC CB Triggered\n");
		uart_fifo_read(dev, &c, 1);
		printk("%c _", c);

		if ((c == '\n' || c == '\r') && rx_pos > 0) {
			rx_buf[rx_pos] = '\0';
			printk("Received command: %s\n", rx_buf);
			k_msgq_put(&uart_msgq, rx_buf, K_NO_WAIT);
			rx_pos = 0;
		} else if (rx_pos < MSG_SIZE - 1) {
			rx_buf[rx_pos++] = c;
		} else {
			LOG_WRN("UART RX buffer overflow");
		}
	}
}

void process_command(const char *inp)
{
	const char delim[] = " ";
	char *save;
	char *tok = strtok_r((char *)inp, delim, &save);
	if (!tok) {
		return;
	}

	char cmd = tok[0];
	if (cmd == 'a') {
		printk("%i\n", alive());
	} else if (cmd == 'p') {
		printk("pong\n");
		// } else if (cmd == 't') {
		// 	float thrusts[8];
		// 	for (int i = 0; i < 8; i++) {
		// 		thrusts[i] = parse_float(delim, &save);
		// 	}
		// 	send_thrusts(thrusts);
		// 	LOG_ERR("Thrusts updated\n");
		// } else if (cmd == 'g') {
		// 	int idx = parse_int(delim, &save);
		// 	int val = parse_int(delim, &save);
		// 	drop(idx, val);
		// } else if (cmd == 'f') {
		// 	float val = parse_float(delim, &save);
		// 	grab(val);
		// } else if (cmd == 'o') {
		// 	int idx = parse_int(delim, &save);
		// 	int val = parse_int(delim, &save);
		// 	shoot(idx, val);
	} else {
		LOG_DBG("Unknown command: %s\n", cmd);
	}
}

int main(void)
{
	struct ahrs_data_s ahrs_m;
	if (!device_is_ready(usb_device)) {
		LOG_ERR("CDC device not ready");
		return -1;
	}
	// usb_enable(NULL);

	uart_irq_rx_enable(usb_device);
	uart_irq_callback_set(usb_device, cdc_cb);
	setup_killswitch();
	setup_ahrs();
	setup_thrusters();
	setup_servos();

	char msg[MSG_SIZE];

	while (true) {
		if (k_msgq_get(&uart_msgq, msg, K_NO_WAIT) == 0) {
			process_command(msg);
		}
		if (k_msgq_get(&ahrs_data_msgq, &ahrs_m, K_NO_WAIT) == 0) {
			LOG_DBG("@%lld ms: Yaw %f, Pitch %f, Roll %f\n", (long long)ahrs_m.ts_us,
				(double)ahrs_m.yaw, (double)ahrs_m.pitch, (double)ahrs_m.roll);
		}

		printk("Killswitch state check: %d\n", alive() ? 1 : 0);
		LOG_DBG("Killswitch state: %d\n", alive() ? 1 : 0);
		k_sleep(K_MSEC(10));
	}

	return 0;
}
