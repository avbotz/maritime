/*
 * thruster.c
 *
 * Implements commanding and response parsing for the Video M5 thruster
 *
 * @author Kalyan Sriram <kalyan@coderkalyan.com>
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <logging/log.h>
#include <sys/ring_buffer.h>

#include "thruster.h"

LOG_MODULE_REGISTER(thruster, LOG_LEVEL_DBG);

#define THRUSTER_STACK_SIZE 1024
#define THRUSTER_PRIORITY 1

#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];

struct ring_buf ringbuf;

void uart_isr(const struct device *dev)
{
	/*static int idx = 0;*/
	static bool filled = false;
	/*LOG_DBG("ISR");*/
	printf("ISR\n");
	/*struct device *gpio = user_data;*/
	/*const struct device *gpio = device_get_binding("GPIOC");*/
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[] = {'H','e','l','l','o'};
			if (!filled) {
				uart_fifo_fill(dev, buffer, sizeof(buffer));
				filled = true;
			} else {
				uart_irq_tx_disable(dev);
			}
			printf("Done? %d\n", uart_irq_tx_complete(dev));
			/*gpio_pin_set(gpio, 6, 1);*/
			/*uart_fifo_fill(dev, buffer, sizeof(buffer));*/
		} else if (uart_irq_tx_complete(dev)) {
			/*gpio_pin_set(gpio, 6, 0);*/
			/*printf("Done\n");*/
			/*uart_irq_tx_disable(dev);*/
		}
	}
/*
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(buffer));

			recv_len = uart_fifo_read(dev, buffer, len);

			rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			if (rb_len < recv_len) {
				LOG_ERR("Drop %u bytes", recv_len - rb_len);
			}

			LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);

			uart_irq_tx_enable(dev);
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len) {
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
	}*/
	/*uart_irq_update(unused);*/

	/*int recv_len;*/
	/*uint8_t buffer[64];*/
	/*do {*/
		/*recv_len = uart_fifo_read(unused, buffer, sizeof(buffer));*/
		/*uart_fifo_write(unused, )*/
		/*[>printf(buffer);<]*/
	/*} while (recv_len > 0);*/
}

/*#define DE_NODE DT_NODELABEL(gpioc)*/
static void thruster_thread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

	/*LOG_INF("Initializing thruster control thread");*/

	int ret;

	const struct device *gpio = device_get_binding("GPIOC");

	gpio_pin_configure(gpio, 6, GPIO_OUTPUT | GPIO_ACTIVE_HIGH);
	gpio_pin_configure(gpio, 5, GPIO_OUTPUT | GPIO_ACTIVE_LOW);

	/*gpio_pin_set(gpio, 6, 1);*/
	/*gpio_pin_set(gpio, 5, 1);*/
	/*gpio_pin_set(gpio, 6, 1);*/
	/*gpio_pin_set(gpio, 5, 0);*/

	/*const struct device *uart = device_get_binding("UART_1");*/
	const struct device *uart = device_get_binding(DT_LABEL(DT_ALIAS(uart1)));
	const struct device *uart3 = device_get_binding(DT_LABEL(DT_ALIAS(uart3)));
	const struct device *lpuart1 = device_get_binding(DT_LABEL(DT_ALIAS(lpuart1)));

	printf("%d %d\n", uart, lpuart1);

	/*if (uart == NULL) printf("null\n");*/

	/*struct uart_config cfg;*/
	/*uart_config_get(uart, &cfg);*/

	/*printf("Sending %d %d %d %d %d\n", cfg.baudrate, cfg.parity,*/
			/*cfg.stop_bits, cfg.data_bits, cfg.flow_ctrl);*/

	/*uart_irq_callback_set(uart, uart_isr);*/
	/*uart_irq_rx_enable(uart);*/
	/*uint8_t buffer[] = {'H', 'e', 'l', 'l', 'o', ',', ' ', 'w', 'o', 'r', 'l', 'd', '!'};*/
	char *data = "Hello, world!";
	/*char *data = 0x42;*/

	uint8_t buffer[COMMAND_PROPULSION_LEN];
	memset(buffer, 0, COMMAND_PROPULSION_LEN);
	struct csr_command_propulsion_s prop;
	memset(prop.power, 0, sizeof(prop.power));
	/*for (int i = 0;i < 8;i++) prop.power[i] = 0.1;*/
	prop.power[7] = 0.1;
	prop.enable_response = true;
	csr_command_propulsion_pack(buffer, &prop);
	/*for (int i = 0;i < COMMAND_PROPULSION_LEN;i++) buffer[i] = 'H';*/
	/*buffer[0] = 'H';*/
	/*buffer[1] = 'e';*/

	printf("Hi\n");
	/*uart_irq_callback_set(uart, uart_isr);*/
	/*uart_irq_tx_enable(uart);*/
	/*uart_irq_rx_enable(uart);*/

	printf("done\n");

	/*return;*/
	/*gpio_pin_set(gpio, 6, 1);*/
	/*uart_fifo_write*/
	gpio_pin_set(gpio, 6, 1);
	k_sleep(K_MSEC(1));
	for (int i = 0;i < COMMAND_PROPULSION_LEN;i++) {
		printf("%02x ", buffer[i]);
		/*char c;*/
		/*sprintf(&c, "%x2", buffer[i]);*/
		uart_poll_out(uart, buffer[i]);
		/*uart_poll_out(uart, c);*/
		/*uart_poll_out(uart, 0x0d);*/
	}
	k_sleep(K_MSEC(1));
	gpio_pin_set(gpio, 6, 0);

	/*
	while (1) {
		for (int i = 0;i < strlen(data);i++) {
			gpio_pin_set(gpio, 6, 1);
			k_sleep(K_MSEC(1));
			uart_poll_out(uart, data[i]);
			k_sleep(K_MSEC(1));
			gpio_pin_set(gpio, 6, 0);
		}

		k_sleep(K_MSEC(250));
	}
	*/
	/*gpio_pin_set(gpio, 5, 0);*/

	/*while (1) {*/
		/*char c;*/
		/*ret = uart_poll_in(uart, &c);*/
		/*if (ret == 0) {*/
			/*printf("Rx: %s\n", ret);*/
		/*} else {*/
			/*printf("None");*/
		/*}*/

		/*printf(c);*/
	/*}*/
	/*uart_poll_out(uart, data);*/
	/*ret = uart_tx(uart, buffer, 1, SYS_FOREVER_MS);*/

	/*while (1) {*/
		/*ret = uart_irq_rx_ready(uart);*/
		/*printf("RX: %d\n", ret);*/

		/*k_sleep(K_MSEC(1));*/
	/*}*/
	/*printf("Sent %d\n", ret);*/
}

K_THREAD_DEFINE(thruster_thread_id, THRUSTER_STACK_SIZE,
				thruster_thread, NULL, NULL, NULL,
				THRUSTER_PRIORITY, 0, 0);
