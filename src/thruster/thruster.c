/*
 * thruster.c
 *
 * Implements commanding and response parsing for the Video M5 thruster
 *
 * @author Kalyan Sriram <kalyan@coderkalyan.com>
 */

#include <logging/log.h>
#include <sys/ring_buffer.h>

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

#include <pubsub/pubsub.h>
#include <pubsub/thruster_output.h>

#include "thruster.h"

LOG_MODULE_REGISTER(thruster, LOG_LEVEL_DBG);
/*LOG_MODULE_REGISTER(thruster, LOG_LEVEL_ERR);*/

#define THRUSTER_STACK_SIZE 1024
#define THRUSTER_PRIORITY 1

/* standard thruster command for 8 thrusters is 48 bytes
 * if we, say, want to queue 4 messages, which is 192 bytes
 * Doesn't really matter, but we're choosing a power of two
 * (256 byte buffer)
 *
 * note that only the data item version of the zephyr ring buffer
 * implementation (which stores 32-bit word data items) benefits from
 * using a power of 2; we want a byte mode buffer though
 *
 * note that this a rough estimate and does not take into account
 * any sending limitations or other messages
 *
 * note also that if you are actually queued up by 5 or more messages,
 * you're doing something wrong and should probably look into
 * your loop timings
 */
#define THRUSTER_RING_BUF_SIZE 256
RING_BUF_DECLARE(tx_ring_buf, THRUSTER_RING_BUF_SIZE);
RING_BUF_DECLARE(rx_ring_buf, THRUSTER_RING_BUF_SIZE);

PUBSUB_SUBSCRIBER_DEFINE(thruster_output_topic, thruster_output_sub, 0);

#define MAX485_NODE DT_ALIAS(max485_thruster)
#if DT_NODE_HAS_STATUS(MAX485_NODE, okay)
#define MAX485_UART_NODE DT_BUS(MAX485_NODE)
#define MAX485_UART_LABEL DT_LABEL(MAX485_UART_NODE)
#else
#error "Compatible MAX485 node not found."
#endif

/* TODO: disabling DMA completely for now, add back when it works */
static bool uart_transmit_dma(const struct device *dev)
{
	/* TODO: having a buffer size of 1 byte completely removes the point
	 * of using DMA. However, larger buffer sizes (although they should work)
	 * are giving me issues, and I've spent too long staring at the UART driver.
	 * This should be revisited some time, but for now it's not a big problem
	 * as performance isn't being hit much anyway. (one might question why we 
	 * are using DMA in the first place then :)
	 *
	 * The issues were around reliability of the data, sometimes some bytes were
	 * not being sent correctly. I suspect this might have nothing to do with DMA and
	 * everything to do with a memory leak or something not being zeroed correctly. */
	uint8_t buffer[1];
	int rb_len, ret;

	rb_len = ring_buf_get(&tx_ring_buf, buffer, sizeof(buffer));
	if (!rb_len) {
		return true;
	}

	ret = uart_tx(dev, buffer, rb_len, 100);
	if (ret != 0) {
		LOG_ERR("Unable to transmit UART: %d", ret);
	}

	return false;
}

static void uart_dma_isr(const struct device *dev, struct uart_event *event, void *data)
{
	struct device *gpio = data;

	switch (event->type) {
	case UART_TX_DONE:;
		bool finished = uart_transmit_dma(dev);
		if (finished) {
			gpio_pin_set(gpio, 6, 0);
		}

		break;
	finally:
		break;
	};
}

static void uart_isr(const struct device *dev, void *data)
{
	struct device *gpio = data;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		/* TODO: implement RX functionality as needed */
		if (uart_irq_tx_complete(dev) && ring_buf_is_empty(&tx_ring_buf)) {
			gpio_pin_set(gpio, 6, 0);
			uart_irq_tx_disable(dev);

			break;
		}

		if (uart_irq_tx_ready(dev)) {
			int rb_len, send_len;

			/* TODO: can the TX fifo handle multiple bytes?
			 * CDCACM can but UART doesn't seem to
			 * STM32G4 datasheet says the TX fifo is only 1 byte,
			 * but this should be verified */
			uint8_t buffer[1];

			rb_len = ring_buf_get(&tx_ring_buf, buffer, sizeof(buffer));
			if (!rb_len) {
				continue;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("tx: Dropping %d bytes", rb_len - send_len);
			}
		}
	}
}

static void thruster_thread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	LOG_INF("Initializing thruster control thread");

	const struct device *max485_uart = device_get_binding(MAX485_UART_LABEL);
	if (max485_uart == NULL) {
		LOG_ERR("Unable to bind max485 uart device");
		return;
	}

	const struct device *gpio = device_get_binding("GPIOC");
	gpio_pin_configure(gpio, 6, GPIO_OUTPUT | GPIO_ACTIVE_HIGH);

	/*uart_callback_set(max485_uart, uart_dma_isr, gpio);*/
	uart_irq_callback_user_data_set(max485_uart, uart_isr, gpio);

	uint8_t buffer[COMMAND_PROPULSION_LEN];
	memset(buffer, 0, COMMAND_PROPULSION_LEN);

	struct thruster_output_s thruster_output;
	int ret;

	while (1) {
		ret = pubsub_poll(&thruster_output_sub, K_MSEC(10));
		if (ret > 0) {
			pubsub_copy(&thruster_output_sub, &thruster_output);

			struct csr_command_propulsion_s prop;

			memset(&prop, 0, sizeof(prop));
			prop.enable_response = false;
			memcpy(prop.power, thruster_output.power, sizeof(thruster_output.power));

			csr_command_propulsion_pack(buffer, &prop);

			ring_buf_put(&tx_ring_buf, buffer, COMMAND_PROPULSION_LEN);
			gpio_pin_set(gpio, 6, 1);
			uart_irq_tx_enable(max485_uart);
		} else if (ret == 0) {
			LOG_WRN("Did not receive new thruster output data for over 10ms");
		} else {
			LOG_ERR("Error polling for thruster output: %d", ret);
		}
	}
}

K_THREAD_DEFINE(thruster_thread_id, THRUSTER_STACK_SIZE,
				thruster_thread, NULL, NULL, NULL,
				THRUSTER_PRIORITY, 0, 0);
