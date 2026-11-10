#include "protocol.h"

#include <zephyr/kernel.h>
#include <zephyr/data/cobs.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>

#include <stdbool.h>

/* type */
#define HEADER_SIZE    1
#define FRAME_MAX_SIZE (HEADER_SIZE + FRAME_MAX_PAYLOAD)

/* One decoded frame as it arrived, not yet checked */
struct raw_frame {
	uint8_t len;
	uint8_t data[FRAME_MAX_SIZE];
};

K_MSGQ_DEFINE(rx_msgq, sizeof(struct raw_frame), 8, 1);

static const struct device *link;
static atomic_t rx_errors;

/* The frame being received, owned by the RX interrupt */
static struct cobs_decoder rx_decoder;
static struct raw_frame rx_frame;
/* The frame overran rx_frame: the rest of it is dropped up to its delimiter */
static bool rx_overflow;

static int payload_size(uint8_t type)
{
	switch (type) {
	case MSG_THRUSTERS:
		return THRUSTERS_SIZE;
	case MSG_TORPEDO:
	case MSG_DROPPER:
		return ACTUATOR_SIZE;
	case MSG_SERVO:
		return SERVO_SIZE;
	case MSG_BOOTLOADER:
		return BOOTLOADER_SIZE;
	default:
		return -1;
	}
}

static void rx_frame_reset(void)
{
	rx_frame.len = 0;
	rx_overflow = false;
}

/* Takes what the decoder outputs: decoded bytes, or NULL at a delimiter */
static int rx_decoded(const uint8_t *buf, size_t len, void *user_data)
{
	ARG_UNUSED(user_data);

	if (buf == NULL) {
		if (rx_overflow) {
			atomic_inc(&rx_errors);
		} else if (rx_frame.len > 0 && k_msgq_put(&rx_msgq, &rx_frame, K_NO_WAIT) != 0) {
			atomic_inc(&rx_errors);
		}
		/* Nothing between two delimiters is not a frame, nor an error */
		rx_frame_reset();
		return 0;
	}
	if (rx_frame.len + len > sizeof(rx_frame.data)) {
		rx_overflow = true;
		return 0;
	}
	memcpy(&rx_frame.data[rx_frame.len], buf, len);
	rx_frame.len += len;
	return 0;
}

static void rx_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uint8_t chunk[16];

	uart_irq_update(dev);
	while (uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			int n = uart_fifo_read(dev, chunk, sizeof(chunk));

			/* A byte at a time: the decoder drops the rest of what it is given after */
			/* an error, and the next frame may start right after a bad one */
			for (int i = 0; i < n; i++) {
				if (cobs_decoder_write(&rx_decoder, &chunk[i], 1) < 0) {
					/* A delimiter cut a frame short */
					atomic_inc(&rx_errors);
					rx_frame_reset();
				}
			}
		}
		uart_irq_update(dev);
	}
}

void protocol_init(const struct device *dev)
{
	link = dev;
	cobs_decoder_init(&rx_decoder, rx_decoded, NULL, COBS_FLAG_TRAILING_DELIMITER);
	uart_irq_callback_user_data_set(dev, rx_handler, NULL);
	uart_irq_rx_enable(dev);
}

int protocol_receive(struct frame *frame)
{
	struct raw_frame raw;

	while (k_msgq_get(&rx_msgq, &raw, K_NO_WAIT) == 0) {
		/* Never empty: rx_decoded() queues none */
		if (payload_size(raw.data[0]) != raw.len - HEADER_SIZE) {
			atomic_inc(&rx_errors);
			continue;
		}
		frame->type = raw.data[0];
		frame->len = raw.len - HEADER_SIZE;
		memcpy(frame->payload, &raw.data[HEADER_SIZE], frame->len);
		return 0;
	}
	return -EAGAIN;
}

static int tx_encoded(const uint8_t *buf, size_t len, void *user_data)
{
	ARG_UNUSED(user_data);

	for (size_t i = 0; i < len; i++) {
		uart_poll_out(link, buf[i]);
	}
	return 0;
}

void protocol_send(uint8_t type, const uint8_t *payload, size_t len)
{
	/* Static: the encoder's 255-byte buffer is too much for the main stack */
	static struct cobs_encoder encoder;
	uint8_t body[FRAME_MAX_SIZE];

	__ASSERT_NO_MSG(len <= FRAME_MAX_PAYLOAD);

	body[0] = type;
	memcpy(&body[HEADER_SIZE], payload, len);

	/* The leading delimiter ends whatever came before: a frame cut off by a reset, */
	/* or a log line in a debug build */
	uart_poll_out(link, COBS_DEFAULT_DELIMITER);
	cobs_encoder_init(&encoder, tx_encoded, NULL, COBS_FLAG_TRAILING_DELIMITER);
	cobs_encoder_write(&encoder, body, HEADER_SIZE + len);
	cobs_encoder_close(&encoder);
}

uint32_t protocol_rx_errors(void)
{
	return (uint32_t)atomic_get(&rx_errors);
}
