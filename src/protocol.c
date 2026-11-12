#include "protocol.h"

#include <zephyr/kernel.h>
#include <zephyr/data/cobs.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <string.h>

BUILD_ASSERT(sizeof(float) == 4, "float must be 4 bytes");

/*
 * Fields are represented as bytes, so they can be accessed through sys_get_le*(). This prevents
 * issues with byte ordering and unaligned access (versus using plain types like float or uint16_t).
 */
struct wire_thrusters {
	uint8_t thrust[PROTOCOL_NUM_THRUSTERS][4]; /* f32 */
} __packed;

struct wire_actuator {
	uint8_t id;
	uint8_t open;
} __packed;

struct wire_servo {
	uint8_t id;
	uint8_t pulse_us[2]; /* u16 */
} __packed;

struct wire_bootloader {
	uint8_t magic[4]; /* u32 */
} __packed;

struct wire_grabber {
	uint8_t closed[4]; /* f32 */
} __packed;

struct wire_state {
	uint8_t protocol;
	uint8_t flags;
	uint8_t depth_m[4];          /* f32 */
	uint8_t uptime_ms[4];        /* u32 */
	uint8_t command_timeouts[4]; /* u32 */
	uint8_t rx_errors[4];        /* u32 */
} __packed;

union wire_payload {
	struct wire_thrusters thrusters;
	struct wire_actuator actuator;
	struct wire_servo servo;
	struct wire_bootloader bootloader;
	struct wire_grabber grabber;
	struct wire_state state;
};

/* MSG_STATE's flags */
#define STATE_KILLED      BIT(0)
#define STATE_DEPTH_VALID BIT(1)

#define HEADER_SIZE       1
#define FRAME_MAX_PAYLOAD sizeof(union wire_payload)
#define FRAME_MAX_SIZE    (HEADER_SIZE + FRAME_MAX_PAYLOAD)

struct raw_frame {
	uint8_t len;
	uint8_t data[FRAME_MAX_SIZE];
};

K_MSGQ_DEFINE(rx_msgq, sizeof(struct raw_frame), 8, 1);

static const struct device *link;
static atomic_t rx_errors;

static struct cobs_decoder rx_decoder;
static struct raw_frame rx_frame;
static bool rx_overflow;

static struct cobs_encoder tx_encoder;

static float get_f32(const uint8_t *src)
{
	uint32_t bits = sys_get_le32(src);
	float value;

	memcpy(&value, &bits, sizeof(value));
	return value;
}

static void put_f32(float value, uint8_t *dst)
{
	uint32_t bits;

	memcpy(&bits, &value, sizeof(bits));
	sys_put_le32(bits, dst);
}

static int payload_size(uint8_t type)
{
	switch (type) {
	case MSG_THRUSTERS:
		return sizeof(struct wire_thrusters);
	case MSG_TORPEDO:
	case MSG_DROPPER:
		return sizeof(struct wire_actuator);
	case MSG_SERVO:
		return sizeof(struct wire_servo);
	case MSG_BOOTLOADER:
		return sizeof(struct wire_bootloader);
	case MSG_GRABBER:
		return sizeof(struct wire_grabber);
	default:
		return -1;
	}
}

/*
 * Decodes a frame from the host into msg. Returns false for unknown types of if payload is the
 * wrong size for its type, otherwise true.
 */
static bool decode(const struct raw_frame *raw, struct host_msg *msg)
{
	const uint8_t type = raw->data[0];
	const union wire_payload *p = (const void *)&raw->data[HEADER_SIZE];

	if (payload_size(type) != raw->len - HEADER_SIZE) {
		return false;
	}

	msg->type = type;
	switch (type) {
	case MSG_THRUSTERS:
		for (int i = 0; i < PROTOCOL_NUM_THRUSTERS; i++) {
			msg->thrusts[i] = get_f32(p->thrusters.thrust[i]);
		}
		break;
	case MSG_TORPEDO:
	case MSG_DROPPER:
		msg->actuator.id = p->actuator.id;
		msg->actuator.open = p->actuator.open;
		break;
	case MSG_SERVO:
		msg->servo.id = p->servo.id;
		msg->servo.pulse_us = sys_get_le16(p->servo.pulse_us);
		break;
	case MSG_BOOTLOADER:
		msg->bootloader_magic = sys_get_le32(p->bootloader.magic);
		break;
	case MSG_GRABBER:
		msg->grabber_closed = get_f32(p->grabber.closed);
		break;
	}
	return true;
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

	/*
	 * End of COBS frame. This only occurs after a frame is sent. Delimiters outside of valid
	 * COBS frames, like the one at the start of each frame, are ignored by Zephyr's COBS
	 * implementation.
	 */
	if (buf == NULL) {
		if (rx_overflow) {
			atomic_inc(&rx_errors);
		} else if (rx_frame.len > 0 && k_msgq_put(&rx_msgq, &rx_frame, K_NO_WAIT) != 0) {
			atomic_inc(&rx_errors);
		}

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
			int bytes_read = uart_fifo_read(dev, chunk, sizeof(chunk));

			/*
			 * A byte at a time: the decoder drops the rest of what it is given after an
			 * error, and the next frame may start right after a bad one
			 */
			for (int i = 0; i < bytes_read; i++) {
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

int protocol_receive(struct host_msg *msg)
{
	struct raw_frame raw;

	while (k_msgq_get(&rx_msgq, &raw, K_NO_WAIT) == 0) {
		if (decode(&raw, msg)) {
			return 0;
		}
		atomic_inc(&rx_errors);
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

static void send_frame(uint8_t type, const uint8_t *payload, size_t len)
{
	uart_poll_out(link, COBS_DEFAULT_DELIMITER);
	cobs_encoder_write(&tx_encoder, &type, HEADER_SIZE);
	cobs_encoder_write(&tx_encoder, payload, len);
	cobs_encoder_close(&tx_encoder);
}

void protocol_send_state(const struct board_state *state)
{
	struct wire_state w = {
		.protocol = PROTOCOL_VERSION,
		.flags = (state->killed ? STATE_KILLED : 0) |
			 (state->depth_valid ? STATE_DEPTH_VALID : 0),
	};

	put_f32(state->depth_m, w.depth_m);
	sys_put_le32(state->uptime_ms, w.uptime_ms);
	sys_put_le32(state->command_timeouts, w.command_timeouts);
	sys_put_le32((uint32_t)atomic_get(&rx_errors), w.rx_errors);
	send_frame(MSG_STATE, (const uint8_t *)&w, sizeof(w));
}

void protocol_init(const struct device *dev)
{
	link = dev;
	cobs_decoder_init(&rx_decoder, rx_decoded, NULL, COBS_FLAG_TRAILING_DELIMITER);
	uart_irq_callback_user_data_set(dev, rx_handler, NULL);
	uart_irq_rx_enable(dev);

	cobs_encoder_init(&tx_encoder, tx_encoded, NULL, COBS_FLAG_TRAILING_DELIMITER);
}
