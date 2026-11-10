#ifndef MARITIME_PROTOCOL_H_
#define MARITIME_PROTOCOL_H_

/*
 * The binary protocol to the host (https://sw.avbotz.com/maritime/protocol/ has the full
 * layout). Every frame on the wire is
 *
 *   0x00, COBS(type, payload), 0x00
 *
 * and every multi-byte field is little-endian. USB already checks and retransmits
 * what it carries, so there is no checksum: a frame whose payload is the wrong size
 * for its type is dropped and counted.
 */

#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define PROTOCOL_VERSION 3

/* Host to board */
#define MSG_THRUSTERS  0x01 /* f32 thrust[8], each between -1 and 1 */
#define MSG_TORPEDO    0x02 /* u8 id (0, 1), u8 open (0, 1) */
#define MSG_DROPPER    0x03 /* u8 id (0, 1), u8 open (0, 1) */
#define MSG_GRABBER    0x04
#define MSG_SERVO      0x05 /* u8 servo (SERVO_ID_*), u16 pulse_us */
#define MSG_BOOTLOADER 0x06 /* u32 BOOTLOADER_MAGIC */
/* Board to host */
#define MSG_STATE      0x81 /* u8 protocol, u8 flags, f32 depth_m, u32 uptime_ms, */
			    /* u32 command_timeouts, u32 rx_errors */

#define THRUSTERS_SIZE  32
#define ACTUATOR_SIZE   2
#define SERVO_SIZE      3
#define BOOTLOADER_SIZE 4
#define STATE_SIZE      18

/* MSG_SERVO's servo */
#define SERVO_ID_DROPPER 0
#define SERVO_ID_GRABBER 1
#define SERVO_ID_SHOOTER 2

/* "BOOT": rebooting into the bootloader stops the sub, so it takes more than one byte */
#define BOOTLOADER_MAGIC 0x544f4f42

/* MSG_STATE's flags */
#define STATE_KILLED      BIT(0) /* Thrust commands are not obeyed */
#define STATE_DEPTH_VALID BIT(1) /* depth_m is a reading, not a placeholder */

#define FRAME_MAX_PAYLOAD THRUSTERS_SIZE

struct frame {
	uint8_t type;
	uint8_t len;
	uint8_t payload[FRAME_MAX_PAYLOAD];
};

/* Starts receiving frames from `dev`, and sends them to it */
void protocol_init(const struct device *dev);
/* The next well-formed frame from the host: 0, or -EAGAIN if none has arrived */
int protocol_receive(struct frame *frame);
/* Sends one frame of `type`. Only ever from one thread. */
void protocol_send(uint8_t type, const uint8_t *payload, size_t len);
/* Frames from the host dropped since boot: garbled, or no room to queue them */
uint32_t protocol_rx_errors(void);

static inline float protocol_get_f32(const uint8_t *src)
{
	uint32_t bits = sys_get_le32(src);
	float value;

	memcpy(&value, &bits, sizeof(value));
	return value;
}

static inline void protocol_put_f32(float value, uint8_t *dst)
{
	uint32_t bits;

	memcpy(&bits, &value, sizeof(bits));
	sys_put_le32(bits, dst);
}

#endif /* MARITIME_PROTOCOL_H_ */
