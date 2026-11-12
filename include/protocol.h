#ifndef MARITIME_PROTOCOL_H_
#define MARITIME_PROTOCOL_H_

/*
 * Full description of protocol can be found at https://sw.avbotz.com/maritime/protocol.
 * Each frame sent is:
 *
 *   0x00, COBS(type, payload), 0x00
 *
 * Multi-byte fields are little-endian. USB CDC-ACM already uses CRC, so an additional error
 * checking mechanism is not needed at this level.
 */

#include <zephyr/device.h>

#include <stdbool.h>
#include <stdint.h>

#define PROTOCOL_VERSION 3

/* Host to board */
#define MSG_THRUSTERS  0x01 /* f32 thrust[8], each between -1 to 1 */
#define MSG_TORPEDO    0x02 /* u8 id (0, 1), u8 open (0, 1) */
#define MSG_DROPPER    0x03 /* u8 id (0, 1), u8 open (0, 1) */
#define MSG_GRABBER    0x04 /* f32 closed, from 0 (open) to 1 (closed) */
#define MSG_SERVO      0x05 /* u8 servo (SERVO_ID_*), u16 pulse_us */
#define MSG_BOOTLOADER 0x06 /* u32 BOOTLOADER_MAGIC */

/* Board to host */
/* u8 protocol, u8 flags, f32 depth_m, u32 uptime_ms, u32 command_timeouts, u32 rx_errors */
#define MSG_STATE 0x81

#define PROTOCOL_NUM_THRUSTERS 8

/* MSG_SERVO's servo */
#define SERVO_ID_DROPPER 0
#define SERVO_ID_GRABBER 1
#define SERVO_ID_SHOOTER 2

/* "BOOT": rebooting into the bootloader stops the sub, so it takes more than one byte */
#define BOOTLOADER_MAGIC 0x544f4f42

struct host_msg {
	uint8_t type;
	union {
		/* MSG_THRUSTERS */
		float thrusts[PROTOCOL_NUM_THRUSTERS];
		/* MSG_TORPEDO, MSG_DROPPER */
		struct {
			uint8_t id;
			uint8_t open;
		} actuator;
		/* MSG_SERVO */
		struct {
			uint8_t id;
			uint16_t pulse_us;
		} servo;
		/* MSG_BOOTLOADER */
		uint32_t bootloader_magic;
		/* MSG_GRABBER */
		float grabber_closed;
	};
};

/* MSG_STATE payload */
struct board_state {
	bool killed;
	bool depth_valid;
	float depth_m;
	uint32_t uptime_ms;
	uint32_t command_timeouts;
};

/* Starts receiving and sending frames using `dev` */
void protocol_init(const struct device *dev);
/*
 * Returns 0 and modified msg if message from host has arrived or -EAGAIN if none have newly arrived
 */
int protocol_receive(struct host_msg *msg);
/* Send MSG_STATE */
void protocol_send_state(const struct board_state *state);

#endif /* MARITIME_PROTOCOL_H_ */
