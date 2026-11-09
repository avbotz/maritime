#include "killswitch.h"
#include "pressure.h"
#include "protocol.h"
#include "thruster.h"
#include "servo.h"
#include "watchdog.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include <stdbool.h>

#ifdef CONFIG_BOARD_RPI_PICO
#include <pico/bootrom.h>
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* Thrusters go neutral if no thrust command arrives for this long [ms] */
static const int64_t COMMAND_TIMEOUT_MS = 500;
/* Time between kill switch powering up ESCs and thrust commands being sent [ms].
 * The ESCs must receive a 1500 us pulse for this long so they can initialize properly.
 */
static const int64_t ESC_INIT_MS = 750;
/* Time between state frames to the host [ms] */
static const int64_t STATE_PERIOD_MS = 200;
/* Depth is re-zeroed at a kill switch release only within this of the surface [m] */
static const float REZERO_LIMIT_M = 0.5f;

static const struct device *usb_device = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

/* What the thrusters may do */
struct thrust_state {
	/* Released, and the ESCs have had their neutral since: thrust commands are obeyed */
	bool running;
	/* A thrust command has arrived since running began, at last_command [ms] */
	bool commanded;
	int64_t last_command;
	/* Times the command watchdog stopped the thrusters */
	uint32_t command_timeouts;
};

static void handle_frame(const struct frame *frame, struct thrust_state *thrust, int64_t now)
{
	const uint8_t *payload = frame->payload;

	switch (frame->type) {
	case MSG_THRUSTERS: {
		if (!thrust->running) {
			break;
		}
		float thrusts[NUM_THRUSTERS];

		for (int i = 0; i < NUM_THRUSTERS; i++) {
			thrusts[i] = protocol_get_f32(&payload[4 * i]);
		}
		thruster_send_all(thrusts);
		thrust->commanded = true;
		thrust->last_command = now;
		break;
	}
	case MSG_TORPEDO:
		if (payload[0] > 1 || payload[1] > 1) {
			break;
		}

		servo_shoot(payload[0], payload[1]);
	case MSG_DROPPER:
		if (payload[0] > 1 || payload[1] > 1) {
			break;
		}

		servo_drop(payload[0], payload[1]);
		break;
	case MSG_SERVO: {
		/* Clamped to the servo's range from the overlay */
		const int pulse_us = sys_get_le16(&payload[1]);

		switch (payload[0]) {
		case SERVO_ID_DROPPER:
			servo_set_pulse(SERVO_DROPPER, pulse_us);
			break;
		case SERVO_ID_GRABBER:
			servo_set_pulse(SERVO_GRABBER, pulse_us);
			break;
		case SERVO_ID_SHOOTER:
			servo_set_pulse(SERVO_SHOOTER, pulse_us);
			break;
		}
		break;
	}
#ifdef CONFIG_BOARD_RPI_PICO
	case MSG_BOOTLOADER:
		if (sys_get_le32(payload) == BOOTLOADER_MAGIC) {
			thruster_stop();
			reset_usb_boot(0, 0);
		}
		break;
#endif
	}
}

static void send_state(const struct thrust_state *thrust, int64_t now)
{
	uint8_t payload[STATE_SIZE];
	/* Left 0 and flagged invalid, rather than made up, if the sensor cannot be read */
	float depth_m = 0.0f;
	const bool depth_valid = pressure_get_depth_m(&depth_m) == 0;

	payload[0] = PROTOCOL_VERSION;
	/* Reported released only once thrust commands are obeyed */
	payload[1] = (thrust->running ? 0 : STATE_KILLED) | (depth_valid ? STATE_DEPTH_VALID : 0);
	protocol_put_f32(depth_m, &payload[2]);
	sys_put_le32((uint32_t)now, &payload[6]);
	sys_put_le32(thrust->command_timeouts, &payload[10]);
	sys_put_le32(protocol_rx_errors(), &payload[14]);
	protocol_send(MSG_STATE, payload, sizeof(payload));
}

int main(void)
{
	if (!device_is_ready(usb_device)) {
		LOG_ERR("CDC ACM device not ready");
		return -1;
	}

	protocol_init(usb_device);
	/* Keeps running, so the host still gets state frames: they say killed */
	if (killswitch_init() < 0) {
		LOG_ERR("No kill switch: staying killed");
	}
	if (thruster_init() < 0) {
		LOG_ERR("No thrusters");
	}
	if (servo_init() < 0) {
		LOG_ERR("No servos");
	}
	if (pressure_init() < 0) {
		LOG_ERR("No pressure sensor");
	}
	if (watchdog_init() < 0) {
		LOG_ERR("No watchdog");
	}

	/* Neutral until the kill switch is released and the ESCs have initialized */
	thruster_stop();

	struct frame frame;
	struct thrust_state thrust = {0};
	/* The kill switch as debounced (killswitch.h): starts killed until it is cycled */
	bool released = false;
	int64_t esc_ready_at = 0;
	int64_t now = k_uptime_get();
	int64_t next_state = now;

	while (true) {
		now = k_uptime_get();

		watchdog_feed();

		const bool was_released = released;

		released = killswitch_update(now) == KILLSWITCH_RELEASED;
		if (released && !was_released) {
			/* The switch powers the ESCs up: they need neutral first */
			esc_ready_at = now + ESC_INIT_MS;
		} else if (!released && was_released) {
			thrust.running = false;
			thrust.commanded = false;
			thruster_stop();
		}
		if (released && !thrust.running && now >= esc_ready_at) {
			thrust.running = true;
			thrust.commanded = false;
			/* The surface for this run, unless released underwater */
			pressure_rezero_near_surface(REZERO_LIMIT_M);
		}

		while (protocol_receive(&frame) == 0) {
			handle_frame(&frame, &thrust, now);
		}

		/* The host stopped commanding (crashed, unplugged): stop rather than keep */
		/* driving on the last command */
		if (thrust.running && thrust.commanded &&
		    now - thrust.last_command > COMMAND_TIMEOUT_MS) {
			thruster_stop();
			thrust.commanded = false;
			thrust.command_timeouts++;
		}

		if (now >= next_state) {
			next_state = now + STATE_PERIOD_MS;
			if (!thrust.running) {
				thruster_stop();
			}

			send_state(&thrust, now);
		}

		/* Yield so lower-priority threads (log thread) get CPU time; without */
		/* this the busy loop starves them */
		k_sleep(K_MSEC(1));
	}

	return 0;
}
