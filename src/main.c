#include "arming.h"
#include "killswitch.h"
#include "pressure.h"
#include "protocol.h"
#include "servo.h"
#include "thruster.h"
#include "watchdog.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stdbool.h>

#ifdef CONFIG_SOC_FAMILY_RPI_PICO
#include <pico/bootrom.h>
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

BUILD_ASSERT(PROTOCOL_NUM_THRUSTERS == NUM_THRUSTERS,
	     "Number of thrusters in protocol and overlay must match");

/* Time between state frames to the host [ms] */
static const int64_t STATE_PERIOD_MS = 100;

static const struct device *usb_device = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

static void handle_msg(const struct host_msg *msg, int64_t now)
{
	switch (msg->type) {
	case MSG_THRUSTERS:
		arming_command(msg->thrusts, now);
		break;
	case MSG_TORPEDO:
		if (msg->actuator.id > 1 || msg->actuator.open > 1) {
			break;
		}

		servo_shoot(msg->actuator.id, msg->actuator.open);
		break;
	case MSG_DROPPER:
		if (msg->actuator.id > 1 || msg->actuator.open > 1) {
			break;
		}

		servo_drop(msg->actuator.id, msg->actuator.open);
		break;
#if HAVE_GRABBER
	case MSG_GRABBER:
		servo_grab(msg->grabber_closed);
		break;
#endif
	case MSG_SERVO:
		/* Clamped to the servo's range from the overlay */
		switch (msg->servo.id) {
		case SERVO_ID_DROPPER:
			servo_set_pulse(SERVO_DROPPER, msg->servo.pulse_us);
			break;
#if HAVE_GRABBER
		case SERVO_ID_GRABBER:
			servo_set_pulse(SERVO_GRABBER, msg->servo.pulse_us);
			break;
#endif
		case SERVO_ID_SHOOTER:
			servo_set_pulse(SERVO_SHOOTER, msg->servo.pulse_us);
			break;
		}
		break;
#ifdef CONFIG_SOC_FAMILY_RPI_PICO
	case MSG_BOOTLOADER:
		if (msg->bootloader_magic == BOOTLOADER_MAGIC) {
			thruster_stop();
			reset_usb_boot(0, 0);
		}
		break;
#endif
	}
}

static void send_state(int64_t now)
{
	struct board_state state = {
		/* Reported released only once thrust commands are obeyed */
		.killed = !arming_is_armed(),
		.uptime_ms = (uint32_t)now,
		.command_timeouts = arming_command_timeouts(),
	};

	/* Left 0 and flagged invalid, rather than made up, if the sensor cannot be read */
	state.depth_valid = pressure_get_depth_m(&state.depth_m) == 0;
	protocol_send_state(&state);
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
		LOG_ERR("No kill switch");
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
	arming_init();

	struct host_msg msg;
	int64_t next_state = k_uptime_get();

	while (true) {
		const int64_t now = k_uptime_get();

		watchdog_feed();

		arming_update(killswitch_update(now) == KILLSWITCH_RELEASED, now);

		while (protocol_receive(&msg) == 0) {
			handle_msg(&msg, now);
		}

		if (now >= next_state) {
			next_state = now + STATE_PERIOD_MS;
			send_state(now);
		}

		/*
		 * Yield so lower-priority threads (log thread) get CPU time. Without this, the busy
		 * loop starves them.
		 */
		k_sleep(K_MSEC(1));
	}

	return 0;
}
