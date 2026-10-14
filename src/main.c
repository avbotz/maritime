// Need the following line to include strtok_r, as picolibc does not include it by default
// Seems to build fine though, though I get editor warnings if the following line is not included
#define _POSIX_C_SOURCE 200809L

#include "killswitch.h"
#include "pressure.h"
#include "thruster.h"
#include "servo.h"
#include "util.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef CONFIG_BOARD_RPI_PICO
#include <pico/bootrom.h>
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

#define PROTOCOL_VERSION    2
/* While running, every thruster goes neutral if no thrust command arrives for this long [ms] */
#define COMMAND_TIMEOUT_MS  500
/* Time between kill switch powering up ESCs and thrust commands being sent [ms] */
#define ESC_INIT_MS         750
#define KILL_DEBOUNCE_MS    75
/* Kill switch, depth and heartbeat lines [ms] */
#define STATE_PERIOD_MS     200
/* The hardware watchdog timeout [ms] */
#define WATCHDOG_TIMEOUT_MS 1000
/* Depth is re-zeroed at a kill switch release only within this of the surface [m] */
#define REZERO_LIMIT_M      0.5f

static const struct device *usb_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#define MSG_SIZE 256
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 1);

static char rx_buf[MSG_SIZE];
static size_t rx_pos = 0;

// the pico does not support floating point formatting in printk, so we need to
// convert to integer parts
static void format_float(float value, char *buf, size_t size)
{
	int whole = abs((int)value);
	int frac = abs((int)((value - (int)value) * 1000));

	snprintf(buf, size, "%s%d.%03d", value < 0 ? "-" : "", whole, frac);
}

static void uart_rx_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uint8_t c;

	uart_irq_update(dev);
	while (uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			if (uart_fifo_read(dev, &c, 1) != 1) {
				uart_irq_update(dev);
				continue;
			}

			if (c == '\n' || c == '\r') {
				if (rx_pos > 0) {
					rx_buf[rx_pos] = '\0';
					k_msgq_put(&uart_msgq, rx_buf, K_NO_WAIT);
					rx_pos = 0;
				}
			} else if (rx_pos < MSG_SIZE - 1) {
				rx_buf[rx_pos++] = (char)c;
			} else {
				rx_pos = 0;
			}
		}
		uart_irq_update(dev);
	}
}

static int setup_watchdog(const struct device *wdt)
{
	if (wdt == NULL || !device_is_ready(wdt)) {
		LOG_ERR("Watchdog device not ready");
		return -ENODEV;
	}
	const struct wdt_timeout_cfg cfg = {
		.window = {.min = 0, .max = WATCHDOG_TIMEOUT_MS},
		.callback = NULL,
		.flags = WDT_FLAG_RESET_SOC,
	};
	int channel = wdt_install_timeout(wdt, &cfg);
	if (channel < 0) {
		LOG_ERR("Watchdog install failed: %d", channel);
		return channel;
	}
	int ret = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (ret < 0) {
		LOG_ERR("Watchdog setup failed: %d", ret);
		return ret;
	}
	return channel;
}

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

/* Parses and sends an m, p or a command; false if it is malformed */
static bool send_thrust_command(char c, char **save_ptr)
{
	float value;

	if (c == 'm') {
		float thrusts[NUM_THRUSTERS];
		for (int i = 0; i < NUM_THRUSTERS; i++) {
			if (!parse_float_arg(strtok_r(NULL, " ", save_ptr), &thrusts[i])) {
				return false;
			}
		}
		send_thrusts(thrusts);
	} else if (c == 'p') {
		char *thruster_token = strtok_r(NULL, " ", save_ptr);
		char *thrust_token = strtok_r(NULL, " ", save_ptr);
		int thruster;

		if (!parse_int_arg(thruster_token, &thruster) ||
		    !parse_float_arg(thrust_token, &value)) {
			return false;
		}
		send_thrust(thruster, value);

		LOG_DBG("Setting thruster power: p %d %d\n", thruster, (int)(value * 1000));
	} else {
		if (!parse_float_arg(strtok_r(NULL, " ", save_ptr), &value)) {
			return false;
		}
		send_all_thrusts(value);
	}
	return true;
}

static void handle_command(char *msg, struct thrust_state *thrust, int64_t now)
{
	char *save_ptr;
	char *token = strtok_r(msg, " ", &save_ptr);
	if (!token) {
		return;
	}
	char c = token[0];

	if (c == 'm' || c == 'p' || c == 'a') {
		if (thrust->running && send_thrust_command(c, &save_ptr)) {
			thrust->commanded = true;
			thrust->last_command = now;
		}
	} else if (c == 's') {
		// Usage: s <type> <pulse>
		// type: g (grabber), t (shooter), d (dropper)
		// pulse: microseconds (1000 = 1ms)
		char *type_token = strtok_r(NULL, " ", &save_ptr);

		if (!type_token) {
			return;
		}

		char type = type_token[0];

		char *pulse_token = strtok_r(NULL, " ", &save_ptr);
		int pulse;

		if (!parse_int_arg(pulse_token, &pulse)) {
			return;
		}

		// convert to nanoseconds (1000 = 1ms)
		pulse = pulse * 1000;

		if (type == 'g') {
			set_pulse(SERVO_GRABBER, pulse);
		} else if (type == 't') {
			set_pulse(SERVO_SHOOTER, pulse);
		} else if (type == 'd') {
			set_pulse(SERVO_DROPPER, pulse);
		}
	} else if (c == 't' || c == 'd') {
		char *selector_token = strtok_r(NULL, " ", &save_ptr);
		char *state_token = strtok_r(NULL, " ", &save_ptr);

		if (!selector_token || !state_token) {
			return;
		}

		int selector = selector_token[0] == '1';
		int state = state_token[0] == '1';

		(c == 't' ? shoot : drop)(selector, state);
	}
#ifdef CONFIG_BOARD_RPI_PICO
	else if (c == 'r') {
		stop_thrusters();
		reset_usb_boot(0, 0);
	}
#endif
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
	setup_thrusters();
	setup_servos();
	setup_pressure();

	// Neutral until the kill switch is released and the ESCs have initialized
	stop_thrusters();

	const struct device *wdt = DEVICE_DT_GET_OR_NULL(DT_ALIAS(watchdog0));
	const int wdt_channel = setup_watchdog(wdt);

	char msg[MSG_SIZE];
	struct thrust_state thrust = {0};
	// The kill switch as read, since when, and as debounced. Starts killed, whatever it
	// reads, and a release counts only after the switch has read killed since boot: a
	// board that resets mid-run (brownout, watchdog) stays killed until someone cycles
	// the switch, rather than driving again on its own.
	int64_t now = k_uptime_get();
	bool raw_alive = alive();
	int64_t raw_since = now;
	bool released = false;
	bool seen_killed = false;
	int64_t esc_ready_at = 0;
	int64_t next_state = now;

	while (true) {
		now = k_uptime_get();
		if (wdt_channel >= 0) {
			wdt_feed(wdt, wdt_channel);
		}

		// Kill switch: a new reading counts once it has held for KILL_DEBOUNCE_MS
		bool reading = alive();
		if (reading != raw_alive) {
			raw_alive = reading;
			raw_since = now;
		}
		const bool settled = now - raw_since >= KILL_DEBOUNCE_MS;
		if (settled && !raw_alive) {
			seen_killed = true;
		}
		if (settled && raw_alive != released && seen_killed) {
			released = raw_alive;
			if (released) {
				// The switch powers the ESCs up: they need neutral first
				esc_ready_at = now + ESC_INIT_MS;
			} else {
				thrust.running = false;
				thrust.commanded = false;
				stop_thrusters();
			}
		}
		if (released && !thrust.running && now >= esc_ready_at) {
			thrust.running = true;
			thrust.commanded = false;
			// The surface for this run, unless released underwater
			rezero_near_surface(REZERO_LIMIT_M);
		}

		while (k_msgq_get(&uart_msgq, msg, K_NO_WAIT) == 0) {
			handle_command(msg, &thrust, now);
		}

		// The host stopped commanding (crashed, unplugged): stop rather than keep
		// driving on the last command
		if (thrust.running && thrust.commanded &&
		    now - thrust.last_command > COMMAND_TIMEOUT_MS) {
			stop_thrusters();
			thrust.commanded = false;
			thrust.command_timeouts++;
		}

		if (now >= next_state) {
			next_state = now + STATE_PERIOD_MS;
			if (!thrust.running) {
				stop_thrusters();
			}

			// Reported released only once thrust commands are obeyed
			printk(thrust.running ? "x 0\n" : "x 1\n");

			// No line rather than a made-up depth if the sensor cannot be read
			float depth_m;
			if (get_depth_meters(&depth_m) == 0) {
				char depth_s[24];
				format_float(depth_m, depth_s, sizeof(depth_s));
				printk("d %s\n", depth_s);
			}

			printk("h %d %u %u\n", PROTOCOL_VERSION, (uint32_t)now,
			       thrust.command_timeouts);
		}

		// Yield so lower-priority threads (log thread) get CPU time; without
		// this the busy loop starves them
		k_sleep(K_MSEC(1));
	}

	return 0;
}
