#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <string.h>

#include "killswitch.h"
#include "servo.h"
#include "thruster.h"
#include "util.h"
#include "usb.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define MSG_SIZE 256
#define RESPONSE_SIZE 128

static bool has_args(const char *args, int count)
{
	bool in_token = false;
	int found = 0;

	while (*args != '\0') {
		if (*args == ' ') {
			in_token = false;
		} else if (!in_token) {
			in_token = true;
			found++;
			if (found >= count) {
				return true;
			}
		}
		args++;
	}

	return false;
}

static void handle_command(char *msg, char *response, size_t response_size)
{
	char *save_ptr;
	char *token = strtok_r(msg, " ", &save_ptr);

	if (!token) {
		snprintf(response, response_size, "error empty command\n");
		return;
	}

	char c = token[0];

	if (c == 'a') {
		snprintf(response, response_size, alive() ? "alive\n" : "not alive\n");
	} else if (c == 'p') {
		snprintf(response, response_size, "pong\n");
	} else if (c == 't') {
		if (!has_args(save_ptr, 2)) {
			snprintf(response, response_size,
				 "error thrust command needs idx and value\n");
			return;
		}

		int idx = parse_int(" ", &save_ptr);
		float value = parse_float(" ", &save_ptr);
		int ret = send_thrusts(idx, value);

		if (ret < 0) {
			snprintf(response, response_size,
				 "error set thruster failed\n");
			return;
		}

		snprintf(response, response_size, "set thruster\n");
	} else if (c == 'g') {
		if (!has_args(save_ptr, 1)) {
			snprintf(response, response_size,
				 "error grab command needs value\n");
			return;
		}

		float value = parse_float(" ", &save_ptr);

		grab(value);
		snprintf(response, response_size, "set grabber\n");
	} else if (c == 'd') {
		if (!has_args(save_ptr, 1)) {
			snprintf(response, response_size,
				 "error drop command needs value\n");
			return;
		}

		int value = parse_int(" ", &save_ptr);

		drop(value);
		snprintf(response, response_size, "set dropper\n");
	} else if (c == 'o') {
		if (!has_args(save_ptr, 2)) {
			snprintf(response, response_size,
				 "error torpedo command needs id and value\n");
			return;
		}

		int idx = parse_int(" ", &save_ptr);
		int value = parse_int(" ", &save_ptr);

		shoot(idx, value);
		snprintf(response, response_size, "set torpedo\n");
	} else {
		LOG_WRN("Unknown command: %c", c);
		snprintf(response, response_size, "error unknown command\n");
	}
}

int main(void)
{
	if (setup_usb() != 0) {
		LOG_ERR("USB setup failed");
		return -1;
	}

	setup_killswitch();
	// setup_ahrs();
	setup_thrusters();
	setup_servos();

	char msg[MSG_SIZE];
	char response[RESPONSE_SIZE];

	while (true) {
		if (usb_recv_ecm_packet(msg, sizeof(msg)) == 0) {
			handle_command(msg, response, sizeof(response));
			(void)usb_send_ecm_packet(response);
		} else {
			(void)usb_send_ecm_packet("error command too long\n");
		}
	}

	return 0;
}
