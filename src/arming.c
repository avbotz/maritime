#include "arming.h"

#include "pressure.h"
#include "thruster.h"

/* Thrusters go neutral if no thrust command arrives for this long [ms] */
static const int64_t COMMAND_TIMEOUT_MS = 500;
/*
 * Time between the kill switch powering up the ESCs and thrust commands being obeyed [ms].
 * The ESCs must receive a 1500 us pulse for this long so they can initialize properly.
 */
static const int64_t ESC_INIT_MS = 750;
/* Depth is re-zeroed on arming only within this of the surface [m] */
static const float REZERO_LIMIT_M = 0.5f;

enum arming_state {
	/* Killed, or not yet cycled since startup */
	ARMING_KILLED,
	/* Released. The ESCs are powering up and get neutral until esc_ready_at */
	ARMING_ESC_INIT,
	/* Thrust commands are obeyed, but none have been sent yet or the last timed out */
	ARMING_ARMED,
	/* Driving on the thrust command that arrived at last_command */
	ARMING_DRIVING,
};

static enum arming_state state;
static int64_t esc_ready_at;
static int64_t last_command;
static uint32_t command_timeouts;

static void enter(enum arming_state next)
{
	if (next != ARMING_DRIVING) {
		thruster_stop();
	}
	state = next;
}

void arming_init(void)
{
	enter(ARMING_KILLED);
}

void arming_update(bool released, int64_t now)
{
	if (!released) {
		if (state != ARMING_KILLED) {
			enter(ARMING_KILLED);
		}
		return;
	}

	switch (state) {
	case ARMING_KILLED:
		esc_ready_at = now + ESC_INIT_MS;
		enter(ARMING_ESC_INIT);
		break;
	case ARMING_ESC_INIT:
		if (now >= esc_ready_at) {
			pressure_rezero_near_surface(REZERO_LIMIT_M);
			enter(ARMING_ARMED);
		}
		break;
	case ARMING_ARMED:
		break;
	case ARMING_DRIVING:
		if (now - last_command > COMMAND_TIMEOUT_MS) {
			++command_timeouts;
			enter(ARMING_ARMED);
		}
		break;
	}
}

void arming_command(const float thrusts[NUM_THRUSTERS], int64_t now)
{
	if (!arming_is_armed()) {
		return;
	}
	if (state != ARMING_DRIVING) {
		enter(ARMING_DRIVING);
	}
	thruster_send_all(thrusts);
	last_command = now;
}

bool arming_is_armed(void)
{
	return state == ARMING_ARMED || state == ARMING_DRIVING;
}

uint32_t arming_command_timeouts(void)
{
	return command_timeouts;
}
