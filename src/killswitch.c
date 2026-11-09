#include "killswitch.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <stdbool.h>

LOG_MODULE_REGISTER(killswitch, LOG_LEVEL_DBG);

/* Killed reading counts once it has held this long [ms] */
static const uint32_t KILLSWITCH_KILL_MS = 20;
/* Released reading counts once it has held this long [ms] */
static const uint32_t KILLSWITCH_RELEASE_MS = 50;

/* Pulled up and active high */
static const struct gpio_dt_spec killswitch = GPIO_DT_SPEC_GET(DT_NODELABEL(killswitch), gpios);

/* The last reading, and the time of the first call that saw it [ms] */
static bool sampled;
static bool raw_alive;
static int64_t raw_since;
/* A killed reading has counted since setup */
static bool seen_killed;
static bool released;

int setup_killswitch(void)
{
	sampled = false;
	seen_killed = false;
	released = false;

	if (!gpio_is_ready_dt(&killswitch)) {
		LOG_ERR("Kill switch GPIO not ready");
		return -ENODEV;
	}
	int ret = gpio_pin_configure_dt(&killswitch, GPIO_INPUT);

	if (ret != 0) {
		LOG_ERR("Failed to configure kill switch: %d", ret);
		return ret;
	}
	return 0;
}

enum killswitch_state killswitch_update(int64_t now)
{
	/* A read error is killed too */
	const bool reading = gpio_pin_get_dt(&killswitch) == 0;

	if (!sampled || reading != raw_alive) {
		sampled = true;
		raw_alive = reading;
		raw_since = now;
	}
	const int64_t held = now - raw_since;

	if (!raw_alive && held >= KILLSWITCH_KILL_MS) {
		seen_killed = true;
		released = false;
	} else if (raw_alive && held >= KILLSWITCH_RELEASE_MS && seen_killed) {
		released = true;
	}

	if (released) {
		return KILLSWITCH_RELEASED;
	}
	return seen_killed ? KILLSWITCH_KILLED : KILLSWITCH_NEEDS_CYCLE;
}
