#include "killswitch.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <stdbool.h>

LOG_MODULE_REGISTER(killswitch, LOG_LEVEL_DBG);

static const struct gpio_dt_spec killswitch = GPIO_DT_SPEC_GET(DT_NODELABEL(killswitch), gpios);
/* Killed reading counts after it has held this long [ms] */
static const int64_t KILLSWITCH_KILL_MS = DT_PROP(DT_NODELABEL(killswitch), kill_debounce_ms);
/* Released reading counts after it has held this long [ms] */
static const int64_t KILLSWITCH_RELEASE_MS = DT_PROP(DT_NODELABEL(killswitch), release_debounce_ms);

static struct {
	enum killswitch_state state;
	bool sampled;
	bool killed;   /* last raw reading */
	int64_t since; /* time that reading was first seen [ms] */
} debounce;

int killswitch_init(void)
{
	debounce.state = KILLSWITCH_NEEDS_CYCLE;
	debounce.sampled = false;

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
	const bool killed = gpio_pin_get_dt(&killswitch) != 0;

	if (!debounce.sampled || killed != debounce.killed) {
		debounce.sampled = true;
		debounce.killed = killed;
		debounce.since = now;
	}

	const int64_t held = now - debounce.since;

	if (debounce.killed && held >= KILLSWITCH_KILL_MS) {
		debounce.state = KILLSWITCH_KILLED;
	} else if (!debounce.killed && held >= KILLSWITCH_RELEASE_MS &&
		   debounce.state == KILLSWITCH_KILLED) {
		/*
		 * Require that killswitch is killed before it can be released, to avoid accidental
		 * release on power-up
		 */
		debounce.state = KILLSWITCH_RELEASED;
	}

	return debounce.state;
}
