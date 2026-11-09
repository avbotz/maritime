#include "watchdog.h"

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/watchdog.h>

LOG_MODULE_REGISTER(watchdog, LOG_LEVEL_DBG);

/* The hardware watchdog timeout [ms] */
static const uint32_t WATCHDOG_TIMEOUT_MS = 1000;

static const struct device *wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));

static int watchdog_channel = -1;

int watchdog_init(void)
{
	if (!device_is_ready(wdt)) {
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

	watchdog_channel = channel;

	return 0;
}

void watchdog_feed(void)
{
	if (watchdog_channel >= 0) {
		wdt_feed(wdt, watchdog_channel);
	}
}
