#include "killswitch.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <stdbool.h>

LOG_MODULE_REGISTER(killswitch, LOG_LEVEL_DBG);

static const struct gpio_dt_spec killswitch =
	GPIO_DT_SPEC_GET(DT_NODELABEL(killswitch_button), gpios);

void setup_killswitch()
{
	int ret = gpio_pin_configure_dt(&killswitch, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Failed to configure killswitch: %d", ret);
	}
}

bool alive()
{
	return gpio_pin_get_dt(&killswitch) == 0;
}
