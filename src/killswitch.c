#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define KILLSWITCH_NODE DT_NODELABEL(killswitch)

static const struct gpio_dt_spec ks = GPIO_DT_SPEC_GET(KILLSWITCH_NODE, gpios);

bool alive()
{
    uint8_t current_state = gpio_pin_get_dt(ks);

    // SUB is ALIVE when state is 1
    return current_state == 1;
}
