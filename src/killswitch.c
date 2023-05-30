#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>

static const struct gpio_dt_spec killswitch = 
    GPIO_DT_SPEC_GET(DT_NODELABEL(killswitch_button), gpios);

bool alive()
{
    uint8_t current_state = gpio_pin_get_dt(&killswitch);
    // SUB is ALIVE when state is 1
    return current_state == 1;
}