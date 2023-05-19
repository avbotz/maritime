#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

#define KILLSWITCH_NODE DT_NODELABEL(ks)
#define KILLSWITCH_GPIO DT_GPIO_LABEL(KILLSWITCH_NODE, gpios)

static const struct device *get_KILLSWITCH_device(void) {
    
    const struct device *const dev = device_get_binding(KILLSWITCH_GPIO);
    return dev;
}

bool alive()
{
    // Todo: read the kill switch pin to return if we are alive or not
    uint8_t killswitch_pin = DT_GPIO_PIN(DT_NODELABEL(ks), gpios);
    uint8_t current_state = gpio_pin_get(ks, killswitch_pin);
    // SUB is ALIVE when state is 1
    return current_state == 1;
}
