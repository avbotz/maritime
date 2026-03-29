#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>

#include "killswitch.h"
#include "thruster.h"

static const struct gpio_dt_spec killswitch = GPIO_DT_SPEC_GET(DT_NODELABEL(killswitch_button), gpios);

void setup_killswitch() {
    gpio_pin_configure_dt(&killswitch, GPIO_INPUT);
}

bool alive() {
    int current_state = gpio_pin_get_dt(&killswitch);

    // SUB is ALIVE when state is 0
    bool status = current_state == 0;

    if (!status) {
	    float new_thrusts[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	    send_thrusts(new_thrusts);
    }
    return status;
}
