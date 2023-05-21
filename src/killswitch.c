#include <gpio.h>
#include <zephyr/devicetree.h>

#define KILLSWITCH_NODE DT_NODELABEL(ks)

static const struct gpio_dt_spec *get_KILLSWITCH_device(void){
    
    static const struct gpio_dt_spec ks = GPIO_DT_SPEC_GET(KILLSWITCH_NODE, gpios);
    return ks;
}

