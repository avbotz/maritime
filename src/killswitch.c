#include <zephyr.h>
#include <device.h>

#define KILLSWITCH_NODE DT_NODELABEL(ks)
#define KILLSWITCH_GPIO DT_GPIO_LABEL(KILLSWITCH_NODE, gpios)

static const struct device *get_KILLSWITCH_device(void) {
    
    const struct device *const dev = device_get_binding(KILLSWITCH_GPIO);
    return dev;
}
