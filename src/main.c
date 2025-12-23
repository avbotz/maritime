#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/uart.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "ahrs.h"
// #include "thruster.h"
// #include "servo.h"
#include "killswitch.h"
#include "util.h"

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	    "Console device is not ACM CDC UART device");

const struct device *usb_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

int main(void)
{
    struct ahrs_data_s ahrs_m;

    while (true) {
        if (k_msgq_get(&ahrs_data_msgq, &ahrs_m, K_FOREVER) == 0) {
            printf("@%lld ms: Yaw %f, Pitch %f, Roll %f\n",
                   (long long)ahrs_m.ts_us,
                   (double)ahrs_m.yaw,
                   (double)ahrs_m.pitch,
                   (double)ahrs_m.roll);
        }

        printk("Killswitch state: %d\n", alive() ? 1 : 0);

        k_sleep(K_MSEC(10));
    }

    return 0;
}
