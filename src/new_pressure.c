#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <math.h>

#include <zephyr/logging/log.h>
#include "ads1115.h"

ADS1115 ads1115;

LOG_MODULE_REGISTER(main);

const struct device *const dev = DEVICE_DT_GET_ANY(ti_ads1115);


void init_pressure()
{
    
    if(!device_is_ready(dev.bus)){
        printk("pressure sensor not ready, aborting");
        return;
    }
    

}

float pressure_get_depth()
{

    // burst writing 
    i2c_burst_write(dev, 0x1E, 0x00, datas, 1);

    // burst reading
    i2c_burst_reading(dev, 0x1E, 0x03, datas, 1);

    i2c_reg_read_byte(dev, 0x48, )




	return depth;
}
