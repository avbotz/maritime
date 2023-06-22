/* @author: Kush Nayak */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <math.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

struct sensor_value oversampling_rate = { 8192, 0 };
const struct device *const dev = DEVICE_DT_GET_ANY(meas_ms5837);

static const struct gpio_dt_spec pressure_adc = 
    I2C_DT_SPEC_GET(DT_NODELABEL(external_adc));

void init_pressure()
{
	if (!device_is_ready(pressure_adc))
	{
		printk("I2C bus %s is not ready!\n\r",pressure_adc.bus->name);
		return;
	}
}

bool pressure_get_depth()
{
	// Get data from i2c
	uint8_t buffer[2];
	ret = i2c_read_dt(&pressure_adc, buffer, sizeof(buffer));
	if(ret != 0){
		printk("Failed to read from I2C device address %x at Reg. %x n", pressure_adc->addr,config[0]);
	}

    uint16_t adc_value = (buffer[0] << 8) | buffer[1];
    float depth = (adc_value - 893) / 260.;
    return depth;
}