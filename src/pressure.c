/* @author: Kush Nayak */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <math.h>

#include "maritime/ads1115.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

/* 
 * https://www.hackster.io/mahmood-ul-hassan/how-to-interface-nordic-thingy-53-with-ads1115-external-adc-caf14a 
 * This tutorial is for the ads1115, but hopefully it works for our ads1015
 */

ADS1115 ads1115;

void init_pressure()
{
	ADS1115_init(&ads1115);
	// const struct adc_channel_cfg channel_cfg = {
	//     .gain = ADC_GAIN_1,
	//     .reference = ADC_REF_INTERNAL,
	//     .acquisition_time = ADC_ACQ_TIME_DEFAULT,
	//     .channel_id = 0,
	//     .differential = 0,
	// };

	// adc_channel_setup(pressure_adc, &channel_cfg);
	// // if (!device_is_ready(pressure_adc))
	// // {
	// // 	printk("Pressure ADC's I2C bus %s is not ready!\n\r");
	// // 	return;
	// // }
}

float pressure_get_depth()
{
	printk("ADC_0: %0.2f | ADC_1: %0.2f | ADC_2: %0.2f | ADC_3: %0.2f\n", \ 
	ADS1115_readADC(&ads1115, CH_0), \
	ADS1115_readADC(&ads1115, CH_1), \
	ADS1115_readADC(&ads1115, CH_2), \
	ADS1115_readADC(&ads1115, CH_3));  	
	return 0;

	// float adc_value = ADS1115_readADC(&ads1115, CH_0);
	// float depth = (adc_value - 893) / 260.; // find this out thru collecting samples and doing linear regression
    // return depth;
}

int main(void)
{
	init_pressure();
	while (1)
	{
		pressure_get_depth();
		// printk("%d\n", pressure_get_depth());
		k_sleep(K_MSEC(1000));
	}
}
