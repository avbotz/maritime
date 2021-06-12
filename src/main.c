#include <stdio.h>

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>

#define LED_PORT "GPIOA"
#define LED_PIN 5

void main(void)
{
	return;
	printf("Hello, world! %s\n", CONFIG_BOARD);

	const struct device *gpio_port_dev;
	int ret;

	gpio_port_dev = device_get_binding(LED_PORT);
	if (gpio_port_dev == NULL) {
		printf("Unable to initialize device %s\n", LED_PORT);
		return;
	}

	printf("Initialized GPIO device %s\n", LED_PORT);

	ret = gpio_pin_configure(gpio_port_dev, LED_PIN, GPIO_OUTPUT);
	if (ret < 0) {
		printf("Unable to configure %s pin %d: error %d\n", LED_PORT, LED_PIN, ret);
		return;
	}

	while (1)
	{
		printf("Toggling!\n");
		gpio_pin_toggle(gpio_port_dev, LED_PIN);
		k_msleep(1000);
	}
}
