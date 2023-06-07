# Sensors (STM32 Nucleo-F767ZI)
Documentation of each sensor in Maritime (our low level control stack), interfaced using Zephyr Project's RTOS.

## General Starters for interfacing with a sensor
* Each sensor should be defined in the device tree file ({board-name}.overlay)
inside maritime/boards. With necessary flags that its device initialization would
need. 
    For example, for the initialization of the killswitch button, we have defined
    it as a button, with the name label as `killswitch_button`, along with the 
    gpios attribute at the specific pin `PH1` with the flag `GPIO_ACTIVE_HIGH` as
    shown in the code block below, which is within the overlay file.
    ```
    /* Communicate with the killswitch as a gpio "button" on pin PH1 */
        / {
             buttons {
                compatible = "gpio-keys";
                killswitch_button: killswitch_button {
                    /* Pin PH1 -> gpioh 1 */
                    gpios = <&gpioh 1 GPIO_ACTIVE_HIGH>;
                    label = "Killswitch modeled as a button";
                };
            };  
        };

    ```
    This is defined as such because we want to define the killswitch as a button 
    and communicate it through gpios. By initializing the killswitch device in the
    `killswitch.c` file, through the function `GPIO_DT_SPEC_GET(DT_NODELABEL(killswitch_button), gpios);` 
    where we point the device to the `killswitch_button` node in our device tree
    with the specific property that we defined as the variable gpios within its 
    node.
     
    
* Each sensor is initialized with a init_{sensor} function located in their own
{sensor}.c file inside maritime/src/
    * You can generally find samples online or through Zephyr Project's basic
    samples (buttons, servos, etc.) in this [repository](https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/basic)
    * Following the killswitch example, we can see that the killswitch device is 
    initialized with the the following line of code `gpio_pin_configure_dt(&killswitch, GPIO_INPUT);`
        * Where this function initializes the device, pointing to the configured killswitch device from the first function `GPIO_DT_SPEC_GET` and treating it as an input device (See [Zephyr Project GPIO flags](https://docs.zephyrproject.org/latest/hardware/peripherals/gpio.html#c.GPIO_INPUT))
     * With a defined device with correct initialization & configuration, we are able to successfully interface with it.


## DVL (Doppler Velocity Logger)
TODO


## AHRS (Altitude and Heading Reference System)
TODO


## Pressure Sensor 
TODO


## Servos
TODO


## Thrusters  
TODO


## Killswitch 
Refer to the General Starters section for a general look on interfacing the killswitch.
