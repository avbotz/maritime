# Sensors (STM32 Nucleo-F767ZI)
Documentation of each sensor in Maritime (our low level control stack), interfaced using Zephyr Project's RTOS.

## General Starters for interfacing with a sensor
* Each sensor should be defined in the overlay file ({board-name}.overlay)
inside maritime/boards. The overlay goes on top of the [predefined device tree](https://github.com/zephyrproject-rtos/zephyr/blob/main/boards/arm/nucleo_f767zi/nucleo_f767zi.dts).
With necessary flags that its device initialization would need.
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
    and communicate it through gpios (General Purpose I/O). By initializing the killswitch device in the
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


## DVL
* The DVL, or [Doppler Velocity Log](https://www.blueyerobotics.com/blog/how-a-dvl-simplifies-rov-navigation-and-maneuvering), gives us the linear velocity of the submarine in xyz coordinates. We then convert this to North-East-Down (NED) coordinates in the control loop, which is sent to sub_control.

* We use the [Waterlinked A-50 DVL](https://waterlinked.github.io/dvl/dvl-a50/). Its [Baud Rate](https://en.wikipedia.org/wiki/Baud) is 115200.

* The DVL sends data to the microcontroller (currently [STM32 Nucleo H743ZI](https://docs.zephyrproject.org/latest/boards/arm/nucleo_h743zi/doc/index.html)) through UART.

* The serial protocol for the A-50 can be found [here](https://waterlinked.github.io/dvl/dvl-protocol/#serial-protocol). We are primarily interested in the velocity reports, which are sent as a string, one character at a time, in the following format:
```
wrz,[vx],[vy],[vz],[valid],[altitude],[fom],[covariance],[time_of_validity],[time_of_transmission],[time],[status]
```

## DVL Switch

The problem with the Waterlinked A-50 DVL is that on startup, it sends a 10 microsecond ~5 V spike to the microcontroller's pin, which can brick our pin. To fix this, we have a DVL relay switch, which by default disconnects the microcontroller to the DVL's UART wires. When the sub is unkilled, we assume the spike has passed and we trigger the relay to connect the UART lines. When the sub is killed, we tell the relay to disconnect the lines for safety.

The relay is connected to a GPIO pin on the microcontroller. To trigger the relay, we set that pin to a HIGH state, where the pin sends out a voltage. This causes the relay to connect the DVL UART lines. To turn off the rleay, we set that pin to a LOW state, where the pin does not send voltage. This causes the relay to disconnect the DVL UART lines.

## AHRS
* The AHRS, or Attitude and Heading Reference System, consists of a 6-axis IMU and a 3-axis magnetometer. The IMU returns angular velocity and angular acceleration across all 3 axes, and the magnetometer returns the attitude (yaw, pitch, roll).

* We are interested in angular velocity and attitude.

* We use the [Naviguider AHRS](https://www.pnicorp.com/naviguider-modules/), which communicates with the microcontroller over UART.

## Pressure Sensor
* We use a [Blue Robotics Bar30](https://bluerobotics.com/store/sensors-sonars-cameras/sensors/bar30-sensor-r1/) pressure sensor.

* We use the pressure sensor to find the depth of the submarine.

* The pressure sensor communicates with the microcontroller over the I2C protocol, so we assign it [a pin that is capable of using I2C](https://danieleff.github.io/STM32GENERIC/board_Nucleo_F767ZI/).

## Servos
* We have three servos on the sub: for the grabber, dropper, and shooter.

* We control the servos with sinusoidal control, meaning we use [PWM](https://www.electronics-tutorials.ws/blog/pulse-width-modulation.html).

* Grabber: servo has two modes

* Dropper: servo has 3 modes: positive, negative, & neutral

* Shooter: servo has 3 modes: positive, negative, & neutral

* There are certain pre-defined PWM pins that can be found on the [nucleo's pinout](https://os.mbed.com/platforms/ST-Nucleo-F767ZI/). Each pin has a specific timer and channel that we need to specify in the overlay. We get them from [here](https://github.com/micropython/micropython/blob/master/ports/stm32/boards/stm32f767_af.csv).

## Thrusters/ESCS

We send commands to 8 electronic speed controllers (ESCs), which then spin the thrusters. We use the CAN protocol to send these commands.

First, the escs must be configured using the VESC software (ask Kush how to do it). 


## Killswitch
Refer to the General Starters section for a general look on interfacing the killswitch.
