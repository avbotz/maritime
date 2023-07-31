# Maritime
Maritime is the code on our microcontroller, which reads data from sensors and controls the thrusters to move the sub to a specific destination. It uses the North-East-Down coordinate system measured in meters and radians (roll pitch yaw order), which is created when the sub is unkilled. This is different from Boops-Boops, which uses degrees instead of radians (yaw pitch roll order). 

# Sensors (STM32 Nucleo-H743ZI)
Documentation of each sensor in Maritime (our low level control stack), interfaced using Zephyr Project's RTOS.

# Multithreading
For each sensor, some of them send one character/byte at a time through a wire, which eventually builds up to a string. We have callbacks, which are functions that are called whenever a new character arrives, and they can handle that individual character. However, it's unwise to parse the full string inside the callback because the callback should be as fast/minimal as possible so we don't lose incoming characters. 

Because of this, we use multithreading, or having one thread (process) for each sensor that needs it. We parse the full message inside of the thread, so that it doesn't slow down the callback. The callback usually builds up the full string and then sends it to a message queue, where the thread then handles the parsing.  

## General Starters for interfacing with a sensor
* Each sensor should be defined in the overlay file ({board-name}.overlay)
inside maritime/boards. The overlay goes on top of the [predefined device tree](https://github.com/zephyrproject-rtos/zephyr/blob/main/boards/arm/nucleo_f767zi/nucleo_f767zi.dts).
With necessary flags that its device initialization would need.
    For example, for the initialization of the killswitch button, we have defined
    it as a button, with the name label as `killswitch_button`, along with the
    gpios attribute at the specific pin `PG4` with the flag `GPIO_ACTIVE_HIGH` as
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


* Each sensor is initialized with a setup_{sensor} function located in their own
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

* In dvl.c, there is a callback that is called each time a new character arrives to the nucleo. We parse this data by first searching for the first 'wrz' string, then building up the full string one character at a time, until we reach the end of the string, marked by a \r or \n character. Then, we send this string to the dvl thread (process) through a k_msgq (message queue). Once the process receives a new message, it will parse the string, token by token, and it will convert the relevant data strings to floats. Once it finishes parsing the string, it will send the data to another k_msgq, which the loop in the main.c file can access. We use k msg queues because they allow different threads to send data between each other.

## DVL Switch
The problem with the Waterlinked A-50 DVL is that on startup, it sends a 10 microsecond ~5 V spike to the microcontroller's pin, which can brick our pin. To fix this, we have a DVL relay switch, which by default disconnects the microcontroller to the DVL's UART wires. When the sub is unkilled, we assume the spike has passed and we trigger the relay to connect the UART lines. When the sub is killed, we tell the relay to disconnect the lines for safety.

The relay is connected to a GPIO pin on the microcontroller. To trigger the relay, we set that pin to a HIGH state, where the pin sends out a voltage. This causes the relay to connect the DVL UART lines. To turn off the rleay, we set that pin to a LOW state, where the pin does not send voltage. This causes the relay to disconnect the DVL UART lines.

## AHRS
* The AHRS, or Attitude and Heading Reference System, consists of a 6-axis IMU and a 3-axis magnetometer. The IMU returns angular velocity and angular acceleration across all 3 axes, and the magnetometer returns the attitude (yaw, pitch, roll).

* Currently, we are only using the gyroscope and accelerometer on the AHRS because the magnetometer (compass) is heavily affected by electromagnetic interference when we are running the thrusters. 

* We are interested in angular velocity and attitude.

* We use the WitMotion WT901, which communicates with the microcontroller over UART. See their documentation for the format of the strings they send.

* To read data coming in from the AHRS, the process is similar to that of the DVL, except it parses the AHRS string which has a different format. 

* Sometimes, the AHRS' angle sensor will drift. When this happens, boot into the Witmotion windows software:
For the WitMotion, if something is going weird:
- Make sure the board holding the AHRS and the boards holding parts near the AHRS (i.e. the Arduino or the microcontroller) are strapped down so that they do not budge when you push them. We had an issue in 2022 comp where the AHRS board and Arduino boards were not fastened down, leading to the Arduino board smashing the AHRS board, causing jitter that threw off the gyro on the AHRS and caused some drift. We fixed that by fastening the AHRS board by tightening the screws and using hot glue, and fastening the Arduino board by using electrical tape to tape it down so that it didn't move.
- Use WitMotion Windows app (search up how to download it, it's on their website for the WT-901 AHRS)
- Connect the AHRS to computer using FTDI (which is a Serial to USB converter)
- Open their Windows app
- Click Config -> Reset
- Exit Config and reenter Config (for changes to update in the menu)
- Click 6 axis mode to not use magnetometer (because electromagnetic interference makes the reading drift and wobble heavy)
- Use vertical mode (instead of horizontal) for more gyro stability
- Make the output rate 100 Hz so that we get lots of data for the microcontroller to use and make PID adjustments fast to make the sub move smooth in the water (or else the sub might wobble back and forth on the yaw axis.)
- Set the serial baud rate to 115200
- Make sure changes are saved and then exit.

## Pressure Sensor
* We use a Ashkroft K1 pressure sensor. It sends a raw voltage proportional with pressure to an analog to digital pin on the microcontroller. The microcontroller then converts this voltage to a number, which is then converted to depth in meters. 

* Since the analog to digital converter (ADC) gives a unitless integer, we have to manually calibrate to find the mapping between the number and the actual meter depth. To do this, we collected lots of data points and did linear regression to find the relationship (x = integer reading, y = actual meter depth).

## Servos
* We have three servos on the sub: for the grabber, dropper, and shooter.

* We control the servos with [PWM signals](https://www.electronics-tutorials.ws/blog/pulse-width-modulation.html).

* Grabber: servo has an angular range it can go between to move the grabber

* Dropper: servo has 3 modes: positive, negative, & neutral

* Shooter: servo has 3 modes: positive, negative, & neutral

* There are certain pre-defined PWM pins that can be found on the [nucleo's pinout](https://os.mbed.com/platforms/ST-Nucleo-F767ZI/). Each pin has a specific timer and channel that we need to specify in the overlay. We get them from [here](https://github.com/micropython/micropython/blob/master/ports/stm32/boards/stm32f767_af.csv).

## Servo testing and programming
You can test and program a servo (useful for adding overload protection, servo endpoints, etc.) by connecting a servo to the [DPC-11 Programming Interface](https://hitecrcd.com/products/servos/programmers/dpc-11/product), and running its [interfacing software](https://hitecrcd.com/uploads/DPC-11_Install__2020_12_09_01-2.9.9.zip) on a Windows 10/11 computer.

To run the interfacing software properly, you'll need to enable .NET 3.5 on your device. To do this, use the Windows start menu to navigate to "Turn Windows features on and off" and select ".NET Framework 3.5 (includes .NET 2.0 and 3.0)." Then navigate to `DPC-11_Install__2020_12_09_01-2.9.9/DPC-11_2020_12_09_01-2.9.9/` and run `DPC-11_Setup.msi`

To get drivers set up on your computer to properly interface with the DPC-11, you'll need to disable driver signature enforcement. In Windows settings, go to Settings > Recovery > Advanced Startup > Restart Now. During startup, click on Troubleshoot > Advanced Options > Startup Settings > Restart, and then press 7 / F7 to disable driver signature enforcement.

Once your device boots up, navigate to `DPC-11_Install__2020_12_09_01-2.9.9/DPC-11_2020_12_09_01-2.9.9/HITECRCD_DPC-11 Driver Installer` and run `install_DPC-11_2020_12_09_01-2.9.9.exe` to install the drivers. To configure Windows to use the drivers with your DPC-11, plug it in, and then navigate to it on Windows Device Manager. Right click on it and press "Update drivers," then "Browse my computer for drivers," select the `HITECRCD_DPC-11 Driver Installer` folder, and then hit "Next." Once you've completed all this, you'll be able to use the DPC-11 software to interface with your servo!

To use the DPC-11 software, run it with the Windows start menu and, from the app's menu, select the appropriate type for your servo (Botz use BlueTrail's D-Series servos), then plug in the DPC-11 via USB; you should see the rectangle on the top left turn green. Now wire up the servo to the DPC-11 and press the "Connect" button on your computer. To wire it up, you need to use female duponts. You can input around 6V into the "Batt" power port and a ground. Then on the servo port, route the servo's power, ground, and signal. Hopefully all goes well and you can finally start programming/testing your servo in accordance with [the manual](https://hitecrcd.com/images/products/pdf/411_DPC11_ManualV3.pdf).

## Thrusters/ESCS
We send commands to 8 electronic speed controllers (ESCs), which then spin the thrusters. We use the CAN protocol to send these commands.

First, the escs must be configured using the VESC software (ask Kush Nayak how to do it).

They must be assigned IDs that correspond to their location on the sub:
* Vertical front left    - 0
* Vertical front right   - 1
* Vertical back left     - 2
* Vertical back right    - 3
* Horizontal front left  - 4
* Horizontal front right - 5
* Horizontal back left   - 6
* Horizontal back right  - 7

The thruster thread continuously sends the latest set of thrust values to the escs, to ensure that the escs keep spinning the thrusters.

## Killswitch

When the killswitch is flipped, it kills power to the thrusters, killing the sub. When it is unkilled, we are free to run the sub. In main.c, we continuously monitor the status of the killswitch, and when it is unflipped, we begin the loop to send commands to each thruster.

The killswitch connects to a GPIO pin on the nucleo. When the sub is unkilled, there will be no voltage on the pin. When the sub is killed, there will be voltage on the pin. 

Refer to the General Starters section for a general look on interfacing the killswitch.

# Control Loop

Maritime's primary goal is to command the thrusters to move the sub to a coordinate destination (x, y, z, yaw, pitch, roll) that Boops-Boops wants it to move to. It uses a cascade PID controller, where a position controller's output feeds into a velocity controller. 

This is how it does so:
* Read all the sensors to get the current state of the sub. Multiplying DVL's velocity * dt gives us x and y, pressure sensor gives us z, and AHRS gives us yaw, pitch, roll.
* Calculate the error between our current state and desired state on each of the 6 axes.
* Convert that absolute error into relative error on each axis.
* Feed the relative error on each axis into pid controllers, which edit those errors. The edited errors become the velocity (m/s) or angular velocity (rad/s) setpoints we want the sub to move at. Essentially, the farther you are from the target, the faster you wanna go.
* Calculate the error between your current velocity and your desired velocity on each of the 6 axes. These errors are then fed into our velocity pid controllers, which edit those errors. The edited errors then become our force/torque setpoints, one for each of the 6 axes. Essentially, the farther away you are from your desired velocity, the more force you wanna apply. 
* These force and torque setpoints are values from -1 to 1, where 1 = 100% thruster power. These values are then clamped to be in the range of [-power, power], where power is the proportion of the thruster's power we want to use. Eg. power = 0.6 means the max thruster power we'll use is 60%. Finally, we have a set of forces and torques we want to apply to the sub.
* Map the forces and torques to how much thrust we should put on each thruster. There is some matrix math, but you can simplify it like so: For each thruster, loop over each axis. If there is error on this axis and the thruster can go in this direction, add thrust to this thruster.
* Now, you have an array of length 8, with each value being the thrust output in the range [-power, power], one value per thruster. The index of each value in the array is the same as the ID of the esc that the command is sent to. Pass this array to the send_thrusts() function, which sends these thrust values to the escs via the CAN protocol. 

## Depth vs Altitude Control

* The sub can either control depth using the pressure sensor, or altitude using the DVL. We default to controlling depth because it is more consistent and does not depend on the shape of the floor. 
* To toggle altitude control: Send "b [desired altitude]" to enable altitude control. Send "b -1" to disable altitude control and go back to depth control.

## PID Controllers

We use PID controllers to edit the raw position/angle error to optimally control the sub. I recommend watching videos to understand this. 

* The proportional gain multiplies the raw error
* The integral gain multiplies the area under the error vs. time curve and adds it to the overall error. This builds up error over time and forces the sub to adjust, avoiding steady state error (where the sub is close to a setpoint but refuses to go all the way)
* The derivative gain multiplies the current slope of the error vs. time curve and adds it to the overall error. If your error is decreasing, this is a negative slope, so you have less response and are less aggressive as you approach the setpoint, allowing for a smooth setpoint approach. On the flip side, if your error is increasing with a disturbance, this is a positive error slope, so this adds more error and more aggression to combat that change. 

- Proportional gain  =  Kp
- Integral term      =  Ti     (More Ti is less integral gain, Ki = 1/Ti)
- Derivative term    =  Td     (Td = Kd)

## Communicating with maritime

When a PC is connected via USB to the microcontroller, the PC can send and receive data to the microcontroller via the serial protocol. Instructions for this are in the main maritime readme.

