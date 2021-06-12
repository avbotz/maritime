# Maritime
Welcome to maritime! This is the new, experimental low level control stack
for the AVBotz submarine, Nemo. Once completed, it will replace Nautical in
driving the sub.

## What's the name?
![why-maritime-name](docs/why-maritime-name.png)

## Why maritime?
Maritime was designed from the ground up to solve many deficiencies with the current
Nautical system. These include:
* Limited computing power on the Atmega 2560 (clock, flash/ram, no FPU)
* Constrained to AVR devices only
* No threading capabilities
* Lack of a robust communication protocol between nautical and a PC
* Lack of robust sensor filtering and estimation
* Lack of robust attitude and velocity control in 6 DoF

Maritime addresses the above deficiences as well as makes further improvements:
* Switched hardware from Arduino Mega2560 to STM32G4/G0/H7 based boards
* Uses Zephyr RTOS (for hardware abstraction as well as multithreading)
* MAVLink communication protocol between maritime and the PC
* Improved sensor filtering and state estimation (Kalman filter planned)
* 6 DoF control
* Cascaded P/PID control architecture.
* (Hopefully) better simulation capabilities
* Runtime parameter configuration
* Distributed across multiple MCUs that each handle a certain part of the stack (motor control, estimation/AHRS, DVL)
* Dual CAN-FD for robust and redudant bus communication between the MCUs
* Additional improvements are planned. Stay tuned!
