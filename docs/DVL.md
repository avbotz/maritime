The DVL, or Doppler Velocity Log, gives us the linear velocity of the submarine in North-East-Down coordinates.

We use the [Waterlinked A-50 DVL](https://waterlinked.github.io/dvl/dvl-a50/).

The DVL sends the data to our microcontroller (currently [STM32 Nucleo F767ZI](https://docs.zephyrproject.org/latest/boards/arm/nucleo_f767zi/doc/index.html)) through UART. The microcontroller, which runs maritime, sends the data to the Jetson, running sub_control.

The serial protocol for the A-50 can be found [here](https://waterlinked.github.io/dvl/dvl-protocol/#serial-protocol).
