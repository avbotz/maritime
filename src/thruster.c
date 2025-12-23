#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include "thruster.h"

int setup_thrusters(void) {};
void send_thrusts(float thrusts[8]) {};
