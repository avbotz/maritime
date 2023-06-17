#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#define THRUSTER_SEND_STACK 256

const struct device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(can1));

struct thruster_data
{
	float thrusts[8];
};

K_MSGQ_DEFINE(to_send_can_msgq, sizeof(struct thruster_data), 3, 4);

void send_thrusts_over_can()
{
	/* 
	 * Send the thrust values to each thruster over CAN 
	 * Expects an array of 8 elements, values from -1 to 1 
	 * of desired thruster output.
	 */

	struct thruster_data data_to_send;
	while (1)
	{
        if (k_msgq_get(&to_send_can_msgq, &data_to_send, K_NO_WAIT) == 0)
        {
        	// Received array of thrusts from the main thread
        }
		// Send thrust msg to each thruster esc individually
		for (uint8_t i = 0; i < 8; i++)
		{
			// Convert the number to be between -8192 to 8192 (esc expects that)
			int16_t thrust = (int16_t) (data_to_send.thrusts[i] * 8192);

			// Check for nan
			if (thrust != thrust)
				continue;

			struct can_frame frame;

			// ESCs are identified as esc 0, esc1, esc2, ... , esc 7
			frame.id = i;
			frame.dlc = sizeof(thrust);  // Data length = size of thrust msg

			// Convert the value into bytes
			uint8_t data[sizeof(thrust)];
			data[0] = thrust & 0xFF; // Lower byte
			data[1] = (thrust >> 8) & 0xFF; // Upper byte

		    memcpy(frame.data, data, sizeof(thrust)); // Set the data payload

		    int ret = can_send(can_dev, &frame, K_NO_WAIT, NULL, NULL);

		    if (ret < 0) {
		        // Error handling
		    } else {
		        // Frame sent successfully
		    }
		}
	}
}

K_THREAD_DEFINE(thruster_send_tid, THRUSTER_SEND_STACK,
            send_thrusts_over_can, NULL, NULL, NULL,
            K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

void send_thrusts(float *thrusts)
{
	// Check for nan, if nan, then don't send
	for (int i = 0; i < sizeof(thrusts)/sizeof(thrusts[0]); i++)
	{
		if (thrusts[i] != thrusts[i])
			return;
	}

	struct thruster_data data_to_send;
	memcpy(data_to_send.thrusts, thrusts, sizeof(data_to_send.thrusts));

    k_msgq_put(&to_send_can_msgq, &data_to_send, K_NO_WAIT);
}

void init_thrusters()
{
	int ret = can_start(can_dev);
	k_thread_start(thruster_send_tid);
	if (ret != 0) {
		// printk("Error starting CAN controller [%d]", ret);
		return;
	}
}