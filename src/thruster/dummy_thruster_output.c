#include <stdio.h>
#include <math.h>
#include <string.h>

#include <zephyr.h>

#include <pubsub/pubsub.h>
#include <pubsub/thruster_output.h>

#include "thruster.h"

#define M_PI 3.14159265358979323846

#define DUMMY_THRUSTER_OUTPUT_STACK_SIZE 1024
#define DUMMY_THRUSTER_OUTPUT_PRIORITY 1

PUBSUB_TOPIC_DEFINE(thruster_output_topic, sizeof(struct thruster_output_s));

static void dummy_thruster_output_entry_point(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	int64_t clk = k_ticks_to_us_near64(k_uptime_ticks());
	float sin_clk = 0;

	while (1) {
		struct thruster_output_s thruster_output;
		memset(&thruster_output, 0, sizeof(struct thruster_output_s));

		/* TODO: true microsecond timestamping */
		thruster_output.timestamp = clk;

		float thrust = 0.15 + 0.1 * sin(2 * M_PI * 0.2 * sin_clk);
		for (int i = 0;i < NUM_THRUSTERS;i++) {
			thruster_output.power[i] = thrust;
		}

		pubsub_publish(&thruster_output_topic, 0, &thruster_output);

		clk += 5000; /* 200Hz */
		sin_clk += 0.02;
		k_sleep(K_TIMEOUT_ABS_US(clk));
	}
}

/*K_THREAD_DEFINE(dummy_thruster_output_thread_id, DUMMY_THRUSTER_OUTPUT_STACK_SIZE,*/
				/*dummy_thruster_output_entry_point, NULL, NULL, NULL,*/
				/*DUMMY_THRUSTER_OUTPUT_PRIORITY, 0, 0);*/
