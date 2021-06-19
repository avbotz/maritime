#ifndef _PUBSUB_THRUSTER_OUTPUT_H
#define _PUBSUB_THRUSTER_OUTPUT_H

#include <pubsub/pubsub.h>

struct thruster_output_s {
	int64_t timestamp;

	float power[8];
};

extern struct pubsub_topic_s thruster_output_topic;

#endif /* _PUBSUB_TEST_H */
