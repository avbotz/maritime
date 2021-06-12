#ifndef _PUBSUB_TEST_H
#define _PUBSUB_TEST_H

#include <pubsub/pubsub.h>

struct test_msg_s {
	int val1;
	int val2;
};

extern struct pubsub_topic_s test_topic;

#endif /* _PUBSUB_TEST_H */
