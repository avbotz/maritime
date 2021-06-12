#include <string.h>
#include <stdbool.h>

#include <sys/slist.h>
#include <kernel.h>

#include <pubsub/pubsub.h>

void pubsub_topic_init(struct pubsub_topic_s *topic, size_t size)
{
	topic->init = false;
	
	sys_slist_init(&(topic->subscribers));
	for (size_t i = 0;i < MAX_CHANNELS;i++) {
		topic->message[i].size = size;
		topic->message[i].data = k_malloc(size);
	}

	topic->init = true;
}

void pubsub_subscriber_register(struct pubsub_topic_s *topic,
		struct pubsub_subscriber_s *subscriber, size_t channel)
{
	subscriber->topic = topic;
	subscriber->updated = false;
	subscriber->channel = channel;

	sys_slist_append(&(topic->subscribers), &(subscriber->node));
}

void pubsub_subscriber_notify(struct pubsub_subscriber_s *subscriber)
{
	subscriber->updated = true;
}

void pubsub_publish(struct pubsub_topic_s *topic, size_t channel, void *data, size_t size)
{
	memcpy(topic->message[channel].data, data, size);

	struct pubsub_subscriber_s *sub;
	struct pubsub_subscriber_s *s_sub;
	
	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&(topic->subscribers), sub, s_sub, node) {
		if (sub->channel == channel) {
			pubsub_subscriber_notify(sub);
		}
	}
}

bool pubsub_subscriber_updated(struct pubsub_subscriber_s *subscriber)
{
	return subscriber->updated;
}

void *pubsub_receive(struct pubsub_subscriber_s *subscriber)
{
	if (!subscriber->topic || !(subscriber->topic->init)) return NULL;

	subscriber->updated = false;

	return subscriber->topic->message[subscriber->channel].data;
}
