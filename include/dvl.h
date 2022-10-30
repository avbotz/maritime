#include <zephyr.h>
#include <device.h>

#ifndef DVL_H
#define DVL_H

struct dvl_sample {
	int64_t timestamp;
  double changeX;
  double changeY;
  double changeZ;
};

void init_dvl(struct k_msgq *dvl_msgq, struct k_msgq *attitude_msgq);
int process_dvl(const struct device *dev, struct dvl_sample *dvl_sample);
extern void dvl_poll_thread_entry(void *, void *, void *);

#endif /* DVL_H */
