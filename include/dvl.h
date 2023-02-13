#include <zephyr.h>
#include <device.h>

#ifndef DVL_H
#define DVL_H

#define PRESSURE_SENSOR_NODE DT_ALIAS(dvl)

struct dvl_sample {
	int64_t timestamp;
  double changeX;
  double changeY;
  double changeZ;
};

void init_dvl(struct k_msgq *dvl_msgq, struct k_msgq *attitude_msgq);
int process_dvl(const struct device *dev, struct dvl_sample *dvl_sample);

#endif /* DVL_H */
