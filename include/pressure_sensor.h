#include <zephyr.h>
#include <device.h>

#ifndef PRESSURE_H
#define PRESSURE_H

struct pressure_sensor_sample {
	int64_t timestamp;
  double pressureKPA;
};

void init_pressure_sensor(struct k_msgq *pressure_sensor_msgq, struct k_msgq *attitude_msgq);
int process_pressure_sensor(const struct device *dev, struct pressure_sensor_sample *pressure_sensor_sample);
extern void pressure_sensor_poll_thread_entry(void *, void *, void *);

#endif /* PRESSURE_H */
