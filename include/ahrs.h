#include <zephyr.h>
#include <device.h>

#ifndef AHRS_H
#define AHRS_H

struct imu_sample {
	int64_t timestamp;
  double accel[3];
	double gyro[3];
  double temperature;
};

struct magnetometer_sample {
	int64_t timestamp;
  double magn[3];
};

void init_ahrs(struct k_msgq *ahrs_msgq, struct k_msgq *attitude_msgq);
int process_ahrs(const struct device *dev, struct ahrs_sample *ahrs_sample);
extern void ahrs_poll_thread_entry(void *, void *, void *);

#endif /* AHRS_H */
