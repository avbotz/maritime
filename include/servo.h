#ifndef MARITIME_SERVO_H_
#define MARITIME_SERVO_H_

#include <zephyr/devicetree.h>

#define HAVE_GRABBER DT_NODE_EXISTS(DT_NODELABEL(grabber_servo))

enum servo_type {
	SERVO_DROPPER,
	SERVO_SHOOTER,
#if HAVE_GRABBER
	SERVO_GRABBER,
#endif
};

int servo_init(void);
int servo_set_pulse(enum servo_type type, int pulse_us);
void servo_drop(int idx, int value);
void servo_shoot(int idx, int value);
#if HAVE_GRABBER
void servo_grab(float closed);
#endif

#endif /* MARITIME_SERVO_H_ */
