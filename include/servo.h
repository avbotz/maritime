#ifndef _MARITIME_SERVO_H
#define _MARITIME_SERVO_H

#include <stdint.h>

enum servo_type {
	SERVO_DROPPER,
	SERVO_GRABBER,
	SERVO_SHOOTER,
};

void setup_servos(void);
void set_pulse(enum servo_type type, uint32_t value);
void drop(int idx, int value);
// void grab(float value);
void shoot(int idx, int value);

#endif /* _MARITIME_SERVO_H */
