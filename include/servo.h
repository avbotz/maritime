#ifndef _MARITIME_SERVO_H
#define _MARITIME_SERVO_H

enum servo_type {
	SERVO_DROPPER,
	SERVO_GRABBER,
	SERVO_SHOOTER,
};

void setup_servos(void);
int set_pulse(enum servo_type type, int pulse_us);
void drop(int idx, int value);
// void grab(float value);
void shoot(int idx, int value);

#endif /* _MARITIME_SERVO_H */
