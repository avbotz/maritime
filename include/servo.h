#ifndef _MARITIME_SERVO_H
#define _MARITIME_SERVO_H

#define HAVE_GRABBER DT_NODE_EXISTS(DT_NODELABEL(grabber_servo))

enum servo_type {
	SERVO_DROPPER,
	SERVO_SHOOTER,
#ifdef HAVE_GRABBER
	SERVO_GRABBER,
#endif
};

int setup_servos(void);
int set_pulse(enum servo_type type, int pulse_us);
void drop(int idx, int value);
void shoot(int idx, int value);
#ifdef HAVE_GRABBER
void grab(int idx, int value);
#endif

#endif /* _MARITIME_SERVO_H */
