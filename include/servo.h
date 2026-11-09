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

int servo_init(void);
int servo_set_pulse(enum servo_type type, int pulse_us);
void servo_drop(int idx, int value);
void servo_shoot(int idx, int value);
#ifdef HAVE_GRABBER
void servo_grab(int idx, int value);
#endif

#endif /* _MARITIME_SERVO_H */
