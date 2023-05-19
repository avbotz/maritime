#ifndef _MARITIME_SERVO_H
#define _MARITIME_SERVO_H

void init_servos();
void drop(int idx, int value);
void grab(int value);
void shoot(int idx, int value);

#endif /* SERVO_H */
