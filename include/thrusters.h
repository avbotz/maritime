#ifndef THRUSTERS_H
#define THRUSTERS_H

void init_thrusters();
void send_thrusts(float *thrusts);

static const int NUM_THRUSTERS = 8;

#endif /* THRUSTERS_H */
