#ifndef _MARITIME_THRUSTER_H
#define _MARITIME_THRUSTER_H

int setup_thrusters(void);
void send_thrusts(float thrusts[8]);
void send_thrust(int thruster, float thrust);

#endif
