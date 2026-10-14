#ifndef _MARITIME_THRUSTER_H
#define _MARITIME_THRUSTER_H

#define NUM_THRUSTERS 8

int setup_thrusters(void);
void send_thrusts(const float thrusts[NUM_THRUSTERS]);
void send_thrust(int thruster, float thrust);
void send_all_thrusts(float thrust);
/* Every thruster to neutral (1500 us) */
void stop_thrusters(void);

#endif /* _MARITIME_THRUSTER_H */
