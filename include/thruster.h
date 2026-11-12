#ifndef MARITIME_THRUSTER_H_
#define MARITIME_THRUSTER_H_

#define NUM_THRUSTERS 8

int thruster_init(void);
void thruster_send(int thruster, float thrust);
void thruster_send_all(const float thrusts[NUM_THRUSTERS]);
void thruster_send_all_uniform(float thrust);
void thruster_stop(void);

#endif /* MARITIME_THRUSTER_H_ */
