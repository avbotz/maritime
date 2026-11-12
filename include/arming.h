#ifndef MARITIME_ARMING_H_
#define MARITIME_ARMING_H_

#include "thruster.h"

#include <stdbool.h>
#include <stdint.h>

/* Starts killed, with the thrusters neutral. After thruster_init(). */
void arming_init(void);
/* Update state based on kill switch position and the command timeout */
void arming_update(bool released, int64_t now);
/* A thrust command from the host. Sent to the thrusters only when armed */
void arming_command(const float thrusts[NUM_THRUSTERS], int64_t now);
/* Thrust commands are obeyed */
bool arming_is_armed(void);
/* Times the command timeout stopped the thrusters */
uint32_t arming_command_timeouts(void);

#endif /* MARITIME_ARMING_H_ */
