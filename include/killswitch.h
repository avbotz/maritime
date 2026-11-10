#ifndef MARITIME_KILLSWITCH_H_
#define MARITIME_KILLSWITCH_H_

#include <stdint.h>

enum killswitch_state {
	/* Killswitch in unkilled position, but hasn't been killed since startup. Require a cycle to
	 * be considered released.
	 */
	KILLSWITCH_NEEDS_CYCLE,
	KILLSWITCH_KILLED,
	KILLSWITCH_RELEASED,
};

int killswitch_init(void);
/* Should be called around every ms from a single thread. */
enum killswitch_state killswitch_update(int64_t now);

#endif /* MARITIME_KILLSWITCH_H_ */
