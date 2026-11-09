#ifndef _MARITIME_KILLSWITCH_H
#define _MARITIME_KILLSWITCH_H

#include <stdint.h>

enum killswitch_state {
	/*
	 * Not read killed since setup, whatever it reads now: a board that resets mid-run
	 * (brownout, watchdog) stays killed until someone cycles the switch, rather than
	 * driving again on its own.
	 */
	KILLSWITCH_NEEDS_CYCLE,
	KILLSWITCH_KILLED,
	KILLSWITCH_RELEASED,
};

/*
 * Configures the pin and starts over at KILLSWITCH_NEEDS_CYCLE. If it fails, the switch
 * reads killed from then on.
 */
int killswitch_init(void);
/*
 * Reads the switch at `now` [ms] and returns its debounced state. Called every ms or so, and
 * only ever from one thread: a reading holds from the first call that sees it to the last.
 */
enum killswitch_state killswitch_update(int64_t now);

#endif /* _MARITIME_KILLSWITCH_H */
