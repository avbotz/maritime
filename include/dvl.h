#ifndef _MARITIME_DVL_H
#define _MARITIME_DVL_H

#include <zephyr/kernel.h>

struct dvl_data_s {
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float altitude;
    float time_since_last_ms;
    bool health;
};

void setup_dvl(void);

extern struct k_msgq dvl_data_msgq;

#endif
