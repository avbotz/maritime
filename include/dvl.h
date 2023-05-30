#ifndef DVL_H
#define DVL_H

void init_dvl();
float dvl_get_velocity_x();
float dvl_get_velocity_y();
float dvl_get_velocity_z();
float dvl_get_altitude();
void uart_irq_callback(const struct device *dev, void *data);

#endif /* DVL_H */
