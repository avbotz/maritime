#ifndef MARITIME_PRESSURE_H_
#define MARITIME_PRESSURE_H_

int pressure_init(void);
/* Takes the current reading as the surface (depth 0) */
int pressure_reset_reference(void);
/* Depth below the surface reading [m, positive down]; -EIO if the sensor cannot be read */
int pressure_get_depth_m(float *depth_m);

#endif /* MARITIME_PRESSURE_H_ */
