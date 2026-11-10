#ifndef MARITIME_PRESSURE_H_
#define MARITIME_PRESSURE_H_

int pressure_init(void);
/* Takes the current reading as the surface (depth 0) */
int pressure_reset_reference(void);
/*
 * Takes the current reading as the surface, but only if it is within limit_m of
 * the surface already (or there is no surface reading yet), so that a kill switch
 * release underwater does not move the surface. -ERANGE if it is not.
 */
int pressure_rezero_near_surface(float limit_m);
/* Depth below the surface reading [m, positive down]; -EIO if the sensor cannot be read */
int pressure_get_depth_m(float *depth_m);

#endif /* MARITIME_PRESSURE_H_ */
