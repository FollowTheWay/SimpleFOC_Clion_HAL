#ifndef __SENSOR_H
#define __SENSOR_H

#define  AS5600_CPR       4096       //12bit
extern  long  cpr;
extern  float angle_prev;
void sensor_init(void);
float getAngle(void);
float getVelocity(void);

#endif

