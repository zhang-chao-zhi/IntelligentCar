#ifndef _MOTION_CONTROL_
#define _MOTION_CONTROL_

#include "main.h"

void motor_test(void);
int16_t *kinematic_analysis(float Vx,float Vy,float Vz);
void set_ccr(int16_t MA, int16_t MB, int16_t MC, int16_t MD);
int16_t *read_encoder(void);

	
#endif
