#ifndef  __SPEED_CONTROL__
#define  __SPEED_CONTROL__
#include "ftm.h"
#define FTM1array 0
#define FTM2array 1


void  speedcontrol_read(int *value_left,int *value_right);
void speedcontrol_init(void);

#endif

