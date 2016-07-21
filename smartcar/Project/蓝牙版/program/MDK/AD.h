#ifndef  __AD_H__
#define  __AD_H__
#include "adc.h"
#include "gpio.h"

void  AD_init(void);
void  AD_Read(int32_t* leftA_data,int32_t* leftB_data,int32_t* rightA_data,int32_t* rightB_data);

#endif
