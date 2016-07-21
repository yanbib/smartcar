#ifndef __LEDCONTROL_H__
#define __LEDCONTROL_H__

#include "gpio.h"

#define LEDCTL_A 0x08
#define LEDCTL_B 0x04
#define LEDCTL_C 0x02
#define LEDCTL_D 0x01

void led_toggle(uint16_t name);
void led_ctl(uint16_t name,uint8_t data);
void led_init(void);

#endif

