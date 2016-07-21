/**
  ******************************************************************************
  * @file    bluetooth.h
  * @author  zhangyan
  * @date    2016.06.14
  * @note    À¶ÑÀÐÅºÅ·¢ËÍÄ£¿é
  ******************************************************************************
  */
#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include "stdint.h"
#include "uart.h"

typedef __packed struct uartsend
{
    uint8_t FH;
    
    int16_t test1;
    int16_t test2;
    int16_t test3;
    int16_t test4;
    uint8_t FL;

}bluetoothsend;


void bluetoooth_init(void);
void bluetooth_send(void);
int16_t bluetoooth_int_16_Exchange(int16_t value);
int32_t bluetoooth_int_32_Exchange(int value);
uint32_t bluetoooth_float_Exchange(float value);
#endif


