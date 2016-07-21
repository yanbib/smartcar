#ifndef __FLASHCONTROL_H__
#define __FLASHCONTROL_H__
  
  #include "flash.h"
  #include <stdint.h>
  
   #define CAR_ANGLE_SET            84
  #define ANGLE_CONTROL_P           4
  #define ANGLE_CONTROL_D           8
  #define ANGLE_CONTROL_OUT_MAX     12
  #define ANGLE_CONTROL_OUT_MIN     16
  #define MOTOR_OUT_DEAD_VAL_L      20
  #define MOTOR_OUT_MAX             24
  #define MOTOR_OUT_MIN             28
  #define CAR_SPEED_SET             32
  #define SPEED_CONTROL_P           36  //速度比例控制
  #define SPEED_CONTROL_I           40  //速度积分控制 
  #define DIR_ZEROSHIFT_LEFTB       44
  #define DIR_ZEROSHIFT_RIGHTB      48
  #define DIR_LEFT_RIGHT_MINIMUM    52
  #define DIR_CONTROL_P             56
  #define DIR_CONTROL_D             60
  #define MOTOR_OUT_DEAD_VAL_R      64
  #define DELAY_START               68
  #define SPEED_START_SPEED         72
  #define CAR_SPEED_SET_BEND        76
  #define BEND_DETETION             80
  #define GRAVITY_ANGLE_RATIO       88
  #define GYROSCOPE_ANGLE_RATIO     92
  #define GRAVITY_ADJUST_TIME_CONSTANT  96
  #define GYROSCOPE_OFFSET          100
  #define ZGYRO_OFFSET              104
  #define SPEEDLEFT                 108
  #define SPEEDRIGHT                112
  #define MOTOR_CORRECT_VAL_L_BACK       116
  #define MOTOR_CORRECT_VAL_R_BACK       120
  #define MOTOR_CORRECT_VAL_L_FRONT      124
  #define MOTOR_CORRECT_VAL_R_FRONT      128
  #define CAR_ANGLE_SET_BEND             132
  #define CAR_ANGLE_SET_START            136
  
  int flashcontrol_write(uint8_t addr,float value);
  float flashcontrol_read(uint8_t addr);
  void flashcontrol_init(uint8_t flag);
#endif

