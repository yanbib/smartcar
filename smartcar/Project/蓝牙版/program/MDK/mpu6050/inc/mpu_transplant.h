#ifndef __MPU_TRANSPLANT_H__
#define __MPU_TRANSPLANT_H__

  #include "i2c.h"
  #include "common.h"

  
  
  #define true 1
  #define false 0 

  #define TRUE  0
  #define FALSE -1
  
  
    int IIC_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data);
    int IIC_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);


    int mpu6050_readangle_dmp(float *pitch_dmp,float *roll_dmp,float *yaw_dmp,int16_t *gyro_x,int16_t *gyro_y,int16_t *gyro_z);
  
    int16_t mpu6050_init(void);

    void PrintChar(char *s);

#endif
