#include "mpu_transplant.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "i2c.h"
#include <math.h>
#include "flashcontrol.h"
#define q30  1073741824.0f
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float Pitch,Roll,Yaw;
unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];
                                           
//static float roll_angle;
//////////////////////////////////////////////////////////////////////////
/*内部库调用*/
int IIC_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data)
{
	if(I2C_BurstWrite(HW_I2C0,addr,reg,1,data,len))
		return FALSE;
	else
		return TRUE;
}

int IIC_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	if(I2C_BurstRead(HW_I2C0,addr,reg,1,buf,len))
		return FALSE;
	else
		return TRUE;
}


void PrintChar(char *s)
{


}
//////////////////////////////////////////////////////////////////////////


 /**
 * @brief  mpu6050_init
 * @param  void
 * @retval 成功为0
 * @note 初始化mpu6050    频率为100*1000HZ
 */
int16_t mpu6050_init(void)
{
    
    I2C_QuickInit(I2C0_SCL_PB02_SDA_PB03,100*1000);
    
    
    int result=0;
    
    result=mpu_init();
	if(!result)
	{	 		 
	
		PrintChar("mpu initialization complete......\n ");		//mpu initialization complete	 	  

		if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))		//mpu_set_sensor
			PrintChar("mpu_set_sensor complete ......\n");
		else
			PrintChar("mpu_set_sensor come across error ......\n");

		if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))	//mpu_configure_fifo
			PrintChar("mpu_configure_fifo complete ......\n");
		else
			PrintChar("mpu_configure_fifo come across error ......\n");

		if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))	   	  		//mpu_set_sample_rate
		 PrintChar("mpu_set_sample_rate complete ......\n");
		else
		 	PrintChar("mpu_set_sample_rate error ......\n");

		if(!dmp_load_motion_driver_firmware())   	  			//dmp_load_motion_driver_firmvare
			PrintChar("dmp_load_motion_driver_firmware complete ......\n");
		else
			PrintChar("dmp_load_motion_driver_firmware come across error ......\n");

		if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) 	  //dmp_set_orientation
		 	PrintChar("dmp_set_orientation complete ......\n");
		else
		 	PrintChar("dmp_set_orientation come across error ......\n");

		if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
		    DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
		    DMP_FEATURE_GYRO_CAL))		   	 					 //dmp_enable_feature
		 	PrintChar("dmp_enable_feature complete ......\n");
		else
		 	PrintChar("dmp_enable_feature come across error ......\n");

		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))   	 			 //dmp_set_fifo_rate
		 	PrintChar("dmp_set_fifo_rate complete ......\n");
		else
		 	PrintChar("dmp_set_fifo_rate come across error ......\n");

		run_self_test();		//自检

		if(!mpu_set_dmp_state(1))
		 	PrintChar("mpu_set_dmp_state complete ......\n");
		else
		 	PrintChar("mpu_set_dmp_state come across error ......\n");
	}
    return 0;
}

 /**
 * @brief  mpu6050_readangle_dmp
 * @param   *pitch_dmp   *roll_dmp   *yaw_dmp  gyro (x y z)
 * @retval 成功为0，不成功为1
 * @note   读取mpu6050角度
 */

//int mpu6050_readangle_dmp(float *pitch_dmp,float *roll_dmp,float *yaw_dmp,int16_t *gyro_x,int16_t *gyro_y,int16_t *gyro_z)
//{
//      
//    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);	 
//      
//      	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
//	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
//	**/
//	/*if (sensors & INV_XYZ_GYRO )
//	send_packet(PACKET_TYPE_GYRO, gyro);
//	if (sensors & INV_XYZ_ACCEL)
//	send_packet(PACKET_TYPE_ACCEL, accel); */
//	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
//	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
//	**/
//    
//      if(sensors & INV_WXYZ_QUAT )
//      {
//        
//        
//        q0 = quat[0] / q30;	
//        q1 = quat[1] / q30;
//        q2 = quat[2] / q30;
//        q3 = quat[3] / q30;

//        Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
//        Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
//        Yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
//        
//        
//          
//        *pitch_dmp =  Pitch;
//        *roll_dmp  =  Roll;
//        *yaw_dmp   =  Yaw;
//        *gyro_x     =   gyro[0];
//        *gyro_y     =   gyro[1];
//        *gyro_z     =   gyro[2];
//          
//        return 0;//success
//      }
//    
//      return 1;
//}

int mpu6050_readangle_dmp(float *pitch_dmp,float *roll_dmp,float *yaw_dmp,int16_t *gyro_x,int16_t *gyro_y,int16_t *gyro_z)
{
  
    static int16_t gyro_y_save=0,gyro_z_save=0;
    static float  roll_save =0,pitch_save =0,yaw_save =0;
      float GyroscopeAngleSpeed;
    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);	 
      
      	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	/*if (sensors & INV_XYZ_GYRO )
	send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
	send_packet(PACKET_TYPE_ACCEL, accel); */
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
        int16_t  gdata[3];
        uint8_t buf[6];
    
        while(I2C_BurstRead(HW_I2C0, 0x68, 0x43, 1, buf, 6));

        
        gdata[0] = (int16_t)(((uint16_t)buf[0]<<8)+buf[1]); 	    
        gdata[1] = (int16_t)(((uint16_t)buf[2]<<8)+buf[3]); 	    
        gdata[2] = (int16_t)(((uint16_t)buf[4]<<8)+buf[5]);
    
      if(sensors & INV_XYZ_GYRO )
      {
        
        
        q0 = quat[0] / q30;	
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;

        Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
        Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
        Yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
        
        
         
        *pitch_dmp =  pitch_save= Pitch;
        *roll_dmp  =  roll_save = Roll;
        *yaw_dmp   =  yaw_save  = Yaw;
        
        *gyro_x     =   gdata[0]+ flashcontrol_read(GYROSCOPE_OFFSET);
        *gyro_y     =   gyro_y_save = gyro[1];
        *gyro_z     =   gyro_z_save = gyro[2];
        
        

        return 0;//success
      }
    

      
        
        GyroscopeAngleSpeed = (gdata[0] + flashcontrol_read(GYROSCOPE_OFFSET)) *flashcontrol_read(GYROSCOPE_ANGLE_RATIO);
        roll_save += GyroscopeAngleSpeed;
      
        *pitch_dmp =  pitch_save;
        *roll_dmp  =  roll_save ;
        *yaw_dmp   =  yaw_save  ;
      
        *gyro_x     =   gdata[0]+ flashcontrol_read(GYROSCOPE_OFFSET);
        *gyro_y     =   gyro_y_save;
        *gyro_z     =   gyro_z_save;
      
      return 1;
}

