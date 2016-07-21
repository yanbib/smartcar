/**
  ******************************************************************************
  * @file    main.c
  * @author  zhangyan
  * @date    2016.06.14
  * @note    智能车主体模块
  *          
  ******************************************************************************
  */
  

#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "ftm.h"
#include "pit.h"

#include "AD.h"
#include "bluetooth.h"
#include "speed_control.h"
#include "timecontrol.h"
#include "mpu_transplant.h"
#include "nrfcontrolyan.h"
#include "motorcontrol.h"
#include "flashcontrol.h"
#include "ledcontrol.h"

#include "shell.h"
#include "stdbool.h"
#include "stdlib.h"
#include "QUEUEyan.h"

#include <math.h>

/////////////////////////////////////////////
/*宏定义*/

//ControlBus timeUS 定时器中断时间
#define ControlBus_timeValue 1000
//msgdeal and flash W/R 定时器中断时间
#define msgdealISR_timeValue 1000*50
//速度控制平滑次数
#define SPEED_CONTROL_PERIOD 20
//方向控制平滑总次数
#define DIRECTION_CONTROL_PERIOD 2
/////////////////////////////////////////////


/////////////////////////////////////////////
 /*数据结构体*/
 //蓝牙数据  
bluetoothsend blstruction;

//时间轮转结构体
TIMqueue timeturnstruction;

SqQueue detetionbendque;

/////////////////////////////////////////////


/////////////////////////////////////////////
/*全局变量定义*/

//小车速度
static float g_fCarSpeed;
//速度控制参数
//static float g_fspeedcontrolargp;
//速度控制积分
static float g_fSpeedControlIntegral;
//速度输出新旧值
static float g_fSpeedControlOutOld = 0 ;
static float g_fSpeedControlOutNew = 0;
//速度控制输出
static float g_fSpeedControlOut;
//速度控制当前次数
static uint8_t g_nSpeedControlPeriod =0;
//角度角速度
static float g_fCarAngle,g_fGyroscopeAngleSpeed;
//角度输出
static float g_fAngleControlOut;
//左右电机输出
static float g_fLeftMotorOut;
static float g_fRightMotorOut;

//方向角速度
static float g_fDirControlDValue;

//旧的方向控制的值
static float g_fDirectionControlOutOld;
//新的方向控制的值
static float g_fDirectionControlOutNew;

//方向平滑输出量
static float g_fDirectionControlOut;

//方向控制当前次数
static uint8_t g_nDirectionControlPeriod;


/*数据发送控制*/
bool valuecontrol = 0;
/*启停开关*/
uint8_t StopMark = 0;
/*弯道判断标记标记*/
uint8_t BendMark;

/////////////////////////////////////////////


/////////////////////////////////////////////
/*函数声明*/
void ControlBus(void);
void init_start(void);
void msgdealWR(void);
void testbluetooth(void);
void test_speed_control_read(void);
void angle_read(void);
void AngleControl(void);
void MotorOutput(void);
void MotorSpeedRead(void);
void AD_init(void);
void DirectionControl(void);
void leddisplay(void);
void delaytaskstart(void);
float detetionbend(float value);//检测入弯滤波
void stopcontroldelay(void);
////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief  主函数
 * @param[in]  void 
 * @retval None
 * @note 
 */
int main(void)
{

    
    init_start();


    while(1)
    {

        
        shell_main_loop("byGTA^_^>>");
        DelayMs(100);

        
    }
}
                              
////////////////////////////////////////////////////////////////////////////////////////////
                                /*定时器中断*/
//////////////////////////////////////////////////////////////////////
/**
 * @brief  ControlBus
 * @param[in]  void 
 * @retval None
 * @note 定时器中断 数据的读取和处理 周期：1MS
 */
void ControlBus(void)
{
    
    timeCTL_TaskqueueExecute(&timeturnstruction);
}
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
/**
 * @brief  msgdealWR
 * @param[in]  void 
 * @retval None
 * @note 定时器中断  数据通讯，flash读写 周期：msgdealISR_timeValue
 */
void msgdealWR(void)
{
  
  if(valuecontrol == true)\
      bluetooth_send();
    
}

//停止触发中断
void stop_isr(uint32_t array)
{
    static uint8_t count=0;
    if(array & (1 << 7))
    {
        GPIO_ITDMAConfig(HW_GPIOA, 7, kGPIO_IT_Low, false);
        count++;
        if(count==1)
        {
          StopMark = 0;
          count = 0;
          led_ctl(LEDCTL_B,0);
        }
        
//        GPIO_ITDMAConfig(HW_GPIOA, 7, kGPIO_IT_Low, true);
    }
}
  

//////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
/*模式选择*/
void modeselect(void)
{
    static uint8_t flag;
    GPIO_QuickInit(HW_GPIOE,5,kGPIO_Mode_IPD);
    GPIO_QuickInit(HW_GPIOE,1,kGPIO_Mode_IPD);
    GPIO_QuickInit(HW_GPIOE,4,kGPIO_Mode_IPD);
    GPIO_QuickInit(HW_GPIOE,3,kGPIO_Mode_IPD);
  
    flag=GPIO_ReadBit(HW_GPIOE,3)|GPIO_ReadBit(HW_GPIOE,4)<<1|GPIO_ReadBit(HW_GPIOE,1)<<2|GPIO_ReadBit(HW_GPIOE,5)<<3;
    led_ctl(flag,0);
    
}

/**
 * @brief  完成智能车启动配置
 * @param[in]  void 
 * @retval None
 * @note 
 */
void init_start(void)
{
////////////////////
    /*系统初始化*/
    DelayInit();
  
    /*延时 硬件初始化等待 */
    DelayMs(10);
    
    
//////////////////////////////////////////////////////////////////////////////////
    /*初始化定时器中断*/
  
    /*初始化中断优先级*/
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_2);
    NVIC_SetPriority(PORTA_IRQn,NVIC_EncodePriority(NVIC_PriorityGroup_2,0,0));
    NVIC_SetPriority(PIT0_IRQn,NVIC_EncodePriority(NVIC_PriorityGroup_2,0,2));
    NVIC_SetPriority(PIT1_IRQn,NVIC_EncodePriority(NVIC_PriorityGroup_2,1,2));  
//    NVIC_SetPriority(PIT2_IRQn,NVIC_EncodePriority(NVIC_PriorityGroup_2,2,2));  
    /*分配定时器硬中断*/
    /*smartcarcontror*/
    PIT_QuickInit(HW_PIT_CH0,ControlBus_timeValue);         // 快速初始化PIT模块的0通道
    PIT_CallbackInstall(HW_PIT_CH0,ControlBus);             //快速初始化  定时器中断 数据的读取和处理
  
    /*msgdeal and flash W/R*/
    PIT_QuickInit(HW_PIT_CH1,msgdealISR_timeValue);         //快速初始化PIT模块的1通道
    PIT_CallbackInstall(HW_PIT_CH1,msgdealWR);              //快速初始化PIT模块的1通道数据通讯
    
       
    GPIO_QuickInit(HW_GPIOA, 7, kGPIO_Mode_IPU); /* KEY */

    /* 设置GPIO外部引脚中断回调函数 */
    GPIO_CallbackInstall(HW_GPIOA, stop_isr);
    /* 中断 触发 */
    GPIO_ITDMAConfig(HW_GPIOA, 7, kGPIO_IT_Low, false);

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
    /*初始化时间片队列*/
    timeCTL_TurntableInit(&timeturnstruction);
    timeCTL_TaskqueueInit(&timeturnstruction);
    
    InitQueue(&detetionbendque);
//////////////////////////////////////////////////////////////////////////////////
    /*初始化传感器及其他*/   
    
    bluetoooth_init();
    speedcontrol_init();
    mpu6050_init();
    nrfcontrol_init();
    motorcontrol_init();
    led_init();
    led_ctl(LEDCTL_A|LEDCTL_B | LEDCTL_C |LEDCTL_D,1);
    AD_init(); 
    flashcontrol_init(2);
//    modeselect();
//////////////////////////////////////////////////////////////////////////////////    
    /*shell初始化*/
    shell_init();

//////////////////////////////////////////////////////////////////////////////////
    /*插入执行任务*/
    timeCTL_TaskqueueInsert(&timeturnstruction,leddisplay,5);
    timeCTL_TaskqueueInsert(&timeturnstruction,angle_read,6);
    timeCTL_TaskqueueInsert(&timeturnstruction,MotorOutput,8);
    timeCTL_TaskqueueInsert(&timeturnstruction,MotorSpeedRead,7);
    timeCTL_TaskqueueInsert(&timeturnstruction,DirectionControl,9);
    timeCTL_TaskqueueInsert(&timeturnstruction,stopcontroldelay,100);
//////////////////////////////////////////////////////////////////////////////////
    /*定时器中断打开*/
    PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE); //打开中断
    PIT_ITDMAConfig(HW_PIT_CH1, kPIT_IT_TOF,ENABLE); //打开中断

  
    

}
////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief  智能车任务处理
 * @note   包含角度速度方向控制及处理，电机输出函数
 */


/**
 * @brief  8s延时打开终点停止
 * @param[in]  void 
 * @retval None
 * @note 
 */


void stopcontroldelay(void)
{
    static uint16_t count=0;
    if(StopMark)
      count++;
    else
      count = 0;
    if(count>70)
      GPIO_ITDMAConfig(HW_GPIOA, 7, kGPIO_IT_Low, true);
    
  timeCTL_TaskqueueInsert(&timeturnstruction,stopcontroldelay,100);
}

/**
 * @brief  实现LED闪烁
 * @param[in]  void 
 * @retval None
 * @note 80ms翻转
 */

void leddisplay(void)
{
    
    led_toggle(LEDCTL_A);
    timeCTL_TaskqueueInsert(&timeturnstruction,leddisplay,80);
}

/**
 * @brief  角度角速度读取 5ms
 * @param[in]  void 
 * @retval None
 * @note 从MPU6050中读取角度角速度：
 *       1 保存roll（X轴角度值）到 变量g_fGyroscopeAngleSpeed X轴角速度 到 变量 g_fGyroscopeAngleSpeed 用于直立控制
 *       2 保存z轴角速度值      到 变量g_fDirControlDValue 用于方向控制
 *       3 执行AngleControl()
 */
void angle_read(void)
{
  
    float   pitch;
    float   roll;
    float   yaw;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
  

  //读取mpu6050值
    mpu6050_readangle_dmp( &pitch, &roll, &yaw, &gyro_x, &gyro_y, &gyro_z);
    
  //发送z轴角速度
    g_fDirControlDValue = gyro_z;
  //发送x轴角度和角速度
    g_fCarAngle = roll;
    g_fGyroscopeAngleSpeed = gyro_x;



    AngleControl();
    
    //插入到时间任务队列

    timeCTL_TaskqueueInsert(&timeturnstruction,angle_read,5);
  
}
/*目标角度选择*/
float AngleTarget(uint8_t flag)
{
    switch(flag)
    {
      case 1: return flashcontrol_read(CAR_ANGLE_SET);
      case 0: return flashcontrol_read(CAR_ANGLE_SET_BEND);
      case 2: return flashcontrol_read(CAR_ANGLE_SET_START);
    }
      return 1;
    
}

/**
 * @brief  角度控制计算并发送
 * @param[in]  void 
 * @retval None
 * @note 进行PID计算，阈值处理 并保存 计算输出值 到 变量g_fAngleControlOut
 */
void AngleControl(void)
{
    static uint16_t count;
    float fValue;
    uint8_t flag=0;
    if(StopMark)
    {
      if(count<600)
      {
          count++;
          flag =2;
      }
      else
      {
          flag = 1;
      } 
    }
    else
    {
        count =0;
    
    }
    //PID计算
    fValue =(AngleTarget(flag) - g_fCarAngle) *flashcontrol_read(ANGLE_CONTROL_P)\
            -g_fGyroscopeAngleSpeed * flashcontrol_read(ANGLE_CONTROL_D);
    
    //阈值控制
    if(fValue > flashcontrol_read(ANGLE_CONTROL_OUT_MAX))
        fValue = flashcontrol_read(ANGLE_CONTROL_OUT_MAX);
    else if(fValue < flashcontrol_read( ANGLE_CONTROL_OUT_MIN))
        fValue = flashcontrol_read( ANGLE_CONTROL_OUT_MIN);
    //发送输出值
    g_fAngleControlOut = fValue;
}

/**
 * @brief  输出电机功率
 * @param[in]  fLeftVoltage 左电机输出值
 * @param[in]  fRightVoltage  右电机输出值
 * @retval None
 * @note 
         1 根据输入输出值 控制4路PWM 从而达到控制电机输出
         2 根据标志位StopMark 控制电机是否启动
 */
void SetMotorVoltage(float fLeftVoltage, float fRightVoltage)
{
    uint32_t backleft ,frontleft,backright,frontright;
    backleft = frontleft = backright = frontright =0;
    
    if(fLeftVoltage > 0 ) \
        frontleft = fLeftVoltage+flashcontrol_read(MOTOR_CORRECT_VAL_L_FRONT);
    else \
        backleft  = - (fLeftVoltage+flashcontrol_read(MOTOR_CORRECT_VAL_L_BACK));
    
    if(fRightVoltage > 0 ) \
        frontright = fRightVoltage+flashcontrol_read(MOTOR_CORRECT_VAL_R_FRONT);
    else \
        backright  = - (fRightVoltage+flashcontrol_read(MOTOR_CORRECT_VAL_R_BACK));
    /*电机控制*/
    if(StopMark) \
      motorcontrol_TRL(backleft,frontleft,backright,frontright);
    else \
      motorcontrol_TRL(0,0,0,0);
}

/**
 * @brief  电机输出克服死区电压，设置最大阈值
 * @param[in]  void 
 * @retval None
 * @note 
          1 获取 g_fLeftMotorOut,g_fRightMotorOut 
          2 补偿左右两电机死区电压
          3 控制输出的最大阈值
          4 执行 SetMotorVoltage(fLeftVal, fRightVal)
 */
void MotorSpeedOut()
{
    
    float fLeftVal, fRightVal;
    
    fLeftVal = g_fLeftMotorOut;
    fRightVal = g_fRightMotorOut;
  
    //补偿死区电压
    if(fLeftVal > 0)
        fLeftVal += flashcontrol_read(MOTOR_OUT_DEAD_VAL_L);
    else if(fLeftVal < 0)
        fLeftVal -= flashcontrol_read(MOTOR_OUT_DEAD_VAL_L);
    
    if(fRightVal > 0)
        fRightVal +=flashcontrol_read(MOTOR_OUT_DEAD_VAL_R);
    else if(fRightVal < 0)
        fRightVal -=flashcontrol_read(MOTOR_OUT_DEAD_VAL_R);
    
    //控制阈值
    if(fLeftVal >flashcontrol_read( MOTOR_OUT_MAX )) //正转最大值
        fLeftVal = flashcontrol_read (MOTOR_OUT_MAX);
    if(fLeftVal < flashcontrol_read(MOTOR_OUT_MIN))  //反转最大值
        fLeftVal = flashcontrol_read(MOTOR_OUT_MIN);
    if(fRightVal > flashcontrol_read(MOTOR_OUT_MAX))
        fRightVal = flashcontrol_read(MOTOR_OUT_MAX);
    if(fRightVal < flashcontrol_read(MOTOR_OUT_MIN))
        fRightVal = flashcontrol_read(MOTOR_OUT_MIN);
    
    SetMotorVoltage(fLeftVal, fRightVal);
}

/**
 * @brief  累加所有输出值准备电机输出 5ms
 * @param[in]  void 
 * @retval None
 * @note 对直立控制输出值,速度控制输出值,向控制输出值融合 执行 MotorSpeedOut()
 */

void MotorOutput(void) 
{
    float fLeft, fRight;


    fLeft = g_fAngleControlOut - g_fSpeedControlOut -g_fDirectionControlOut;
    fRight = g_fAngleControlOut- g_fSpeedControlOut +g_fDirectionControlOut;
    
    g_fLeftMotorOut = fLeft;
    g_fRightMotorOut = fRight;
    MotorSpeedOut();
//    if(StopMark) 
        timeCTL_TaskqueueInsert(&timeturnstruction,MotorOutput,5);

}

/**
 * @brief  平滑计算发送真实速度控制输出
 * @param[in]  void 
 * @retval None
 * @note 对速度输出值g_fSpeedControlOutNew 进行平滑计算 分成 20步 达到目标控制值（g_fSpeedControlOutNew）降低智能车抖动
 */
void SpeedControlOutput(void) {
    
    float fValue;
    fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld;
    
    g_fSpeedControlOut = fValue * (g_nSpeedControlPeriod + 1) /
    SPEED_CONTROL_PERIOD + g_fSpeedControlOutOld;
    
}


/**
 * @brief  速度读取输出 5ms
 * @param[in]  void  
 * @retval None
 * @note 
          1 进行次数累计
          2 累计100ms（20次） 进行速度读取并进行PID计算
          3 每5ms执行 SpeedControlOutput()
 */
void MotorSpeedRead(void)
{
    float fDelta;
    float fP, fI;
    int32_t leftspeed;
    int32_t rightspeed;
    float f_leftspeed;
    float f_rightspeed;
    int16_t i_leftspeed;
    int16_t i_rightspeed;
    if(g_nSpeedControlPeriod==0){
    //读取速度
    speedcontrol_read(&leftspeed,&rightspeed);
      
    //读取出的值先进行单位转换成int16 从而得到正确的正负表示值
    i_leftspeed = leftspeed;
    i_rightspeed = rightspeed;      
    i_rightspeed = -i_rightspeed;
    f_leftspeed =(float)i_leftspeed;
    f_rightspeed =(float)i_rightspeed;
    //得到小车当前速度
    g_fCarSpeed = (f_leftspeed+f_rightspeed)/2;
//    g_fCarSpeed *= CAR_SPEED_CONSTANT;等比例转化速度单位，

    //计算与目标速度差值
    fDelta =  flashcontrol_read(CAR_SPEED_SET) - g_fCarSpeed;
    //进行PID计算
      
    fP = fDelta * flashcontrol_read(SPEED_CONTROL_P);//flashcontrol_read(SPEED_CONTROL_P);
    //积分
    fI = fDelta * flashcontrol_read(SPEED_CONTROL_I);
    if(StopMark) \
        g_fSpeedControlIntegral += fI;      
    else g_fSpeedControlIntegral =0;

    g_fSpeedControlOutOld = g_fSpeedControlOutNew;   
    g_fSpeedControlOutNew = fP + g_fSpeedControlIntegral;
    

    }
    
    SpeedControlOutput();
    //进行次数累计
    g_nSpeedControlPeriod++;
    if(g_nSpeedControlPeriod==20) g_nSpeedControlPeriod = 0;
    //插入到时间任务队列
    
        timeCTL_TaskqueueInsert(&timeturnstruction,MotorSpeedRead,5);
}

/**
 * @brief  平滑计算发送真实方向控制输出
 * @param[in]  void 
 * @retval None
 * @note 对g_fDirectionControlOut值 进行平滑输出 同上
 */
void DirectionControlOutput(void) 
{
    float fValue;
    fValue = g_fDirectionControlOutNew - g_fDirectionControlOutOld;
    g_fDirectionControlOut = fValue * (g_nDirectionControlPeriod + 1) / DIRECTION_CONTROL_PERIOD + g_fDirectionControlOutOld;

}

/**
 * @brief  方向控制输出 5ms
 * @param[in]  void 
 * @retval None
 * @note 
          1 进行次数累计
          2 累计10ms（2次） 进行方向读取并进行PID计算
          3 每5ms执行 DirectionControlOutput()
 */

void DirectionControl(void)
{
    float sampling;
    int32_t leftA_data,leftB_data,rightA_data, rightB_data;
    int16_t i=0,left_add = 0,right_add = 0;
    float nLeft,nRight,fLeftRightAdd,fLeftRightSub,fValue,fDValue;
    float fLeftVoltageSigma ,fRightVoltageSigma;
    if(g_nDirectionControlPeriod==0)
    {
        for(i=0;i<=3;i++)
        {
            AD_Read(&leftA_data,&leftB_data,&rightA_data,&rightB_data);
            left_add  +=rightB_data+rightA_data;
            right_add +=leftB_data+leftA_data;
          
        }
        fLeftVoltageSigma = left_add/4;
        fRightVoltageSigma = right_add/4;
       

           
        nLeft = fLeftVoltageSigma-flashcontrol_read(DIR_ZEROSHIFT_LEFTB);
        nRight = fRightVoltageSigma-flashcontrol_read(DIR_ZEROSHIFT_RIGHTB); 
            
        //蓝牙数据发送

        fLeftRightAdd = nLeft + nRight;
        fLeftRightSub = nLeft - nRight;
        g_fDirectionControlOutOld = g_fDirectionControlOutNew;
        if(fLeftRightAdd < flashcontrol_read(DIR_LEFT_RIGHT_MINIMUM)) 
            g_fDirectionControlOutNew = 0;
        else 
        {
            fValue = fLeftRightSub * flashcontrol_read(DIR_CONTROL_P)/ fLeftRightAdd;
            fDValue = g_fDirControlDValue;
            fDValue *=flashcontrol_read(DIR_CONTROL_D);          
            fValue += fDValue;
            g_fDirectionControlOutNew = fValue;
            sampling = fValue;
            if(sampling<0) sampling=-sampling;
            if(detetionbend(sampling)<1200) BendMark = 0;
            else BendMark=1;
            blstruction.test1 = bluetoooth_int_16_Exchange((int16_t)BendMark);
           
        } 
        
    }
    DirectionControlOutput();
 
    g_nDirectionControlPeriod++;
    if(g_nDirectionControlPeriod == DIRECTION_CONTROL_PERIOD) g_nDirectionControlPeriod =0;
    
    
        timeCTL_TaskqueueInsert(&timeturnstruction,DirectionControl,5);

}

//递推队列判断弯道
float detetionbend(float value)
{
    uint16_t length = QueueLength(detetionbendque);
    float max;
    if(length<(MAXSIZEQUE-1))
    {
        EnQueue(&detetionbendque,value);
         for(int i=0;i<=length;i++)
         {
          if(detetionbendque.data[i]>max)
              max=detetionbendque.data[i];
         }
        return max ;
    }
    else
    {
        DeQueue(&detetionbendque);
        EnQueue(&detetionbendque,value);
         for(int i=0;i<=length;i++)
         {
          if(detetionbendque.data[i]>max)
              max=detetionbendque.data[i];
         }
        return max ;
    }        

}

////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  智能车控制平台
 * @note   通过串口控制的接口函数
 */



/*shell命令控制函数*/

/*hello*/
int CMD_hello(int argc,char *const argv[])
{
    printf("hello my owner\r\n");
  
    return 0;
}

/*打开串口数据发送开关*/
int CMD_valuesendcontrol(int argc,char *const argv[])
{
      
    if( atoi(argv[argc-1]) > 1|| atoi(argv[argc-1]) <0)
    {
      printf("error:controlvalue is 0(off) or 1(on)\r\n");
      return 1;
    }
    
    valuecontrol = atoi(argv[argc-1]);
    printf("set OK controlvalue is %s \r\n",argv[argc-1]);
    return 0;
}

/*向智能车写入flash参数*/
int CMD_flashwrite(int argc,char *const argv[])
{
  if(atoi(argv[argc-2])<0||atoi(argv[argc-2])>255)
  {
    printf("error:flash address is 0 to 255\r\n");
    return 1;
  }
  if(atoi(argv[argc-2])%2)
  {
    printf("error:flash address is even number\r\n");
    return 1;
  }
  
  PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,DISABLE);
  if(flashcontrol_write(atoi(argv[argc-2]),atof(argv[argc-1])))
  {
    printf("flash write error\r\n");
    return 1;
  }
  PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE);
  printf("flash write ok address is %s value is %s\r\n",argv[argc-2],argv[argc-1]);
  return 0;
}

/*智能车运行开关*/
int CMD_startswitch(int argc,char *const argv[])
{
      if( atoi(argv[argc-1]) > 1|| atoi(argv[argc-1]) <0)
    {
      printf("error:switch is 0(off) or 1(on)\r\n");
      return 1;
    }
    StopMark = atoi(argv[argc-1]);
    printf("set OK switch is %s \r\n",argv[argc-1]);
    return 0;
}

/*读取flash已写入的值*/
int CMD_flashread(int argc,char *const argv[])
{
    int count = atoi(argv[argc-1]);
    int i = 0;
    float valuec;
    if(atoi(argv[argc-1])<0||atoi(argv[argc-1])>255)
    {
      printf("error:flash address is 0 to 255\r\n");
      return 1;
    }
    for(i=0;i<=count;i+=4)
    {
      valuec = flashcontrol_read(i);
      printf("flash address is %d value is %f \r\n",i,valuec);
    }
    
    return 0;
}


/*shell命令链接*/
SHELL_EXPORT_CMD(CMD_hello, hellorobot ,help of hello);
SHELL_EXPORT_CMD(CMD_valuesendcontrol,valuesend,help of valuesend 0 or 1);
SHELL_EXPORT_CMD(CMD_flashwrite,flashwrite,flashwrite param address and value);
SHELL_EXPORT_CMD(CMD_startswitch,startswitch,help of value is 0 or 1);
SHELL_EXPORT_CMD(CMD_flashread,flashread,max is 255)
////////////////////////////////////////////////////////////////////////////////////////////

