
#include "motorcontrol.h"

 /**
 * @brief  motorcontrol_init
 * @param  void 
 * @retval void
 * @note   电机控制初始化
 */
void  motorcontrol_init(void)
{

    
    FTM_PWM_QuickInit(FTM0_CH0_PC01, kPWM_EdgeAligned,20000);
    FTM_PWM_QuickInit(FTM0_CH1_PC02, kPWM_EdgeAligned,20000);
    FTM_PWM_QuickInit(FTM0_CH2_PC03, kPWM_EdgeAligned,20000);
    FTM_PWM_QuickInit(FTM0_CH3_PC04, kPWM_EdgeAligned,20000);
    
    FTM_PWM_ChangeDuty(HW_FTM0 ,HW_FTM_CH0,0);
    FTM_PWM_ChangeDuty(HW_FTM0 ,HW_FTM_CH1,0);
    FTM_PWM_ChangeDuty(HW_FTM0 ,HW_FTM_CH2,0);
    FTM_PWM_ChangeDuty(HW_FTM0 ,HW_FTM_CH3,0);   
    
}



void motorcontrol_TRL(uint32_t frontleft,uint32_t backleft,uint32_t backright,uint32_t frontright)
{
    
       FTM_PWM_ChangeDuty(HW_FTM0 ,HW_FTM_CH0,backleft);
       FTM_PWM_ChangeDuty(HW_FTM0 ,HW_FTM_CH1,frontleft);
       FTM_PWM_ChangeDuty(HW_FTM0 ,HW_FTM_CH2,backright);
       FTM_PWM_ChangeDuty(HW_FTM0 ,HW_FTM_CH3,frontright); 

}

