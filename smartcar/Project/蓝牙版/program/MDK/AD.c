#include "AD.h"

/**
 * @brief  AD_init
 * @code
 *    //初始化 ADC0 通道20 引脚DM1 单端 精度 12位
 *    ADC_QuickInit(ADC0_SE20_DM1, kADC_SingleDiff12or13);
 *    //读取AD转换结果
 *    value = ADC_QuickReadValue(ADC0_SE20_DM1);
 * @retval void
 *note       ADC的初始化
 */
void  AD_init(void)
{   
    ADC_InitTypeDef adcstructionadc0;
    adcstructionadc0.instance   =   HW_ADC0;
    adcstructionadc0.triggerMode    =   kADC_TriggerSoftware;
    adcstructionadc0.clockDiv   =   kADC_ClockDiv2;
    adcstructionadc0.resolutionMode =   kADC_SingleDiff12or13;
    adcstructionadc0.singleOrDiffMode   = kADC_Single;
    adcstructionadc0.continueMode   =   kADC_ContinueConversionDisable;
    adcstructionadc0.hardwareAveMode    = kADC_HardwareAverageDisable;
    adcstructionadc0.vref   =   kADC_VoltageVREF;
          
    ADC_InitTypeDef adcstructionadc1;
    adcstructionadc1.instance = HW_ADC1;                   ///<模块号
    adcstructionadc1.triggerMode = kADC_TriggerSoftware;               ///<触发模式 软件触发 或 硬件触发
    adcstructionadc1.clockDiv = kADC_ClockDiv2;                   ///<ADC时钟分频
    adcstructionadc1.resolutionMode = kADC_SingleDiff12or13;             ///<分频率选择 8 10 12 16位精度等
    adcstructionadc1.singleOrDiffMode = kADC_Single;           ///<单端 还是 差分输入
    adcstructionadc1.continueMode = kADC_ContinueConversionDisable;               ///<是否启动连续转换
    adcstructionadc1.hardwareAveMode = kADC_HardwareAverageDisable;            ///<硬件平均功能选择
    adcstructionadc1.vref = kADC_VoltageVREF;                       ///<模拟电压参考源
    
    PORT_PinMuxConfig(HW_GPIOE,24,kPinAlt0);
    PORT_PinMuxConfig(HW_GPIOE,25,kPinAlt0);
//    PORT_PinMuxConfig(HW_GPIOA,7,kPinAlt0);
    PORT_PinMuxConfig(HW_GPIOA,8,kPinAlt0);
    
    ADC_Init(&adcstructionadc0);
    ADC_Init(&adcstructionadc1);
    
    ADC_ChlMuxConfig(HW_ADC0,kADC_MuxA);
    ADC_ChlMuxConfig(HW_ADC1,kADC_MuxA);
    
}


/**
 * @brief  AD_Read
 *    value = ADC_QuickReadValue(ADC0_SE20_DM1);
 * @retval void
 *note       ADC的数据读取
*/
void AD_Read(int32_t* leftA_data,int32_t* leftB_data,int32_t* rightA_data,int32_t* rightB_data)
{
   
    ADC_StartConversion(HW_ADC0,17,kADC_MuxA);                  //PTE24
    while(ADC_IsConversionCompleted(HW_ADC0,kADC_MuxA));
    *leftA_data  =   ADC_ReadValue(HW_ADC0,kADC_MuxA);
    
    ADC_StartConversion(HW_ADC0,11,kADC_MuxA);                  //ADC1_SE16
    while(ADC_IsConversionCompleted(HW_ADC0,kADC_MuxA));           
    *leftB_data  =   ADC_ReadValue(HW_ADC0,kADC_MuxA);
    
    ADC_StartConversion(HW_ADC0,18,kADC_MuxA);              //ADC0_SE16
    while(ADC_IsConversionCompleted(HW_ADC0,kADC_MuxA));
    *rightA_data  =   ADC_ReadValue(HW_ADC0,kADC_MuxA);
    
    ADC_StartConversion(HW_ADC0,23,kADC_MuxA);              //ADC0_SE23
    while(ADC_IsConversionCompleted(HW_ADC0,kADC_MuxA));
    *rightB_data  =   ADC_ReadValue(HW_ADC0,kADC_MuxA);
    

}

