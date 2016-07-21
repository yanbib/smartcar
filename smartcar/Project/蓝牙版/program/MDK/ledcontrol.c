/**
  ******************************************************************************
  * @file    ledcontrol.c
  * @author  zhangyan
  * @date    2016.06.23
  * @note    此文件为led小灯亮灭控制
  ******************************************************************************
  */
#include "ledcontrol.h"

 /**
 * @brief  快速初始化一个GPIO引脚 实际上是GPIO_Init的最简单配置
 * @code
 *      //初始化配置PORTB端口的10引脚为推挽输出引脚
 *      GPIO_QuickInit(HW_GPIOB, 10, kGPIO_Mode_OPP);
 * @endcode
 * @param[in]  instance GPIO模块号
 *              @arg HW_GPIOA 芯片的PORTA端口
 *              @arg HW_GPIOB 芯片的PORTB端口
 *              @arg HW_GPIOC 芯片的PORTC端口
 *              @arg HW_GPIOD 芯片的PORTD端口
 *              @arg HW_GPIOE 芯片的PORTE端口
 * @param[in]  pinx 端口上的引脚号 0~31
 * @param[in]  mode 引脚工作模式
 *              @arg kGPIO_Mode_IFT 悬空输入
 *              @arg kGPIO_Mode_IPD 下拉输入
 *              @arg kGPIO_Mode_IPU 上拉输入
 *              @arg kGPIO_Mode_OOD 开漏输出
 *              @arg kGPIO_Mode_OPP 推挽输出
 * @retval instance GPIO模块号
 */
void led_init(void)
{
    GPIO_QuickInit(HW_GPIOE,6,kGPIO_Mode_OOD);
    GPIO_QuickInit(HW_GPIOE,7,kGPIO_Mode_OOD);
    GPIO_QuickInit(HW_GPIOE,11,kGPIO_Mode_OOD);
    GPIO_QuickInit(HW_GPIOE,12,kGPIO_Mode_OOD);

}

 /**
 * @brief  设置指定引脚输出高电平或者低电平
 * @note   此引脚首先配置成输出引脚
 * @code
 *      //设置PORTB端口的10引脚输出高电平
 *      GPIO_WriteBit(HW_GPIOB, 10, 1);
 * @endcode
 * @param[in]  instance GPIO模块号
 *              @arg HW_GPIOA 芯片的PORTA端口
 *              @arg HW_GPIOB 芯片的PORTB端口
 *              @arg HW_GPIOC 芯片的PORTC端口
 *              @arg HW_GPIOD 芯片的PORTD端口
 *              @arg HW_GPIOE 芯片的PORTE端口
 * @param[in]  pin  端口上的引脚号 0~31
 * @param[in]  data 引脚的电平状态  
 *              @arg 0  低电平 
 *              @arg 1  高电平
 * @retval None
 */
void led_ctl(uint16_t name,uint8_t data)
{
    if(name &   LEDCTL_A)   GPIO_WriteBit(HW_GPIOE,6,data);
    if(name &   LEDCTL_B)   GPIO_WriteBit(HW_GPIOE,7,data);
    if(name &   LEDCTL_C)   GPIO_WriteBit(HW_GPIOE,11,data);
    if(name &   LEDCTL_D)   GPIO_WriteBit(HW_GPIOE,12,data);
    
}

/**
 * @brief  翻转一个引脚的电平状态
 * @code
 *      //翻转PORTB端口的10引脚的电平状态
 *      GPIO_ToggleBit(HW_GPIOB, 10); 
 * @endcode
 * @param[in]  instance: GPIO模块号
 *              @arg HW_GPIOA 芯片的PORTA端口
 *              @arg HW_GPIOB 芯片的PORTB端口
 *              @arg HW_GPIOC 芯片的PORTC端口
 *              @arg HW_GPIOD 芯片的PORTD端口
 *              @arg HW_GPIOE 芯片的PORTE端口
 * @param[in]  pin  端口上的引脚号 0~31
 * @retval None
 */
void led_toggle(uint16_t name)
{
    
    if(name&LEDCTL_A)   GPIO_ToggleBit(HW_GPIOE,6);
    if(name&LEDCTL_B)   GPIO_ToggleBit(HW_GPIOE,7);
    if(name&LEDCTL_C)   GPIO_ToggleBit(HW_GPIOE,11);
    if(name&LEDCTL_D)   GPIO_ToggleBit(HW_GPIOE,12);
    
}
