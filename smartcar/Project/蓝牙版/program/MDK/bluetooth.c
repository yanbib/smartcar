
/**
******************************************************************************
  * @file    bluetooth.c
  * @author  zhangyan
  * @date    2016.06.15
  * @note    蓝牙模块
  ******************************************************************************
  */
#include "bluetooth.h"

extern bluetoothsend blstruction;

static uint8_t instance;

 /**
 * @brief  bluetoooth_init
 * @param  void
 * @retval void
 * @note   蓝牙初始化
 */
void bluetoooth_init(void)
{
    
    instance = UART_QuickInit(UART0_RX_PA15_TX_PA14,115200);
    UART_SetDMATxMode(instance,true);
    
    blstruction.FH = 0xBB;
    blstruction.FL = 0xEE;
    
}


 /**
 * @brief  bluetooth_send
 * @param  void
 * @retval void
 * @note   蓝牙发送数据
 */
void bluetooth_send(void)
{
    
    
//    
    UART_DMASendByte(instance,(uint8_t*)&blstruction,20);
    

}


 /**
 * @brief  bluetoooth_int_16_Exchange
 * @param  int16_t 
 * @retval int16_t
 * @note   蓝牙高低八位数据交换
 */
int16_t bluetoooth_int_16_Exchange(int16_t value)
{
    int16_t temp;
    temp = value>>8;
    
    value = (value&0xff)<<8|(0xff&temp);
    
    return value;

}


 /**
 * @brief  bluetoooth_float_Exchange
 * @param  float 
 * @retval uint32_t
 * @note   蓝牙高低32位数据交换
 */
uint32_t bluetoooth_float_Exchange(float value)
{
    uint8_t a,b,c,d;
    uint32_t *temp;
    temp=(uint32_t *)&value;
    
    a = *temp & 0xff;
    b = *temp   >>  8;
    c = *temp   >> 16;
    d = *temp   >> 24;
//    temp = (int8_t)(value>>8);
//    
//    value = (value&0x0f)<<8|temp;
//    

    value = (uint32_t)a<<24 |(uint32_t) b<<16 |(uint32_t) c <<8 | d ;
    return value;

}


 /**
 * @brief  bluetoooth_int_32_Exchange
 * @param  int 
 * @retval int32_t
 * @note   蓝牙高低32位数据交换
 */
int32_t bluetoooth_int_32_Exchange(int32_t value)
{
    uint8_t a,b,c,d;
    uint32_t *temp;
    temp=(uint32_t *)&value;
    
    a = *temp & 0xff;
    b = *temp>>8;
    c = *temp >> 16;
    d = *temp >> 24;
//    temp = (int8_t)(value>>8);
//    
//    value = (value&0x0f)<<8|temp;
//    

    value = (uint32_t)a<<24 |(uint32_t) b<<16 |(uint32_t) c <<8 | d ;
    return value;

}


