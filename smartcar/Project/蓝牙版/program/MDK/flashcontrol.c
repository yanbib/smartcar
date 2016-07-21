
#include "flashcontrol.h"

//uint8_t * flashvalue = (uint8_t *)0x13000;
uint8_t * flashvalue;
uint32_t addrvalue[7]={0x13000,0x14000,0x15000,0x16000,0x17000,0x18000,0x19000};
static uint8 addrflag;
typedef union flash_arg
{
    float   f_value;
    uint8_t i_value[4];
}Flash_arg;


int flashcontrol_write(uint8_t addr,float value)
{
    
    uint8_t err;
    uint8_t buf[2048];
    uint16_t i;
    uint32_t addrphysical = addrvalue[addrflag];
    Flash_arg valuesave;
  
        for(i=0;i<2048;i++)
    {
        buf[i] = flashvalue[i];
    }
    
    
    valuesave.f_value = value;
    
    buf[addr]   = valuesave.i_value[0];
    buf[addr+1] = valuesave.i_value[1];
    buf[addr+2] = valuesave.i_value[2];
    buf[addr+3] = valuesave.i_value[3];
    
    err = 0;
    err += FLASH_EraseSector(addrphysical);
    err += FLASH_WriteSector(addrphysical, buf, 2048);
    return err;//success
    
}

float flashcontrol_read(uint8_t addr)
{
    
    Flash_arg valuesave;
    valuesave.i_value[0] = flashvalue[addr];
    valuesave.i_value[1] = flashvalue[addr+1];
    valuesave.i_value[2] = flashvalue[addr+2];
    valuesave.i_value[3] = flashvalue[addr+3];
  
    return valuesave.f_value;
//    return ((flashvalue[addr]<<8)|flashvalue[addr+1]);

}

void flashcontrol_init(uint8_t flag)
{
    flashvalue = (uint8_t *)addrvalue[flag];
    addrflag   =  flag;
}
