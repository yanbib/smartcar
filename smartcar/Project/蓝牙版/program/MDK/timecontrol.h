#ifndef __TIMECONTROL_H__
#define __TIMECONTROL_H__


#include "common.h"
#include "stdint.h"

#define MAXSIZEqueue 120
#define MAXSIZEtask 6
//typedef int16_t QElemType;
typedef void (*Task_CallBackType)(void);
typedef __packed struct
{
    int16_t front;
    int16_t rear;
    Task_CallBackType appfun[MAXSIZEtask];
}taskqueue;

typedef __packed struct
{
  taskqueue data[MAXSIZEqueue];
  int16_t front;
  int16_t rear;
  
}TIMqueue;

int16_t timeCTL_TurntableInit(TIMqueue *Q);
int16_t timeCTL_TaskqueueInit(TIMqueue *Q);
int16_t timeCTL_TaskqueueExecute(TIMqueue *Q);
int16_t timeCTL_TaskqueueInsert(TIMqueue *Q,Task_CallBackType irqapp,uint8_t ms);

#endif


