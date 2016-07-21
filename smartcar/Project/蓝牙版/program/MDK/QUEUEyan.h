#ifndef __QUEUEYAN_H__
#define __QUEUEYAN_H__

#include "stdint.h"
#include "common.h"
#define MAXSIZEQUE 50
typedef float QElemTypeint;

typedef struct
{
  QElemTypeint data[MAXSIZEQUE];
  int16_t front;
  int16_t rear;
  
}SqQueue;

int16_t InitQueue(SqQueue *Q);
int16_t QueueLength(SqQueue Q);
int16_t EnQueue(SqQueue *Q,QElemTypeint e);
int16_t DeQueue(SqQueue *Q);


#endif

