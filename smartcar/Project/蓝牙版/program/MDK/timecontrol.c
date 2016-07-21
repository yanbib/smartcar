#include "timecontrol.h"

 /**
 * @brief  增加队列成员（内部）
 * @param  TIMqueue *Q 时间队列,taskqueue e任务队列

 * @retval ch_ok
 */
static int16_t EnQueue(TIMqueue *Q,taskqueue e)
{
  if ((Q->rear+1)%MAXSIZEqueue == Q->front)
    return CH_ERR;
  
  Q->data[Q->rear] = e;
  Q->rear = (Q->rear + 1)%MAXSIZEqueue;
  
  return CH_OK;

}


 /**
 * @brief  初始化队列（内部）
 * @param  TIMqueue *Q

 * @retval ch_ok
 */
static int16_t InitQueue(TIMqueue *Q)
{
  Q->front = 0;
  Q->rear  = 0;
  
  return CH_OK;
}


 /**
 * @brief  时间队列初始化
 * @param  TIMqueue *Q

 * @retval ch_ok
 */

int16_t timeCTL_TurntableInit(TIMqueue *Q)
{
    int8_t i;
    taskqueue k;
    
    if(InitQueue(Q)!=CH_OK) return CH_ERR;
    
    for(i=0;i<(MAXSIZEqueue-1);i++)
    {
        EnQueue(Q,k);
    }
    return CH_OK;
    
}


 /**
 * @brief  查找队列目标位置（内部）
 * @param  TIMqueue *Q  目标位置 flag

 * @retval 位置
 */
uint16_t timeCTL_findplace(TIMqueue *Q,uint8_t flag)
{
    uint16_t temp;
    temp = (Q->front + flag)%(MAXSIZEqueue);
    return temp;
}


 /**
 * @brief  取出当前队列位置成员taskqueue (内部)
 * @param  TIMqueue *Q

 * @retval ch_ok
 */
uint16_t timeCTL_timenext(TIMqueue *Q,taskqueue *e)
{
//    if(Q->front == Q->rear) return CH_ERR;
    
    *e = Q->data[Q->front];
    Q->front = (Q->front + 1)%MAXSIZEqueue;
    Q->rear = (Q->rear + 1)%MAXSIZEqueue;
    
    return CH_OK;

}

 /**
 * @brief  初始化任务队列
 * @param  TIMqueue *Q

 * @retval ch_ok
 */
int16_t timeCTL_TaskqueueInit(TIMqueue *Q)
{
    uint8_t i;
    for(i=0;i<(MAXSIZEqueue-1);i++)
    {
        Q->data[i].front=0;
        Q->data[i].rear=0;
    }
    return CH_OK;
}

 /**
 * @brief  任务队列执行
 * @param  TIMqueue *Q

 * @retval ch_ok
 */
int16_t timeCTL_TaskqueueExecute(TIMqueue *Q)
{
    
//    timeCTL_timenext(Q,e);
    ;
    Q->front = (Q->front + 1)%MAXSIZEqueue;
    Q->rear = (Q->rear + 1)%MAXSIZEqueue;
    while(Q->data[Q->front].front != Q->data[Q->front].rear)
    {
        
        Q->data[Q->front].appfun[Q->data[Q->front].front]();
        Q->data[Q->front].front = (Q->data[Q->front].front + 1)%MAXSIZEtask;
        
//        e->appfun[e->front]();
//        e->front = (e->front + 1)%MAXSIZEtask;      
    }
    return CH_OK;
}


 /**
 * @brief  任务插入时间队列
 * @param  TIMqueue *Q irqapp ,时间MS
 * @retval ch_ok
 */


int16_t timeCTL_TaskqueueInsert(TIMqueue *Q,Task_CallBackType irqapp,uint8_t ms)
{
    uint16_t temp;
    temp = timeCTL_findplace(Q,ms);
//    if(temp==0)temp=1;
    Q->data[temp].appfun[Q->data[temp].rear] = irqapp;
    Q->data[temp].rear = (Q->data[temp].rear + 1)%MAXSIZEtask;
    
    return CH_OK;

}

