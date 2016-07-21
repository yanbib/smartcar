
#include "QUEUEyan.h"
/* ==========================================================
*     开发人员：yan
*     编写时间：2016/5/28
*     函数名称：InitQueue(SqQueue *Q)
*     参数说明：SqQueue *Q 需要清空的队列
*     功能说明：清空队列

*/

int16_t InitQueue(SqQueue *Q)
{
  Q->front = 0;
  Q->rear  = 0;
  
  return CH_OK;
}
/* ==========================================================
*     开发人员：yan
*     编写时间：2016/5/28
*     函数名称：int16_t QueueLength(SqQueue Q)
*     参数说明：
*     功能说明：获取队列长度

*/
int16_t QueueLength(SqQueue Q)
{
  return (Q.rear - Q.front+MAXSIZEQUE)%MAXSIZEQUE;
}

/* ==========================================================
*     开发人员：yan
*     编写时间：2016/5/28
*     函数名称：int16_t EnQueue(SqQueue *Q,QElemType e)
*     参数说明：1 队列名
*               2 添加数据
*     功能说明：添加元素

*/
int16_t EnQueue(SqQueue *Q,QElemTypeint e)
{
  if ((Q->rear+1)%MAXSIZEQUE == Q->front)
    return CH_ERR;
  
  Q->data[Q->rear] = e;
  Q->rear = (Q->rear + 1)%MAXSIZEQUE;
  
  return CH_OK;

}

/* ==========================================================
*     开发人员：yan
*     编写时间：2016/5/28
*     函数名称：int16_t DeQueue(SqQueue *Q,QElemTypeint *e)
*     参数说明：1 队列名
*               2 返回删除的数据
*     功能说明：队列满，删除并返回队列首

*/
int16_t DeQueue(SqQueue *Q)
{
  if(Q->front == Q->rear)
    return CH_ERR;
  
//  *e=Q->data[Q->front];
  Q->front = (Q->front + 1)%MAXSIZEQUE;
  
  return CH_OK;

}


