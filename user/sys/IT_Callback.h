#ifndef IT_CALLBACK_H
#define IT_CALLBACK_H
//更改： 2024/1/4
//新增使用轮询的方式来实现观察者模式
#include "stdint.h"
#include"globalTime.h"
typedef struct _observer {
  int id;  // 观察者id,id又表示运行的顺序，从0开始到OBSERVER_LIST_MAX-1
  void (*update)(void);
  //新增 2024/1/7
  uint16_t freq;//通知的频率，表示每隔多少次通知一次，单位：次，正常情况下为0，表示每次都通知,1和0都表示每隔一次通知一次，以此类推
  uint16_t count;//通知的计数器，每次通知都会加1，当count>=freq时，才会通知，然后count清零

  struct _observer* next;
} Observer;
#define OBSERVER_LIST_MAX 10  // 观察者列表最大长度
typedef struct Subject {
  Observer* observer;  // 观察者列表
  void (*notify)(struct Subject *this);
  void (*add)(struct Subject *this, Observer *observer);
  void (*remove)(struct Subject *this, Observer *observer);
  //新增 2024/1/7
  uint32_t period;//通知的周期，用于与定时器的值作对比，单位：与定时器的分频设置相关
} Subject;
extern Subject gl_TIM6_IT;//定时器中断主题,可以在其他地方订阅他的消息
void TIM6_pollCall(uint32_t gl_time);

#endif  // !IT_CALLBACK_H

