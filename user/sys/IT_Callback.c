#include "IT_Callback.h"
#include "C_code_Log.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
// 观察者模式 （看不懂可以不看，会用就行）
/// @brief 通知所有观察者/订阅者
/// @param  this 把自己作为参数传入
void notify(Subject *this) {
  Observer *target = this->observer;
  while (target)
  {
    if (target != NULL && target->update !=NULL)
    {
      if(target->freq == 0)
      {
        target->update();
      }
      else
      {
        target->count++;
        if(target->count >= target->freq)
        {
          target->count = 0;
          target->update();
        }
      }
    }
    target = target->next;
  }
  
  // if (this->observer[i] != NULL && this->observer[i]->update !=NULL) {
  //   this->observer[i]->update();
  // }
}
/// @brief 添加观察者/订阅者
/// @param this 把自己作为参数传入
/// @param observer 要添加的观察者/订阅者
void addObserver(Subject *this, Observer *observer) {
  Observer *last_target = NULL;
  Observer *target = this->observer;
  while (target)
  {
    // 若发现优先级一致，则添加错误
    if(target->id == observer->id) {
      LOG_ERROR("添加观察者失败,id错误或者此id已存在, id = %d", observer->id);
      return;
    }
    // 当下一个优先级比要插入的小时，就进行插入
    if(target->id > observer->id)
    {
      // 检测是不是头节点
      if (last_target) {
        last_target->next = observer;
        observer->next = target;
      } else {
        observer->next = this->observer;
        this->observer = observer;
      }
      LOG_INFO("添加观察者成功, id = %d", observer->id);
      return ;
    }
    last_target = target;
    target = target->next;
  }
  // 检测是不是第一个加入的
  if (this->observer == NULL){
    this->observer = observer;
  } else {
    last_target->next = observer;
  }
  LOG_INFO("添加观察者成功, id = %d", observer->id);
  return ;
}

/// @brief 删除观察者/订阅者
/// @param  this 把自己作为参数传入
/// @param observer 要删除的观察者/订阅者
void removeObserver(Subject *this, Observer *observer) {
  Observer *target = this->observer;
  Observer *last_target = NULL;
  while (target)
  {
    if (target->id == observer->id) {
      // 查看target是不是头节点
      if (last_target) {
        last_target->next = target->next;
      } else {
        this->observer = target->next;
      }
      return;
    }
    last_target = target;
    target = target->next;
  }
}
//-------------------------TIM6 定时器中断主题-----------------------------------
Subject gl_TIM6_IT = {
    .notify = notify, .add = addObserver, .remove = removeObserver,.period=20};  /* TIM6 1ms 定时器中断主题, 可在其他地方订阅其消息 */
//-------------------------TIM6 定时器中断主题-----------------------------------

/// @brief 在轮询中使用tim6作为时钟来源，实现观察者模式
void TIM6_pollCall(uint32_t gl_time)
{
  static uint32_t lastTime = 0;
  uint32_t interval = gl_time - lastTime;//两次轮询的间隔
  if(interval>=gl_TIM6_IT.period)
  {
    if(interval>=gl_TIM6_IT.period*2)
    {
      LOG_ERROR("长时间未轮询到，严重超出预期,period = %dms, interval = %dms",gl_TIM6_IT.period,interval);
    }
    gl_TIM6_IT.notify(&gl_TIM6_IT);
    // TIM6_CNT_RESET;//必须通知完之后才能重置
		//LOG("INFO","period = %d, interval = %d",gl_TIM6_IT.period,interval);
    lastTime = gl_time;
  }
}
/*
 *观察者使用示例（电机）
 *   先定义一个观察者对象 Observer MotorPIDAdjustObserver = {.id = 2, .update =
 *  SpeedAdjustment_PID}; 在Init()函数或者其他地方中订阅TIM6_IT的主题
 *  gl_TIM6_IT.add(&gl_TIM6_IT, &MotorPIDAdjustObserver); 
 *  在需要的地方取消订阅
 *  gl_TIM6_IT.remove(&gl_TIM6_IT, &MotorPIDAdjustObserver);
 *  可以实现实时更改Observer的update函数，实现不同的功能
 *  更加方便地在中断中使用不同的功能，不用做很多标志位判断
 *
 *主题使用示例
 *   //定时器中断主题的定义
 *   Subject gl_TIM6_IT = {.notify = notify, .add = addObserver, .remove = removeObserver};
 *   主题的变量必须是以上几个，notify,add,remove，这是固定的。当然也可以自己实现，不建议
 *   gl_TIM6_IT.notify(&gl_TIM6_IT);//通知所有定时器6的订阅者
 * 不足:id原来不是用来做执行优先级的，但是后来发现有这个需求，因为id是不可重复的，所以优先级也是不可重复的
 * 优先级主要用来做一些有先后顺序的操作，比如说电机的速度读取和速度调节，读取应该在调节之前 
 * 改进思路：让id可以重复或者把id改个名字，然后还是用数组的形式，在add函数中不要直接将id做为数组下标，而是
 * 通过比较与之前已有的订阅者的id比较，然后使用插入的方法，由于现在能用，所以不改
 * 2025/2/19
 * 将数组形式修改为链表，通过id来调整插入顺序，目前id依然是不可重复，但还是可以通过添加另一个变量，比如说加入
 * 一个名称变量为name，该名称变量来作为唯一标识标志
 */

// 中断处理
/// @brief 定时器中断回调函数，在定时器中断中由HAL库自动调用
/// @param htim
/// @note
/// 实现了观察者模式，现在可以在其他地方订阅定时器中断的消息啦，在这里也只要通知所有订阅者就行了，他们自己会处理的
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//   if (htim == &htim6) {
//     //gl_TIM6_IT.notify(&gl_TIM6_IT);  // 通知TIM6中断的所有订阅者
//     overTimes++;
//   }
// }
