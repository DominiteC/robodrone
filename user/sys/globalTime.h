#ifndef _GLOBAL_TIME_H
#define _GLOBAL_TIME_H 

#include "stdint.h"
#include "tim.h"


#define TIM_HANDLE_GLOBAL_TIME htim6
#define TIM6_CNT_RESET __HAL_TIM_SET_COUNTER(&htim6, 0)  // 重置定时器6的计数器
#define TIM6_CNT_GET __HAL_TIM_GET_COUNTER(&htim6)       // 获取定时器6的计数器的值
uint32_t getGlobalTime(void);
void globalTimeCallBack(TIM_HandleTypeDef *htim);

#endif // GLOBAL_TIME
