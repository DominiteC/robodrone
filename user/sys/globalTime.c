#include "globalTime.h"
//-------------------------全局时间溢出计数-----------------------------------
static volatile uint32_t overTimes = 0;  /* 系统滴答溢出计数 (每 1ms 递增, 与 TIM6 溢出周期同步) */
//-------------------------全局时间溢出计数-----------------------------------

/// @brief 获取全局时间
/// @return uint32_t 全局时间，只增不减，单位1ms，用49天才会溢出
uint32_t getGlobalTime(void)
{
  return overTimes;
}
void globalTimeCallBack(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {
    overTimes++;
  }
}
