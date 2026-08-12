#include "globalTime.h"
static volatile uint32_t overTimes =0;//溢出次数,每1ms溢出一次，与TIM6的溢出时间有关

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
