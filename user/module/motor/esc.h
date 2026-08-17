#ifndef __ESC_H__
#define __ESC_H__

#include "tim.h"

#define ESC_TIM_HANDLE	htim8
#define ESC_1_TIM_CHANNEL	TIM_CHANNEL_4
#define ESC_2_TIM_CHANNEL	TIM_CHANNEL_1
#define ESC_3_TIM_CHANNEL	TIM_CHANNEL_2
#define ESC_4_TIM_CHANNEL	TIM_CHANNEL_3

#define ESC_MIN_THROTTLE	1000	// 最小油门
#define ESC_MAX_THROTTLE	2000	// 最大油门

void ESC_Init(void);
void ESC_Set_PWM(uint16_t PWM_1,uint16_t PWM_2,uint16_t PWM_3,uint16_t PWM_4);
void ESC_Set_Percent(float percent_1,float percent_2,float percent_3,float percent_4);
void ESC_Test(void);

#endif
