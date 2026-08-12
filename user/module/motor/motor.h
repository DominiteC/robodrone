#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "tim.h"
#include "gpio.h"

#define LEFT_FRONT_TIM_HANDLE       htim3
#define LEFT_FRONT_TIM_CHANNEL_1	TIM_CHANNEL_1
#define LEFT_FRONT_TIM_CHANNEL_2	TIM_CHANNEL_2

#define LEFT_BACK_TIM_HANDLE        htim3
#define LEFT_BACK_TIM_CHANNEL_1	    TIM_CHANNEL_3
#define LEFT_BACK_TIM_CHANNEL_2	    TIM_CHANNEL_4

#define RIGHT_FRONT_TIM_HANDLE      htim1
#define RIGHT_FRONT_TIM_CHANNEL_1	TIM_CHANNEL_1
#define RIGHT_FRONT_TIM_CHANNEL_2	TIM_CHANNEL_2

#define RIGHT_BACK_TIM_HANDLE       htim1
#define RIGHT_BACK_TIM_CHANNEL_1	TIM_CHANNEL_3
#define RIGHT_BACK_TIM_CHANNEL_2	TIM_CHANNEL_4

//用以改变电机的转动方向，在装车时如果把电机装反了可以从这里调节
#define LEFT_FRONT_CHANGE_DIRECTION 0
#define LEFT_BACK_CHANGE_DIRECTION 0
#define RIGHT_FRONT_CHANGE_DIRECTION 1
#define RIGHT_BACK_CHANGE_DIRECTION 1

void Motor_Init(void);
void Motor_Stop(void);
void Motor_Set_PWM(int16_t leftFrontPWM,int16_t leftBackPWM,int16_t rightFrontPWM,int16_t rightBackPWM);


#endif
