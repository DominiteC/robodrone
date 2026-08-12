#include "motor.h"

#define max(a,b)			(a>b ? a:b)
#define min(a,b)			(a<b ? a:b)
#define limit(x,a,b)		(min(max(x,a) , b))

/**
 * @brief 电机初始化
 * 
 */
void Motor_Init(void)
{
	__HAL_TIM_SetCompare(&LEFT_FRONT_TIM_HANDLE,LEFT_FRONT_TIM_CHANNEL_1,	0);
	__HAL_TIM_SetCompare(&LEFT_FRONT_TIM_HANDLE,LEFT_FRONT_TIM_CHANNEL_2,	0);
	__HAL_TIM_SetCompare(&LEFT_BACK_TIM_HANDLE,LEFT_BACK_TIM_CHANNEL_1,		0);
	__HAL_TIM_SetCompare(&LEFT_BACK_TIM_HANDLE,LEFT_BACK_TIM_CHANNEL_2,		0);
	__HAL_TIM_SetCompare(&RIGHT_FRONT_TIM_HANDLE,RIGHT_FRONT_TIM_CHANNEL_1,	0);
	__HAL_TIM_SetCompare(&RIGHT_FRONT_TIM_HANDLE,RIGHT_FRONT_TIM_CHANNEL_2,	0);
	__HAL_TIM_SetCompare(&RIGHT_BACK_TIM_HANDLE,RIGHT_BACK_TIM_CHANNEL_1,	0);
	__HAL_TIM_SetCompare(&RIGHT_BACK_TIM_HANDLE,RIGHT_BACK_TIM_CHANNEL_2,	0);

	HAL_TIM_PWM_Start(&LEFT_FRONT_TIM_HANDLE,LEFT_FRONT_TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&LEFT_FRONT_TIM_HANDLE,LEFT_FRONT_TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&LEFT_BACK_TIM_HANDLE,LEFT_BACK_TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&LEFT_BACK_TIM_HANDLE,LEFT_BACK_TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&RIGHT_FRONT_TIM_HANDLE,RIGHT_FRONT_TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&RIGHT_FRONT_TIM_HANDLE,RIGHT_FRONT_TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&RIGHT_BACK_TIM_HANDLE,RIGHT_BACK_TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&RIGHT_BACK_TIM_HANDLE,RIGHT_BACK_TIM_CHANNEL_2);
}

/**
 * @brief 停止电机
 * 
 */
void Motor_Stop(void)
{
	__HAL_TIM_SetCompare(&LEFT_FRONT_TIM_HANDLE,LEFT_FRONT_TIM_CHANNEL_1,	0);
	__HAL_TIM_SetCompare(&LEFT_FRONT_TIM_HANDLE,LEFT_FRONT_TIM_CHANNEL_2,	0);
	__HAL_TIM_SetCompare(&LEFT_BACK_TIM_HANDLE,LEFT_BACK_TIM_CHANNEL_1,		0);
	__HAL_TIM_SetCompare(&LEFT_BACK_TIM_HANDLE,LEFT_BACK_TIM_CHANNEL_2,		0);
	__HAL_TIM_SetCompare(&RIGHT_FRONT_TIM_HANDLE,RIGHT_FRONT_TIM_CHANNEL_1,	0);
	__HAL_TIM_SetCompare(&RIGHT_FRONT_TIM_HANDLE,RIGHT_FRONT_TIM_CHANNEL_2,	0);
	__HAL_TIM_SetCompare(&RIGHT_BACK_TIM_HANDLE,RIGHT_BACK_TIM_CHANNEL_1,	0);
	__HAL_TIM_SetCompare(&RIGHT_BACK_TIM_HANDLE,RIGHT_BACK_TIM_CHANNEL_2,	0);
}

/**
 * @brief 设置单个电机的PWM值
 * 
 * @param htim 定时器句柄
 * @param channel1 通道1
 * @param channel2 通道2
 * @param PWM PWM值，正负表示转动方向
 * @param change_direction 是否改变方向
 */
static void Motor_Set(TIM_HandleTypeDef* htim, uint32_t channel1, uint32_t channel2, int16_t PWM, uint8_t change_direction)
{
	if (change_direction == 0)
	{
		if (PWM > 0)
		{
			__HAL_TIM_SetCompare(htim,channel1,	PWM);
			__HAL_TIM_SetCompare(htim,channel2,	0);
		}
		else
		{
			__HAL_TIM_SetCompare(htim,channel1,	0);
			__HAL_TIM_SetCompare(htim,channel2,	-PWM);
		}
	}
	else
	{
		if (PWM > 0)
		{
			__HAL_TIM_SetCompare(htim,channel1,	0);
			__HAL_TIM_SetCompare(htim,channel2,	PWM);
		}
		else
		{
			__HAL_TIM_SetCompare(htim,channel1,	-PWM);
			__HAL_TIM_SetCompare(htim,channel2,	0);
		}
	}
}

static void Motor_Set_Left_Front(int16_t PWM)
{
	Motor_Set(&LEFT_FRONT_TIM_HANDLE,LEFT_FRONT_TIM_CHANNEL_1,LEFT_FRONT_TIM_CHANNEL_2,PWM,LEFT_FRONT_CHANGE_DIRECTION);
}

static void Motor_Set_Left_Back(int16_t PWM)
{
	Motor_Set(&LEFT_BACK_TIM_HANDLE,LEFT_BACK_TIM_CHANNEL_1,LEFT_BACK_TIM_CHANNEL_2,PWM,LEFT_BACK_CHANGE_DIRECTION);
}

static void Motor_Set_Right_Front(int16_t PWM)
{
	Motor_Set(&RIGHT_FRONT_TIM_HANDLE,RIGHT_FRONT_TIM_CHANNEL_1,RIGHT_FRONT_TIM_CHANNEL_2,PWM,RIGHT_FRONT_CHANGE_DIRECTION);
}

static void Motor_Set_Right_Back(int16_t PWM)
{
	Motor_Set(&RIGHT_BACK_TIM_HANDLE,RIGHT_BACK_TIM_CHANNEL_1,RIGHT_BACK_TIM_CHANNEL_2,PWM,RIGHT_BACK_CHANGE_DIRECTION);
}

/**
 * @brief 设置四个电机的PWM值
 * 
 * @param leftFrontPWM 左前电机PWM值
 * @param leftBackPWM 左后电机PWM值
 * @param rightFrontPWM 右前电机PWM值
 * @param rightBackPWM 右后电机PWM值
 */
void Motor_Set_PWM(int16_t leftFrontPWM,int16_t leftBackPWM,int16_t rightFrontPWM,int16_t rightBackPWM)
{
	leftFrontPWM = limit(leftFrontPWM,-4000,4000);
	leftBackPWM = limit(leftBackPWM,-4000,4000);
	rightFrontPWM = limit(rightFrontPWM,-4000,4000);
	rightBackPWM = limit(rightBackPWM,-4000,4000);
	Motor_Set_Left_Front(leftFrontPWM);
	Motor_Set_Left_Back(leftBackPWM);
	Motor_Set_Right_Front(rightFrontPWM);
	Motor_Set_Right_Back(rightBackPWM);
}

