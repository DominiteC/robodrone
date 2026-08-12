#include "esc.h"
#include "C_code_Log.h"
#include "Mydelay.h"

#define max(a,b)			(a>b ? a:b)
#define min(a,b)			(a<b ? a:b)
#define limit(x,a,b)   		(min(max((x),(a)),(b)))

/**
 * @brief 电调初始化
 * 
 */
void ESC_Init(void)
{
	// 设置成最小油门
	__HAL_TIM_SetCompare(&ESC_TIM_HANDLE,ESC_1_TIM_CHANNEL,ESC_MIN_THROTTLE);
	__HAL_TIM_SetCompare(&ESC_TIM_HANDLE,ESC_2_TIM_CHANNEL,ESC_MIN_THROTTLE);
	__HAL_TIM_SetCompare(&ESC_TIM_HANDLE,ESC_3_TIM_CHANNEL,ESC_MIN_THROTTLE);
	__HAL_TIM_SetCompare(&ESC_TIM_HANDLE,ESC_4_TIM_CHANNEL,ESC_MIN_THROTTLE);

	// 开启PWM输出
	HAL_TIM_PWM_Start(&ESC_TIM_HANDLE,ESC_1_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&ESC_TIM_HANDLE,ESC_2_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&ESC_TIM_HANDLE,ESC_3_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&ESC_TIM_HANDLE,ESC_4_TIM_CHANNEL);
}

static void ESC_Set_Channel_PWM(uint32_t Channel, uint16_t PWM)
{
	PWM = limit(PWM,ESC_MIN_THROTTLE,ESC_MAX_THROTTLE);
	__HAL_TIM_SetCompare(&ESC_TIM_HANDLE,Channel,PWM);
}

/**
 * @brief 设置四个电调的PWM值
 * 
 * @param PWM_1 电调1的PWM值
 * @param PWM_2 电调2的PWM值
 * @param PWM_3 电调3的PWM值
 * @param PWM_4 电调4的PWM值
 */
void ESC_Set_PWM(uint16_t PWM_1,uint16_t PWM_2,uint16_t PWM_3,uint16_t PWM_4)
{
//	PWM_1 = limit(PWM_1,ESC_MIN_THROTTLE,ESC_MAX_THROTTLE);
//	PWM_2 = limit(PWM_2,ESC_MIN_THROTTLE,ESC_MAX_THROTTLE);
//	PWM_3 = limit(PWM_3,ESC_MIN_THROTTLE,ESC_MAX_THROTTLE);
//	PWM_4 = limit(PWM_4,ESC_MIN_THROTTLE,ESC_MAX_THROTTLE);
//	
//	LOG_INFO("PWM_1 = %d,PWM_2 = %d,PWM_3 = %d,PWM_4 = %d,",PWM_1,PWM_2,PWM_3,PWM_4);
//	__HAL_TIM_SetCompare(&ESC_TIM_HANDLE,ESC_1_TIM_CHANNEL,PWM_1);
//	__HAL_TIM_SetCompare(&ESC_TIM_HANDLE,ESC_2_TIM_CHANNEL,PWM_2);
//	__HAL_TIM_SetCompare(&ESC_TIM_HANDLE,ESC_3_TIM_CHANNEL,PWM_3);
//	__HAL_TIM_SetCompare(&ESC_TIM_HANDLE,ESC_4_TIM_CHANNEL,PWM_4);
	ESC_Set_Channel_PWM(ESC_1_TIM_CHANNEL,PWM_1);
	ESC_Set_Channel_PWM(ESC_2_TIM_CHANNEL,PWM_2);
	ESC_Set_Channel_PWM(ESC_3_TIM_CHANNEL,PWM_3);
	ESC_Set_Channel_PWM(ESC_4_TIM_CHANNEL,PWM_4);

}

/**
 * @brief 设置四个电调的油门百分比
 * 
 * @param percent_1 电调1的油门百分比(0\~100)
 * @param percent_2 电调2的油门百分比(0\~100)
 * @param percent_3 电调3的油门百分比(0\~100)
 * @param percent_4 电调4的油门百分比(0\~100)
 * 
 * @note 控制电调的函数最好只调用一次，以免错误调用
 */
void ESC_Set_Percent(float percent_1,float percent_2,float percent_3,float percent_4)
{
	uint16_t pulse_1 = (ESC_MAX_THROTTLE - ESC_MIN_THROTTLE) * limit(percent_1,0,95) * 0.01 + ESC_MIN_THROTTLE;
	uint16_t pulse_2 = (ESC_MAX_THROTTLE - ESC_MIN_THROTTLE) * limit(percent_2,0,95) * 0.01 + ESC_MIN_THROTTLE;
	uint16_t pulse_3 = (ESC_MAX_THROTTLE - ESC_MIN_THROTTLE) * limit(percent_3,0,95) * 0.01 + ESC_MIN_THROTTLE;
	uint16_t pulse_4 = (ESC_MAX_THROTTLE - ESC_MIN_THROTTLE) * limit(percent_4,0,95) * 0.01 + ESC_MIN_THROTTLE;
	// if (myDelay((uint32_t)ESC_Set_Percent,100))
	// 	LOG_DEBUG("ESC percent: %.2f%%,%.2f%%,%.2f%%,%.2f%% -> PWM: %d,%d,%d,%d",percent_1,percent_2,percent_3,percent_4,
	// 				pulse_1,pulse_2,pulse_3,pulse_4);
	ESC_Set_PWM(pulse_1,pulse_2,pulse_3,pulse_4);
}

/**
 * @brief 电调校准
 * 
 */
void ESC_Calibrate(void)
{
	// 设置成最大油门
	ESC_Set_PWM(ESC_MAX_THROTTLE,ESC_MAX_THROTTLE,ESC_MAX_THROTTLE,ESC_MAX_THROTTLE);
	LOG_INFO("ESC设置为最大油门,等待2秒之后开始校准");
	uint8_t delay_time = 4;
	while (delay_time--)
	{
		LOG_INFO("等待%d秒",delay_time+1);
		HAL_Delay(1000);
	}
	ESC_Set_PWM(ESC_MIN_THROTTLE,ESC_MIN_THROTTLE,ESC_MIN_THROTTLE,ESC_MIN_THROTTLE);
	LOG_INFO("ESC设置为最小油门,校准结束");
}


void ESC_Test(void)
{
	uint16_t pulse_10_percent = (ESC_MAX_THROTTLE - ESC_MIN_THROTTLE) * 0.10 + ESC_MIN_THROTTLE;
	uint16_t pulse_15_percent = (ESC_MAX_THROTTLE - ESC_MIN_THROTTLE) * 0.15 + ESC_MIN_THROTTLE;
	ESC_Set_Percent(0,0,0,0);
	LOG_INFO("ESC设置为最小油门,等待10秒之后开始测试");
	uint8_t delay_time = 10;
	while (delay_time--)
	{
		LOG_INFO("等待%d秒",delay_time+1);
		HAL_Delay(1000);
	}
	ESC_Set_Percent(10,10,10,10);
	LOG_INFO("ESC设置为10%%油门,等待5秒");
	HAL_Delay(5000);
	ESC_Set_Percent(15,15,15,15);
	LOG_INFO("ESC设置为15%%油门,等待5秒");
	HAL_Delay(5000);
	ESC_Set_Percent(0,0,0,0);
	LOG_INFO("ESC设置为最小油门,测试结束");
}
