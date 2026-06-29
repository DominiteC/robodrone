/*
 * actuator_types.h
 * 定义电机、电调和执行器输出相关的领域数据结构。
 */
#ifndef __ACTUATOR_TYPES_H__
#define __ACTUATOR_TYPES_H__

#include <stdint.h>

typedef struct _motorCtrl {
	float Esc_Percent_1;
	float Esc_Percent_2;
	float Esc_Percent_3;
	float Esc_Percent_4;
	int16_t Motor_Left_Front_PWM;
	int16_t Motor_Left_Back_PWM;
	int16_t Motor_Right_Front_PWM;
	int16_t Motor_Right_Back_PWM;
} MotorCtrl;

#endif