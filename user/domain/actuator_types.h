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