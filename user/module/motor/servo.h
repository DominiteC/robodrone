#ifndef __SERVO_H_
#define __SERVO_H_
#include "tim.h"
#include <stdbool.h>

#define SERVO_NUM 4

#define SERVO_1_Tim htim2
#define SERVO_2_Tim htim2
#define SERVO_3_Tim htim2
#define SERVO_4_Tim htim2
// #define SERVO_5_Tim htim3
// #define SERVO_6_Tim htim3
// #define SERVO_7_Tim htim3
// #define SERVO_8_Tim htim3

#define SERVO_1_Channel TIM_CHANNEL_1
#define SERVO_2_Channel TIM_CHANNEL_2
#define SERVO_3_Channel TIM_CHANNEL_3
#define SERVO_4_Channel TIM_CHANNEL_4
// #define SERVO_5_Channel TIM_CHANNEL_1
// #define SERVO_6_Channel TIM_CHANNEL_2
// #define SERVO_7_Channel TIM_CHANNEL_3
// #define SERVO_8_Channel TIM_CHANNEL_4


// #define LEFT_FRONT_1 0
#define LEFT_FRONT_2 0
// #define LEFT_BACK_1 2
#define LEFT_BACK_2 1
// #define RIGHT_FRONT_1 4
#define RIGHT_FRONT_2 2
// #define RIGHT_BACK_1 6
#define RIGHT_BACK_2 3

//一些角度值
// 主控板的朝向
//     陆行 飞行      飞行 陆行
// 左上1 120 60   左上2 75 133
// 左下1 35 95    左下2 62 4
// 右上1 20 75  	 右上2 66 8
// 右下1 120 60   右下2 80 137

// #define LEFT_FRONT_1_INIT 60.5
#define LEFT_FRONT_2_INIT 63
// #define LEFT_BACK_1_INIT 95
#define LEFT_BACK_2_INIT 61
// #define RIGHT_FRONT_1_INIT 75
#define RIGHT_FRONT_2_INIT 68
// #define RIGHT_BACK_1_INIT 57
#define RIGHT_BACK_2_INIT 39

// #define LEFT_FRONT_1_UP (120+5)
#define LEFT_FRONT_2_UP 152//113
// #define LEFT_BACK_1_UP (35-5)
#define LEFT_BACK_2_UP 0//40
// #define RIGHT_FRONT_1_UP (20-5)
#define RIGHT_FRONT_2_UP 0//36
// #define RIGHT_BACK_1_UP (120+5)
#define RIGHT_BACK_2_UP 127//99

#define LEFT_FRONT_2_WALK ((LEFT_FRONT_2_INIT+LEFT_FRONT_2_UP)/2)
#define LEFT_BACK_2_WALK ((LEFT_BACK_2_INIT+LEFT_BACK_2_UP)/2)
#define RIGHT_FRONT_2_WALK ((RIGHT_FRONT_2_INIT+RIGHT_FRONT_2_UP)/2)
#define RIGHT_BACK_2_WALK ((RIGHT_BACK_2_INIT+RIGHT_BACK_2_UP)/2)

// #define LEFT_FRONT_1_HALF_UP ((LEFT_FRONT_1_INIT+LEFT_FRONT_1_UP)/2)
// #define LEFT_FRONT_2_HALF_UP ((LEFT_FRONT_1_INIT+LEFT_FRONT_1_UP)/2)
// #define LEFT_BACK_1_HALF_UP ((LEFT_FRONT_1_INIT+LEFT_FRONT_1_UP)/2)
// #define LEFT_BACK_2_HALF_UP ((LEFT_FRONT_1_INIT+LEFT_FRONT_1_UP)/2)
// #define RIGHT_FRONT_1_HALF_UP ((LEFT_FRONT_1_INIT+LEFT_FRONT_1_UP)/2)
// #define RIGHT_FRONT_2_HALF_UP ((LEFT_FRONT_1_INIT+LEFT_FRONT_1_UP)/2)
// #define RIGHT_BACK_1_HALF_UP ((LEFT_FRONT_1_INIT+LEFT_FRONT_1_UP)/2)
// #define RIGHT_BACK_2_HALF_UP ((LEFT_FRONT_1_INIT+LEFT_FRONT_1_UP)/2)

void ServoInit(void);
void setServoAngleByIndex(uint8_t servoNum, float angle);
bool setServoSlowByIndex(uint8_t servoNum, float angle, float tick_angle);
#endif  // !SERVO_H
