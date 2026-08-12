#ifndef __CHANGE_H__
#define __CHANGE_H__

#include "control.h"

typedef enum _servoMode {
	SERVO_AIRPLANE = 0,	// 将舵机切换为飞行状态
	SERVO_WALK_45,		// 将舵机切换为45度陆行状态
	SERVO_WALK,			// 将舵机切换为陆行状态
} ServoMode;

typedef enum _servoAngle {
	SERVO_INIT = 0,
	// SERVO_HALF_UP,
	SERVO_WALK_2,
	SERVO_UP,
} ServoAngle;

ServoMode getServoMode(void);
void initServoMode(ServoMode mode);
void changeAttitude(MotorCtrl* ctrl, state_t* state);

#endif
