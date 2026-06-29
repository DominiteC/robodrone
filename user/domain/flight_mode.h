/*
 * flight_mode.h
 * 定义位置控制、飞控控制模式和姿态模式等领域枚举。
 */
#ifndef __FLIGHT_MODE_H__
#define __FLIGHT_MODE_H__

typedef enum {
	modeDisable = 0,
	modeAbs,
	modeVelocity
} PosMode;

typedef enum _ctrlMode {
	MODE_HEIGHT = 0,
	MODE_MANUAL,
	MODE_THREEHOLD,
} CtrlMode;

typedef enum _attitudeMode {
	MODE_AIRPLANE = 0,
	MODE_WALK,
	MODE_WALK_45,
} AttitudeMode;

#endif