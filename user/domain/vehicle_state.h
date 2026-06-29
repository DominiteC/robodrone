/*
 * vehicle_state.h
 * 定义飞行器当前状态和控制目标设定值。
 */
#ifndef __VEHICLE_STATE_H__
#define __VEHICLE_STATE_H__

#include <stdbool.h>

#include "flight_mode.h"
#include "vector_types.h"

typedef struct
{
	float_velocity velocity;
	float_angle angle;
	float_acc acc;
	float_gyro gyro;
	float height;
	float_xy_pos position; // XY position integration (cm)
	bool isRCLocked;
} state_t;

typedef struct
{
	float_velocity vel;
	float_angle angle;
	float height;
	float thrust;
	float_xy_pos pos; // target XY position (cm)
	PosMode mode_x;
	PosMode mode_y;
	PosMode mode_z;
} setpoint_t;

#endif