/*
 * vector_types.h
 * 定义无人机领域常用的向量、角度、加速度和二维位置类型。
 */
#ifndef __VECTOR_TYPES_H__
#define __VECTOR_TYPES_H__

typedef struct _float_velocity
{
	float x; // cm/s
	float y; // cm/s
	float z; // cm/s
} float_velocity;

typedef struct _float_angle
{
	float roll;
	float pitch;
	float yaw;
} float_angle;

typedef struct _float_acc
{
	float x;
	float y;
	float z;
} float_acc;

typedef struct _float_gyro
{
	float x;
	float y;
	float z;
} float_gyro;

typedef struct _float_xy_pos
{
	float x; // cm
	float y; // cm
} float_xy_pos;

#endif