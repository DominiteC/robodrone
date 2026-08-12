#ifndef __POSITION_H__
#define __POSITION_H__

#include "structConfig.h"

#define DEG_TO_RAD (3.14159265f / 180.0f)  // 度转弧度

void position_init(void);
void position_GetHeight(float* height);
void position_GetVelocity(float_velocity* vel, const float_angle* angle, const float_gyro* gyro);
void position_GetPosition(float_xy_pos* pos);
void position_ResetXY(void);

#endif
