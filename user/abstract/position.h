#ifndef __POSITION_H__
#define __POSITION_H__

#include <stdbool.h>
#include "vector_types.h"

#define DEG_TO_RAD (3.14159265f / 180.0f)  // 度转弧度

void position_init(void);
void position_GetHeight(float* height);
void position_GetVelocity(float_velocity* vel, const float_angle* angle, const float_gyro* gyro);
void position_GetPosition(float_xy_pos* pos);
void position_ResetXY(void);
bool position_IsXYFlowValid(void);
void position_ResetFlowState(void);

//-------------------------速度全局变量-----------------------------------
extern float_velocity velocity;             /* 飞行器三轴速度 (cm/s), 由 position 模块维护 */
//-------------------------速度全局变量-----------------------------------

//-------------------------Predict 调试变量 (供 ANO_DT 遥测)-----------------
extern float debug_acc_hx;                  /* 重力补偿后水平加速度 X (cm/s²), Predict 输入 */
extern float debug_acc_hy;                  /* 重力补偿后水平加速度 Y (cm/s²) */
extern float debug_flow_residual_x;         /* 光流位置残差 X (cm) */
//-------------------------Predict 调试变量-----------------------------------

#endif
