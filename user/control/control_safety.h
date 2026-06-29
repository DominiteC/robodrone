#ifndef __CONTROL_SAFETY_H__
#define __CONTROL_SAFETY_H__

#include "actuator_types.h"
#include "vehicle_state.h"

/*
 * 控制安全模块负责飞控运行时保护检查。
 * 这里可以切断飞行开关、锁定遥控状态并拉低输出，但不放姿态/位置控制算法。
 */
void safeCheck(MotorCtrl* ctrl, state_t* state);

#endif