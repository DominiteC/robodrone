#ifndef __CONTROL_OUTPUT_H__
#define __CONTROL_OUTPUT_H__

#include "actuator_types.h"

/*
 * 控制输出模块只负责把控制结果写入电调和电机驱动。
 * 控制算法仍留在控制核心模块中，避免硬件输出细节扩散。
 */
void MotorControl(MotorCtrl* ctrl);

#endif