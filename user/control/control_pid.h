#ifndef __CONTROL_PID_H__
#define __CONTROL_PID_H__

/*
 * 控制 PID 模块集中维护飞控 PID 实例和初始化入口。
 * PID 访问规范：外部代码通过 pid_dump/pid_get_config/pid_set_config 访问 PID，
 *   禁止直接读写 PIDInstance 字段。
 */

#include "PIDcontroller.h"

/*
 * 控制 PID 模块集中维护飞控 PID 实例和初始化入口。
 * PID 参数仍保持原值，避免参数配置散落在控制任务主文件中。
 */
//-------------------------高度位置环-----------------------------------
extern PIDInstance pid_height_position;     /* 高度位置环 PID (输入:高度误差, 输出:垂直速度目标) */
//-------------------------高度位置环-----------------------------------
//-------------------------X/Y 位置环-----------------------------------
extern PIDInstance pid_x_position;          /* X 位置环 PID (输入:位置误差, 输出:X 速度目标) */
extern PIDInstance pid_y_position;          /* Y 位置环 PID (输入:位置误差, 输出:Y 速度目标) */
//-------------------------X/Y 位置环-----------------------------------

//-------------------------X/Y/Z 速度环-----------------------------------
extern PIDInstance pid_x_velocity;          /* X 轴速度环 PID (输入:速度误差, 输出:俯仰角度) */
extern PIDInstance pid_y_velocity;          /* Y 轴速度环 PID (输入:速度误差, 输出:横滚角度) */
extern PIDInstance pid_z_velocity;          /* Z 轴速度环 PID (输入:速度误差, 输出:油门修正) */
//-------------------------X/Y/Z 速度环-----------------------------------

//-------------------------角度环-----------------------------------
extern PIDInstance pid_roll_angle;          /* 横滚角度环 PID (输入:角度误差, 输出:角速度目标) */
extern PIDInstance pid_pitch_angle;         /* 俯仰角度环 PID (输入:角度误差, 输出:角速度目标) */
extern PIDInstance pid_yaw_angle;           /* 航向角度环 PID (输入:角度误差, 输出:角速度目标) */
//-------------------------角度环-----------------------------------

//-------------------------角速度环-----------------------------------
extern PIDInstance pid_roll_rate;           /* 横滚角速度环 PID (输入:角速度误差, 输出:电机混控) */
extern PIDInstance pid_pitch_rate;          /* 俯仰角速度环 PID (输入:角速度误差, 输出:电机混控) */
extern PIDInstance pid_yaw_rate;            /* 航向角速度环 PID (输入:角速度误差, 输出:电机混控) */
//-------------------------角速度环-----------------------------------

void Control_Init(void);

#endif