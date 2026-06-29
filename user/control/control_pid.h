#ifndef __CONTROL_PID_H__
#define __CONTROL_PID_H__

#include "PIDcontroller.h"

/*
 * 控制 PID 模块集中维护飞控 PID 实例和初始化入口。
 * PID 参数仍保持原值，避免参数配置散落在控制任务主文件中。
 */
extern PIDInstance pid_height_position;
extern PIDInstance pid_x_position;
extern PIDInstance pid_y_position;

extern PIDInstance pid_x_velocity;
extern PIDInstance pid_y_velocity;
extern PIDInstance pid_z_velocity;

extern PIDInstance pid_roll_angle;
extern PIDInstance pid_pitch_angle;
extern PIDInstance pid_yaw_angle;

extern PIDInstance pid_roll_rate;
extern PIDInstance pid_pitch_rate;
extern PIDInstance pid_yaw_rate;

void Control_Init(void);

#endif