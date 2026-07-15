#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "control_pid.h"
#include "drone_types.h"
#include "control_rates.h"


extern float debug_xy_velocity_pid_count;
extern float debug_target_angle_pitch;
extern float debug_target_angle_roll;
extern float debug_target_angle_yaw;
extern float debug_desired_yaw;
extern float yaw_meas_cont;           /* 飞控自积分 yaw 角度, 供地面站遥测 */
extern state_t state;
extern MotorCtrl debugEsc;

void ResetFlightControlPIDs(void);
float getAltholdThrust(void);
float getThrustCmd(void);
void Control_Task(void *param);

#endif
