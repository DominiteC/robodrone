#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "control_pid.h"
#include "drone_types.h"
#include "control_rates.h"


extern float debug_xy_velocity_pid_count;
extern float debug_target_angle_pitch;
extern float debug_target_angle_roll;
extern float debug_target_angle_yaw;
extern state_t state;
extern MotorCtrl debugEsc;

void ResetFlightControlPIDs(void);
float getAltholdThrust(void);
float getThrustCmd(void);
void Control_Task(void *param);

#endif
