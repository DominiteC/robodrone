#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "PIDcontroller.h"
#include "drone_types.h"

#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)

#define MAIN_LOOP_RATE 			RATE_1000_HZ
#define MAIN_LOOP_DT			(u32)(1000 / MAIN_LOOP_RATE)

#define ATTITUDE_ESTIMAT_RATE	RATE_250_HZ
#define ATTITUDE_ESTIMAT_DT		(1.0 / RATE_250_HZ)

#define POSITION_ESTIMAT_RATE	RATE_250_HZ
#define POSITION_ESTIMAT_DT		(1.0 / RATE_250_HZ)

#define RATE_PID_RATE			RATE_500_HZ
#define RATE_PID_DT				(1.0 / RATE_500_HZ)

#define ANGEL_PID_RATE			ATTITUDE_ESTIMAT_RATE
#define ANGEL_PID_DT			(1.0 / ATTITUDE_ESTIMAT_RATE)

#define VELOCITY_PID_RATE		POSITION_ESTIMAT_RATE
#define VELOCITY_PID_DT			(1.0 / POSITION_ESTIMAT_RATE)

#define POSITION_PID_RATE		POSITION_ESTIMAT_RATE
#define POSITION_PID_DT			(1.0 / POSITION_ESTIMAT_RATE)

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

extern float debug_xy_velocity_pid_count;
extern float debug_target_angle_pitch;
extern float debug_target_angle_roll;
extern float debug_target_angle_yaw;
extern state_t state;
extern MotorCtrl debugEsc;

void Control_Init(void);
void ResetFlightControlPIDs(void);
float getAltholdThrust(void);
float getThrustCmd(void);
void Control_Task(void *param);

#endif
