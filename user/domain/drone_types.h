#ifndef __DRONE_TYPES_H__
#define __DRONE_TYPES_H__

#include <stdbool.h>
#include <stdint.h>

typedef struct _float_velocity
{
	float x;	// cm/s
	float y;	// cm/s
	float z;	// cm/s
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
	float x;	// cm
	float y;	// cm
} float_xy_pos;

typedef struct _DroneParams
{
	float Dia;
	float rho;
	float mass;
	float Jxx;
	float Jyy;
	float Jzz;
	float r_x;
	float r_y;
	float r_z;
	float thrust_coef;
	float torque_coef;
	float arm_len_x;
	float arm_len_y;
} DroneParams;

typedef enum {
	modeDisable = 0,
	modeAbs,
	modeVelocity
} PosMode;

typedef enum _ctrlMode {
	MODE_HEIGHT = 0,
	MODE_MANUAL,
	MODE_THREEHOLD,
} CtrlMode;

typedef enum _attitudeMode {
	MODE_AIRPLANE = 0,
	MODE_WALK,
	MODE_WALK_45,
} AttitudeMode;

typedef struct
{
	float_velocity velocity;
	float_angle angle;
	float_acc acc;
	float_gyro gyro;
	float height;
	float_xy_pos position;  // XY position integration (cm)
	bool isRCLocked;
} state_t;

typedef struct
{
	float_velocity vel;
	float_angle angle;
	float height;
	float thrust;
	float_xy_pos pos;       // target XY position (cm)
	PosMode mode_x;
	PosMode mode_y;
	PosMode mode_z;
} setpoint_t;

typedef struct _motorCtrl {
	float Esc_Percent_1;
	float Esc_Percent_2;
	float Esc_Percent_3;
	float Esc_Percent_4;
	int16_t Motor_Left_Front_PWM;
	int16_t Motor_Left_Back_PWM;
	int16_t Motor_Right_Front_PWM;
	int16_t Motor_Right_Back_PWM;
} MotorCtrl;

#endif
