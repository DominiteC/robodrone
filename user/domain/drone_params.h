#ifndef __DRONE_PARAMS_H__
#define __DRONE_PARAMS_H__

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

#endif