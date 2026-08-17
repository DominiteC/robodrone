#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "control_pid.h"
#include "drone_types.h"
#include "control_rates.h"


//-------------------------地面站观测数据 (debug 变量)-----------------------------------
extern float debug_xy_velocity_pid_count;   /* XY 速度 PID 执行计数, 验证 PID 执行频率 */
extern float debug_target_angle_pitch;      /* 俯仰角度目标镜像, 供地面站遥测 */
extern float debug_target_angle_roll;       /* 横滚角度目标镜像, 供地面站遥测 */
extern float debug_target_angle_yaw;        /* 目标 yaw 角度 (Desired_yaw 镜像), 供地面站遥测 */
extern float debug_yaw_rate_target;         /* 最终 yaw 目标角速度 deg/s */
extern float debug_yaw_meas_cont;           /* yaw_meas_cont 镜像, 供地面站遥测 */
extern float debug_pos_pid_out_x;           /* X 位置环 PID 输出速度指令 (cm/s), 拨杆时=0 */
extern float debug_pos_pid_out_y;           /* Y 位置环 PID 输出速度指令 (cm/s), 拨杆时=0 */
extern float debug_roll_rate_target;        /* 横滚角速度目标 (deg/s), 角度环输出 */
extern float debug_pitch_rate_target;       /* 俯仰角速度目标 (deg/s), 角度环输出 */
//-------------------------地面站观测数据 (debug 变量)-----------------------------------

//-------------------------控制输出与状态-----------------------------------
extern state_t state;                       /* 飞行器当前状态 (全局可见) */
extern MotorCtrl debugEsc;                  /* 电机控制输出调试镜像 */
extern setpoint_t target;                   /* 控制目标集合 (全局可见) */
//-------------------------控制输出与状态-----------------------------------

/* PID 复位级别：不同飞行阶段需要清零的组合不同 */
typedef enum {
    FLIGHT_RESET_FULL,   /* 起飞：全部 PID + position XY + yaw 状态 */
    FLIGHT_RESET_LAND,   /* 降落/停机：全部 PID（不含 position 复位） */
    FLIGHT_RESET_YAW,    /* yaw 杆回中：只清 yaw 角度/角速度环积分 */
} FlightResetLevel;

void flightReset(FlightResetLevel level);

/* 清 XY 位置环积分：摇杆回中锁位置时调用，防止模式切换积分残留 */
void flightClearPosPID(void);
float getAltholdThrust(void);
float getThrustCmd(void);
void Control_Task(void *param);

#endif
