#ifndef __GYRO_H_
#define __GYRO_H_
#include "stdint.h"
#include "stdbool.h"
#include "usart.h"
#include "jy901p.h"
#include "vector_types.h"
#define GYRO_USART_HANDLE &huart3

void gyro_init(void);
int8_t gyro_getData(void);
float gyro_getPitch(void);
float gyro_getRoll(void);
float gyro_getYaw(void);

void gyro_getAngle(float_angle* angle);
void gyro_getAcc(float_acc* acc);
void gyro_getAngularVelocity(float_gyro* gyro);

/* 上电后 1.0s 陀螺零偏自校准 (基于 MiniFly 思路) */
void gyro_calibrateGyroZOffset(void);
/* 校准是否通过: 0=未通过 (offset=0, yaw 会缓慢漂), 1=已通过 */
uint8_t gyro_isGyroZCalibrated(void);
/* 校准结果调试变量 (地面站遥测用, 避免 LOG_INFO 关中断导致 IWDG 复位) */
//-------------------------陀螺零偏调试变量-----------------------------------
extern float  gyro_cali_offset;             /* 校准后的 Z 轴零偏值 (地面站遥测) */
extern float  gyro_cali_var_z;              /* Z 轴采样方差 (地面站遥测) */
extern uint8_t gyro_cali_status;            /* 校准结果: 0=未完成, 1=通过, 2=失败 */
//-------------------------陀螺零偏调试变量-----------------------------------

#endif  // !__GYRO_H_
