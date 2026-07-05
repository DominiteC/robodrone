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

#endif  // !__GYRO_H_
