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


#endif  // !__GYRO_H_
