#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__

#include "gpio.h"
#include <stdbool.h>

#define GPIO_PORT_CAT(label) label##_GPIO_Port
#define GPIO_PIN_CAT(label) label##_Pin

#define ACTUATOR_1_1    IN1
#define ACTUATOR_1_2    IN2
#define ACTUATOR_2_1    IN3
#define ACTUATOR_2_2    IN4
#define ACTUATOR_3_1    IN5
#define ACTUATOR_3_2    IN6
#define ACTUATOR_4_1    IN7
#define ACTUATOR_4_2    IN8

#define ACTUATOR_1_1_Port    GPIO_PORT_CAT(IN1)
#define ACTUATOR_1_2_Port    GPIO_PORT_CAT(IN2)
#define ACTUATOR_2_1_Port    GPIO_PORT_CAT(IN3)
#define ACTUATOR_2_2_Port    GPIO_PORT_CAT(IN4)
#define ACTUATOR_3_1_Port    GPIO_PORT_CAT(IN5)
#define ACTUATOR_3_2_Port    GPIO_PORT_CAT(IN6)
#define ACTUATOR_4_1_Port    GPIO_PORT_CAT(IN7)
#define ACTUATOR_4_2_Port    GPIO_PORT_CAT(IN8)

#define ACTUATOR_1_1_Pin    GPIO_PIN_CAT(IN1)
#define ACTUATOR_1_2_Pin    GPIO_PIN_CAT(IN2)
#define ACTUATOR_2_1_Pin    GPIO_PIN_CAT(IN3)
#define ACTUATOR_2_2_Pin    GPIO_PIN_CAT(IN4)
#define ACTUATOR_3_1_Pin    GPIO_PIN_CAT(IN5)
#define ACTUATOR_3_2_Pin    GPIO_PIN_CAT(IN6)
#define ACTUATOR_4_1_Pin    GPIO_PIN_CAT(IN7)
#define ACTUATOR_4_2_Pin    GPIO_PIN_CAT(IN8)

#define LEFT_FRONT 4
#define LEFT_BACK 3
#define RIGHT_FRONT 1
#define RIGHT_BACK 2

void Actuator_Init(void);

void Actuator_Start(uint8_t actuator_num, bool state);
void Actuator_Stop(uint8_t actuator_num);

void Actuator_Set(uint8_t actuator_num, bool state, uint16_t delay_time);
void Actuator_Set_2(uint8_t actuator_num1,uint8_t actuator_num2, bool state, uint16_t delay_time);
void Actuator_AllSet(bool state, uint16_t delay_time);

#endif
