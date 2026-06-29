/*
 * alarm.h
 * 声明告警服务接口，供 app、control 和遥控数据服务调用。
 */
#ifndef __ALARM_H__
#define __ALARM_H__


#include <stdbool.h>
#include "gpio.h"

#define BUZZER_GPIO_PORT        buzzer_GPIO_Port
#define BUZZER_GPIO_PIN         buzzer_Pin

#define BUZZER_ON()             HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_SET)
#define BUZZER_OFF()            HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_RESET)

#define LED_1_GPIO_PORT         LED_1_GPIO_Port	// 蓝色
#define LED_1_GPIO_PIN          LED_1_Pin
#define LED_2_GPIO_PORT         LED_2_GPIO_Port	// 绿色
#define LED_2_GPIO_PIN          LED_2_Pin
#define LED_3_GPIO_PORT         LED_3_GPIO_Port	// 红色
#define LED_3_GPIO_PIN          LED_3_Pin

#define LED_1_ON()             HAL_GPIO_WritePin(LED_1_GPIO_PORT, LED_1_GPIO_PIN, GPIO_PIN_RESET)
#define LED_1_OFF()            HAL_GPIO_WritePin(LED_1_GPIO_PORT, LED_1_GPIO_PIN, GPIO_PIN_SET)
#define LED_2_ON()             HAL_GPIO_WritePin(LED_2_GPIO_PORT, LED_2_GPIO_PIN, GPIO_PIN_RESET)
#define LED_2_OFF()            HAL_GPIO_WritePin(LED_2_GPIO_PORT, LED_2_GPIO_PIN, GPIO_PIN_SET)
#define LED_3_ON()             HAL_GPIO_WritePin(LED_3_GPIO_PORT, LED_3_GPIO_PIN, GPIO_PIN_RESET)
#define LED_3_OFF()            HAL_GPIO_WritePin(LED_3_GPIO_PORT, LED_3_GPIO_PIN, GPIO_PIN_SET)

#define BATTERY_SWITCH_GPIO_PORT    control_GPIO_Port
#define BATTERY_SWITCH_GPIO_PIN     control_Pin

#define BATTERY_VOLTAGE_ID      0
#define BATTERY_CURRENT_ID      1

#define BATTERY_VOLTAGE_RATE   (22.2/3294)  // 分压比 电池电压22.2V时,adc读数约为3294
#define BATTERY_CURRENT_RATE   (100.0/4095)   // 继电器输出1A对应,adc读数4095

#define GET_BATTERY_VOLTAGE(array)   (array[BATTERY_VOLTAGE_ID] * BATTERY_VOLTAGE_RATE)
#define GET_BATTERY_CURRENT(array)   (array[BATTERY_CURRENT_ID] * BATTERY_CURRENT_RATE)

typedef enum _alarm_mode {
    ALARM_MODE_LOCKED = 0,
    ALARM_MODE_UNLOCKED,
    ALARM_MODE_LOW_BATTERY,
    ALARM_MODE_ERROR,
} AlarmMode;

void Alarm_Init(void);
void Alarm_Update(void *param);
void Alarm_SetMode(AlarmMode mode);
float Alarm_GetBatteryVoltage(void);
float Alarm_GetBatteryCurrent(void);
void Alarm_SetBatterySwitch(bool state);
void Alarm_SetBatteryToggle(void);
void LED_Test(void);


#endif
