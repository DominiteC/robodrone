/**
 * @file servo.c
 * @author Sevenfite (lin481399413@163.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-05
 * 
 * @interface 
 * 接口函数：
 *  ServoInit() 舵机初始化,关于舵机的一些参数设置需要改变此函数源码
 *  setServoAngleByIndex(uint8_t servoNum, uint16_t angle) 通过index设置舵机角度
 * @note
 *  Servo结构体存储了舵机的信息，包括舵机编号，设置的角度，最大角度，模式（模拟或数字），
 * 以及对应的GPIO（数字模式下）或者TIM(模拟模式下)
 *  本地全局变量gl_servo[SERVO_NUM]存储了所有舵机的信息
 * @note 
 * 关于数字舵机和模拟舵机的区别：
 * 数字舵机：舵机内置芯片，记录角度信息，所以只用发一次PWM信号，
 * 通过GPIO控制，每次设置角度时，需要给舵机一个脉冲，脉冲的高电平时间决定了舵机的角度，一般来说一个脉冲就可以了，如果不行，可以增加
 * 模拟舵机：传统的舵机，需要不断的发PWM信号，通过TIM控制，每次设置角度时，需要不断的发PWM信号，直到舵机转到指定角度，
 * 定时器的参数，PWM频率为50Hz，周期为20ms，高电平时间为0.5ms-2.5ms，对应角度为0-180度
 * 
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "servo.h"
#include "tim.h"
#include <stdlib.h>
#include "math.h"

enum ServoMode{
  ANALOG_SERVO, // 模拟舵机
  DIGITAL_SERVO // 数字舵机
};
struct Servo{
  uint8_t servoNum;
  float angle;
  uint16_t maxAngle;
  enum ServoMode mode; // 0:模拟舵机 1:数字舵机
  union
  {
    GPIO_TypeDef *GPIOx; // 数字舵机用
    TIM_HandleTypeDef *htim; //模拟舵机用
    /* data */
  }Info1;
  union 
  {
    uint32_t channel; // 模拟舵机用
    uint16_t pin; // 数字舵机用
    /* data */
  }Info2;
};

//-------------------------舵机全局数组-----------------------------------
struct Servo gl_servo[SERVO_NUM];  /* 舵机全局数组 (存储各舵机编号/角度/模式/TIM或GPIO参数) */
//-------------------------舵机全局数组-----------------------------------
static uint16_t calServoAngle(float angle, uint16_t maxAngle);
static void setServoAngle(struct Servo* servo, float angle);
static void setServo(struct Servo *servo,uint8_t servoNum, uint16_t angle, uint16_t maxAngle, enum ServoMode mode, void *Info1, uint32_t Info2){
  servo->servoNum = servoNum;
  servo->angle = angle;
  servo->maxAngle = maxAngle;
  servo->mode = mode;
  if(mode == DIGITAL_SERVO){
    servo->Info1.GPIOx = (GPIO_TypeDef *)Info1;
    servo->Info2.pin = (uint16_t)Info2;
  }else{
    servo->Info1.htim = (TIM_HandleTypeDef *)Info1;
    servo->Info2.channel = Info2;
  }
}
/**
 * @brief 舵机初始化函数
 * 
 */
void ServoInit(void)
{
  // 舵机初始化，设置舵机编号，初始角度，最大角度，模式，对应的GPIO或者TIM，如需更改舵机参数，需要改变这里的源码
  setServo(&gl_servo[0],1,0,180,ANALOG_SERVO,&SERVO_1_Tim,SERVO_1_Channel);
  setServo(&gl_servo[1],2,0,180,ANALOG_SERVO,&SERVO_2_Tim,SERVO_2_Channel);
  setServo(&gl_servo[2],3,0,180,ANALOG_SERVO,&SERVO_3_Tim,SERVO_3_Channel);
  setServo(&gl_servo[3],4,0,180,ANALOG_SERVO,&SERVO_4_Tim,SERVO_4_Channel);

  //初始化角度
  gl_servo[LEFT_FRONT_2].angle=LEFT_FRONT_2_INIT;
  gl_servo[LEFT_BACK_2].angle=LEFT_BACK_2_INIT;
  gl_servo[RIGHT_FRONT_2].angle=RIGHT_FRONT_2_INIT;
  gl_servo[RIGHT_BACK_2].angle=RIGHT_BACK_2_INIT;

  for(int i = 0; i < SERVO_NUM; i++){
    setServoAngle(&gl_servo[i],gl_servo[i].angle);
  }
  for(int i = 0; i < SERVO_NUM; i++){
    if(gl_servo[i].mode == ANALOG_SERVO){
      HAL_TIM_PWM_Start(gl_servo[i].Info1.htim,gl_servo[i].Info2.channel);
//		HAL_Delay(200);
    }
  }
}
void Servo_Uart_Init(uint8_t index)
{
	HAL_TIM_PWM_Start(gl_servo[index].Info1.htim,gl_servo[index].Info2.channel);
	HAL_Delay(200);
}
/**
 * @brief Set the Servo Angle By Index object
 * 
 * @param servoNum 从0开始，小于SERVO_NUM
 * @param angle 要设置的角度
 */
void setServoAngleByIndex(uint8_t servoNum, float angle){
  setServoAngle(&gl_servo[servoNum],angle);
}
void setServoAngleByIndex_int(uint8_t servoNum, uint16_t angle){
  setServoAngle(&gl_servo[servoNum],angle);
}
/**
 * @brief 设置舵机角度
 *
 * @param servoNum 舵机编号
 * @param angle 0-180度
 */
static void setServoAngle(struct Servo* servo, float angle) {
  if(angle > servo->maxAngle){
    angle = servo->maxAngle;
  }
  servo->angle = angle;
  if(servo->mode == DIGITAL_SERVO){
//      HAL_GPIO_WritePin(servo->Info1.GPIOx,servo->Info2.pin,GPIO_PIN_SET);
//			delay_us(calServoAngle(servo->angle,servo->maxAngle));
//			HAL_GPIO_WritePin(servo->Info1.GPIOx,servo->Info2.pin,GPIO_PIN_RESET);
//			delay_us(3000-calServoAngle(servo->angle,servo->maxAngle));
      // HAL_Delay(17);
      // HAL_GPIO_WritePin(servo->Info1.GPIOx,servo->Info2.pin,GPIO_PIN_SET);
			// delay_us(calServoAngle(servo->angle,servo->maxAngle));
			// HAL_GPIO_WritePin(servo->Info1.GPIOx,servo->Info2.pin,GPIO_PIN_RESET);
			// delay_us(3000-calServoAngle(servo->angle,servo->maxAngle));
      // HAL_Delay(17);
      // HAL_GPIO_WritePin(servo->Info1.GPIOx,servo->Info2.pin,GPIO_PIN_SET);
			// delay_us(calServoAngle(servo->angle,servo->maxAngle));
			// HAL_GPIO_WritePin(servo->Info1.GPIOx,servo->Info2.pin,GPIO_PIN_RESET);
			// delay_us(3000-calServoAngle(servo->angle,servo->maxAngle));
  }
  else{
    __HAL_TIM_SET_COMPARE(servo->Info1.htim, servo->Info2.channel, calServoAngle(servo->angle,servo->maxAngle));
//	__HAL_TIM_SET_COUNTER(servo->Info1.htim, 0);
  }
}

/**
 * @brief 输入角度，返回500-2500
 * 
 * @param angle 角度0-180
 * @param maxAngle 最大角度
 * @return uint16_t 返回比较值500-2500
 */
static uint16_t calServoAngle(float angle, uint16_t maxAngle) {
  return angle *2000 /maxAngle + 500;
}

/**
 * @brief 通过索引控制舵机角度
 * 
 * @param servoNum 舵机编号
 * @param angle 目标角度
 * @param tick_angle 单次转动的最大角度 
 * @return bool true 到达指定角度 false 未到达指定角度
 */
bool setServoSlowByIndex(uint8_t servoNum, float angle, float tick_angle)
{
  if (fabsf(gl_servo[servoNum].angle - angle) < tick_angle)
  {
    setServoAngle(&gl_servo[servoNum],angle);
    return true;
  }
  if (gl_servo[servoNum].angle < angle)
  {
    setServoAngle(&gl_servo[servoNum],gl_servo[servoNum].angle+tick_angle);
  }
  else if (gl_servo[servoNum].angle > angle)
  {
    setServoAngle(&gl_servo[servoNum],gl_servo[servoNum].angle-tick_angle);
  }
  return false;
}
