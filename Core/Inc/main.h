/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define buzzer_Pin GPIO_PIN_2
#define buzzer_GPIO_Port GPIOE
#define control_Pin GPIO_PIN_3
#define control_GPIO_Port GPIOE
#define IN1_Pin GPIO_PIN_4
#define IN1_GPIO_Port GPIOE
#define IN2_Pin GPIO_PIN_5
#define IN2_GPIO_Port GPIOE
#define IN3_Pin GPIO_PIN_6
#define IN3_GPIO_Port GPIOE
#define IN4_Pin GPIO_PIN_13
#define IN4_GPIO_Port GPIOC
#define IN5_Pin GPIO_PIN_14
#define IN5_GPIO_Port GPIOC
#define IN6_Pin GPIO_PIN_15
#define IN6_GPIO_Port GPIOC
#define IN7_Pin GPIO_PIN_0
#define IN7_GPIO_Port GPIOF
#define IN8_Pin GPIO_PIN_1
#define IN8_GPIO_Port GPIOF
#define E01_CS_Pin GPIO_PIN_12
#define E01_CS_GPIO_Port GPIOB
#define E01_CE_Pin GPIO_PIN_8
#define E01_CE_GPIO_Port GPIOD
#define E01_IRQ_Pin GPIO_PIN_9
#define E01_IRQ_GPIO_Port GPIOD
#define E01_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define MOTOR1_IN1_Pin GPIO_PIN_11
#define MOTOR1_IN1_GPIO_Port GPIOA
#define MOTOR2_IN1_Pin GPIO_PIN_12
#define MOTOR2_IN1_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_1
#define LED_2_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOD
#define LED_1_Pin GPIO_PIN_4
#define LED_1_GPIO_Port GPIOD
#define BMP_280_CS_Pin GPIO_PIN_15
#define BMP_280_CS_GPIO_Port GPIOG
#define BMP_280_SCK_Pin GPIO_PIN_3
#define BMP_280_SCK_GPIO_Port GPIOB
#define BMP_280_MISO_Pin GPIO_PIN_4
#define BMP_280_MISO_GPIO_Port GPIOB
#define BMP_280_MOSI_Pin GPIO_PIN_5
#define BMP_280_MOSI_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
