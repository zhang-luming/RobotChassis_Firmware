/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : main.c 文件的头文件
  *                   此文件包含应用程序的公共定义
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * 此软件采用在 LICENSE 文件中找到的条款进行许可
  * 如果没有随附 LICENSE 文件，则按原样提供
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* 全局配置参数已移至 USER/user_config.h */
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
#define MOTOR_C_IN1_Pin GPIO_PIN_0
#define MOTOR_C_IN1_GPIO_Port GPIOC
#define MOTOR_C_IN2_Pin GPIO_PIN_1
#define MOTOR_C_IN2_GPIO_Port GPIOC
#define MOTOR_B_IN1_Pin GPIO_PIN_2
#define MOTOR_B_IN1_GPIO_Port GPIOC
#define MOTOR_B_IN2_Pin GPIO_PIN_3
#define MOTOR_B_IN2_GPIO_Port GPIOC
#define MOTOR_A_IN2_Pin GPIO_PIN_4
#define MOTOR_A_IN2_GPIO_Port GPIOC
#define MOTOR_A_IN1_Pin GPIO_PIN_5
#define MOTOR_A_IN1_GPIO_Port GPIOC
#define KEY_GPIO_Pin GPIO_PIN_1
#define KEY_GPIO_GPIO_Port GPIOB
#define MOTOR_D_IN1_Pin GPIO_PIN_11
#define MOTOR_D_IN1_GPIO_Port GPIOA
#define MOTOR_D_IN2_Pin GPIO_PIN_12
#define MOTOR_D_IN2_GPIO_Port GPIOA
#define MPU_INT_Pin GPIO_PIN_12
#define MPU_INT_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
