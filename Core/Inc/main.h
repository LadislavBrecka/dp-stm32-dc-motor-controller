/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

#define PWM_MIN -100
#define PWM_MAX 100

#define POT_MIN 0
#define POT_MAX 4196

#define IDLE   0
#define DONE   1

#define FORWARD_DIR   0
#define BACKWARD_DIR  1

#define F_CLK  100000UL
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_1_PWM_TRIMER_Pin GPIO_PIN_7
#define ADC_1_PWM_TRIMER_GPIO_Port GPIOA
#define TIMER_2_MOTOR_PWM_Pin GPIO_PIN_3
#define TIMER_2_MOTOR_PWM_GPIO_Port GPIOB
#define MOTOR_DIRECTION_1_Pin GPIO_PIN_4
#define MOTOR_DIRECTION_1_GPIO_Port GPIOB
#define MOTOR_DIRECTION_2_Pin GPIO_PIN_5
#define MOTOR_DIRECTION_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef struct USART_Data USART_Data;
struct USART_Data
{
	int32_t speed_set_point;
	int32_t speed;
	int32_t abs_pos;
};
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
