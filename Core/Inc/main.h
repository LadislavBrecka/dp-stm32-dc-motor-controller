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

#define FORWARD_DIR   0
#define BACKWARD_DIR  1

#define F_CLK  100000UL

#define M_PI           3.14159265358979323846  /* pi */

enum AlgorithmStage {
	NON_IDENTIFIED = 0,
	IDENTIFIED = 1,
	READY_FOR_REGULATION = 2,
	MANUAL_MODE = 3
};

enum HalState {
	IDLE = 0,
	RUNNING = 1,
};

struct SimData
{
	int32_t u_speed;
	int32_t y_speed;
	int32_t w_pos;
	int32_t y_pos;
};
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
#define RESET_CLOSED_LOOP_BUTTON_Pin GPIO_PIN_10
#define RESET_CLOSED_LOOP_BUTTON_GPIO_Port GPIOB
#define RESET_CLOSED_LOOP_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define RESET_EXPERIMENT_BUTTON_Pin GPIO_PIN_8
#define RESET_EXPERIMENT_BUTTON_GPIO_Port GPIOA
#define RESET_EXPERIMENT_BUTTON_EXTI_IRQn EXTI9_5_IRQn
#define TIMER_2_MOTOR_PWM_Pin GPIO_PIN_3
#define TIMER_2_MOTOR_PWM_GPIO_Port GPIOB
#define MOTOR_DIRECTION_1_Pin GPIO_PIN_4
#define MOTOR_DIRECTION_1_GPIO_Port GPIOB
#define MOTOR_DIRECTION_2_Pin GPIO_PIN_5
#define MOTOR_DIRECTION_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
