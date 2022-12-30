/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOC
#define PBTN2_Pin GPIO_PIN_3
#define PBTN2_GPIO_Port GPIOA
#define PBTN3_Pin GPIO_PIN_4
#define PBTN3_GPIO_Port GPIOA
#define PBTN1_Pin GPIO_PIN_5
#define PBTN1_GPIO_Port GPIOA
#define RED_Pin GPIO_PIN_9
#define RED_GPIO_Port GPIOE
#define GREEN_Pin GPIO_PIN_11
#define GREEN_GPIO_Port GPIOE
#define BLUE_Pin GPIO_PIN_13
#define BLUE_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_14
#define BUZZER_GPIO_Port GPIOE
#define RS_Pin GPIO_PIN_8
#define RS_GPIO_Port GPIOD
#define E_Pin GPIO_PIN_9
#define E_GPIO_Port GPIOD
#define D4_Pin GPIO_PIN_10
#define D4_GPIO_Port GPIOD
#define D5_Pin GPIO_PIN_11
#define D5_GPIO_Port GPIOD
#define D6_Pin GPIO_PIN_12
#define D6_GPIO_Port GPIOD
#define D7_Pin GPIO_PIN_13
#define D7_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
