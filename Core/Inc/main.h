/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define PWR_ON_Pin GPIO_PIN_13
#define PWR_ON_GPIO_Port GPIOC
#define RF_RX_Pin GPIO_PIN_3
#define RF_RX_GPIO_Port GPIOA
#define VCC_ADC_Pin GPIO_PIN_4
#define VCC_ADC_GPIO_Port GPIOA
#define MIC_Pin GPIO_PIN_7
#define MIC_GPIO_Port GPIOA
#define HIGH_LED_Pin GPIO_PIN_12
#define HIGH_LED_GPIO_Port GPIOB
#define PWR_LOW_LED_Pin GPIO_PIN_13
#define PWR_LOW_LED_GPIO_Port GPIOB
#define PWR_LED_Pin GPIO_PIN_14
#define PWR_LED_GPIO_Port GPIOB
#define LOW_LED_Pin GPIO_PIN_15
#define LOW_LED_GPIO_Port GPIOB
#define AUTO_LED_Pin GPIO_PIN_8
#define AUTO_LED_GPIO_Port GPIOA
#define HIGH_SW_Pin GPIO_PIN_6
#define HIGH_SW_GPIO_Port GPIOF
#define PWR_SW_Pin GPIO_PIN_7
#define PWR_SW_GPIO_Port GPIOF
#define PWR_SW_EXTI_IRQn EXTI4_15_IRQn
#define VSPK_BST_Pin GPIO_PIN_15
#define VSPK_BST_GPIO_Port GPIOA
#define AUTO_SW_Pin GPIO_PIN_3
#define AUTO_SW_GPIO_Port GPIOB
#define AUTO_SW_EXTI_IRQn EXTI2_3_IRQn
#define PWM2_Pin GPIO_PIN_4
#define PWM2_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_5
#define PWM1_GPIO_Port GPIOB
#define LOW_SW_Pin GPIO_PIN_6
#define LOW_SW_GPIO_Port GPIOB
#define LOW_SW_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
