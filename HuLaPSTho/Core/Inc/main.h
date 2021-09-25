/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define X_LEFT_Pin GPIO_PIN_0
#define X_LEFT_GPIO_Port GPIOA
#define Y_LEFT_Pin GPIO_PIN_1
#define Y_LEFT_GPIO_Port GPIOA
#define X_RIGHT_Pin GPIO_PIN_2
#define X_RIGHT_GPIO_Port GPIOA
#define Y_RIGHT_Pin GPIO_PIN_3
#define Y_RIGHT_GPIO_Port GPIOA
#define ADC_SOLAR_Pin GPIO_PIN_4
#define ADC_SOLAR_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_0
#define CE_GPIO_Port GPIOB
#define IRQ_Pin GPIO_PIN_1
#define IRQ_GPIO_Port GPIOB
#define CSN_Pin GPIO_PIN_2
#define CSN_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOB
#define BTN6_Pin GPIO_PIN_15
#define BTN6_GPIO_Port GPIOB
#define BTN6_EXTI_IRQn EXTI15_10_IRQn
#define BTN5_Pin GPIO_PIN_8
#define BTN5_GPIO_Port GPIOA
#define BTN5_EXTI_IRQn EXTI9_5_IRQn
#define BTN4_Pin GPIO_PIN_9
#define BTN4_GPIO_Port GPIOA
#define BTN4_EXTI_IRQn EXTI9_5_IRQn
#define BTN3_Pin GPIO_PIN_10
#define BTN3_GPIO_Port GPIOA
#define BTN3_EXTI_IRQn EXTI15_10_IRQn
#define BTN2_Pin GPIO_PIN_11
#define BTN2_GPIO_Port GPIOA
#define BTN2_EXTI_IRQn EXTI15_10_IRQn
#define BTN1_Pin GPIO_PIN_12
#define BTN1_GPIO_Port GPIOA
#define BTN1_EXTI_IRQn EXTI15_10_IRQn
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOB
#define BTN_RIGHT_Pin GPIO_PIN_4
#define BTN_RIGHT_GPIO_Port GPIOB
#define BTN_LEFT_Pin GPIO_PIN_5
#define BTN_LEFT_GPIO_Port GPIOB
#define BTN_LEFT_EXTI_IRQn EXTI9_5_IRQn
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
