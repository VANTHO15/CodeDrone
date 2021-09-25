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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_4
#define LED3_GPIO_Port GPIOE
#define ADC_SOLAR_Pin GPIO_PIN_0
#define ADC_SOLAR_GPIO_Port GPIOC
#define BUZZ_Pin GPIO_PIN_2
#define BUZZ_GPIO_Port GPIOA
#define SERVO_Pin GPIO_PIN_3
#define SERVO_GPIO_Port GPIOA
#define SPI_CSN_Pin GPIO_PIN_4
#define SPI_CSN_GPIO_Port GPIOA
#define NRF_IRQ_Pin GPIO_PIN_4
#define NRF_IRQ_GPIO_Port GPIOC
#define NRF_CE_Pin GPIO_PIN_5
#define NRF_CE_GPIO_Port GPIOC
#define ESC1_Pin GPIO_PIN_12
#define ESC1_GPIO_Port GPIOD
#define ESC2_Pin GPIO_PIN_13
#define ESC2_GPIO_Port GPIOD
#define ESC3_Pin GPIO_PIN_14
#define ESC3_GPIO_Port GPIOD
#define ESC4_Pin GPIO_PIN_15
#define ESC4_GPIO_Port GPIOD
#define I2C1_DRDY_Pin GPIO_PIN_0
#define I2C1_DRDY_GPIO_Port GPIOD
#define I2C1_INTA_Pin GPIO_PIN_1
#define I2C1_INTA_GPIO_Port GPIOD
#define RGB1_Pin GPIO_PIN_5
#define RGB1_GPIO_Port GPIOD
#define RGB2_Pin GPIO_PIN_6
#define RGB2_GPIO_Port GPIOD
#define RGB3_Pin GPIO_PIN_7
#define RGB3_GPIO_Port GPIOD
#define I2C1_SFYNC_Pin GPIO_PIN_8
#define I2C1_SFYNC_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
