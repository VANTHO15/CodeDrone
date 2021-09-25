/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HuLaNRF24L01.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint16_t a[5],Test=0,i=0,NUT1=0,NUT2=0,NUT3=0,NUT4=0,NUT5=0,NUT6=0,StateMachine=0,NangHa=0,KhoiDong=0;
volatile uint16_t ADC_SOLAR=0,X_LEFT=255,Y_LEFT=0,X_RIGHT=0,Y_RIGHT=0,Start=0;
volatile uint32_t Last=0, ViTri=0, FlightMode=1;

uint8_t TxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA};
uint8_t TxData[] = "123456789\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void Get_ADC_By_DMA();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
		if(GPIO_Pin== BTN1_Pin)
		{
			for(int i = 0; i < 50000; i++); //Delay
			while(!HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin));
			for(int i = 0; i < 50000; i++);
			EXTI->PR |= BTN1_Pin;
			NUT1=1;
			FlightMode=2;
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );

		}
		else
		if(GPIO_Pin== BTN2_Pin)
		{
			for(int i = 0; i < 50000; i++); //Delay
			while(!HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin));
			for(int i = 0; i < 50000; i++);
			EXTI->PR |= BTN2_Pin;
			NUT2=1;
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
		}
		else
		if(GPIO_Pin== BTN3_Pin)
		{
			for(int i = 0; i < 50000; i++); //Delay
			while(!HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin));
			for(int i = 0; i < 50000; i++);
			EXTI->PR |= BTN3_Pin;
			NUT3=1;

			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
		}
		else
 		if(GPIO_Pin== BTN4_Pin)
 		{
 			for(int i = 0; i < 50000; i++); //Delay
 			while(!HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin));
 			for(int i = 0; i < 50000; i++);
 			EXTI->PR |= BTN4_Pin;
 			NUT4=1;

 			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
 		}
 		else
 		if(GPIO_Pin== BTN5_Pin)
		{
			for(int i = 0; i < 50000; i++); //Delay
			while(!HAL_GPIO_ReadPin(BTN5_GPIO_Port, BTN5_Pin));
			for(int i = 0; i < 50000; i++);
			EXTI->PR |= BTN5_Pin;
			NUT5=1;
 			Start=1;
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
		}
		else
		if(GPIO_Pin== BTN6_Pin)
		{
			for(int i = 0; i < 50000; i++); //Delay
			while(!HAL_GPIO_ReadPin(BTN6_GPIO_Port, BTN6_Pin));
			for(int i = 0; i < 50000; i++);
			EXTI->PR |= BTN6_Pin;
			NUT6=!NUT6;
			FlightMode=3;
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
		}
		else
		if(GPIO_Pin== BTN_LEFT_Pin)
		{
			for(int i = 0; i < 50000; i++); //Delay
			while(!HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port, BTN_LEFT_Pin));
			for(int i = 0; i < 50000; i++);
			EXTI->PR |= BTN_LEFT_Pin;
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
		}
		else
		if(GPIO_Pin== BTN_RIGHT_Pin)
		{
			for(int i = 0; i < 50000; i++); //Delay
			while(!HAL_GPIO_ReadPin(BTN_RIGHT_GPIO_Port, BTN_RIGHT_Pin));
			for(int i = 0; i < 50000; i++);
			EXTI->PR |= BTN_RIGHT_Pin;
			Test=8;
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
		}

 }

void Get_ADC_By_DMA()
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)a, 5);

		if(a[2] < 1000) X_LEFT =1;
		else if(a[2] > 3000) X_LEFT = 0;
		else X_LEFT =255;

		Y_LEFT = 1000 + (1000/4028.0)*a[3];
		X_RIGHT =3000 - (1000 + (1000/4028.0)*a[0]);
		Y_RIGHT = 1000 + (1000/4028.0)*a[1];
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  NRF24_Init();
  NRF24_TxMode(TxAddress, 10);

  Last=HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Get_ADC_By_DMA();


	  if (NRF24_Transmit(TxData) == 1)
	  {
		  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
	  }
	  if(Start==1)
	  {
		  if(KhoiDong==0)
		  {
			  KhoiDong=1;
			  for(int i=1;i<=6;i++)
			  {
				  HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
				  HAL_Delay(500);
			  }
		  }

		 TxData[0]=X_LEFT;

		 TxData[1]=Y_LEFT/100;
		 TxData[2]=Y_LEFT%100;
		 TxData[3]=X_RIGHT/100;
		 TxData[4]=X_RIGHT%100;
		 TxData[5]=Y_RIGHT/100;
		 TxData[6]=Y_RIGHT%100;


		 TxData[7]=FlightMode;
		 TxData[8]=NUT2;
		 TxData[9]=NUT3;
		 TxData[10]=NUT4;
		 TxData[11]=NUT5;

		 HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	  }
	  else
	  {
		 TxData[0]=255;
		 TxData[1]=255;
		 TxData[2]=255;
		 TxData[3]=255;
		 TxData[4]=255;
		 TxData[5]=255;
		 TxData[6]=255;
		 TxData[7]=255;
		 TxData[8]=255;
		 TxData[9]=255;
		 TxData[10]=255;
		 TxData[11]=255;
	  }
	  if(FlightMode==2)
	  {
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
	  }
	  if(FlightMode==3)
	  {
		  if(HAL_GetTick()-Last>500)
		  {
			  Last=HAL_GetTick();
			  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		  }
	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CE_Pin|CSN_Pin|BUZZER_Pin|LED3_Pin
                          |LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CE_Pin CSN_Pin BUZZER_Pin LED3_Pin
                           LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin|BUZZER_Pin|LED3_Pin
                          |LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN6_Pin */
  GPIO_InitStruct.Pin = BTN6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN5_Pin BTN4_Pin BTN3_Pin BTN2_Pin
                           BTN1_Pin */
  GPIO_InitStruct.Pin = BTN5_Pin|BTN4_Pin|BTN3_Pin|BTN2_Pin
                          |BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_RIGHT_Pin BTN_LEFT_Pin */
  GPIO_InitStruct.Pin = BTN_RIGHT_Pin|BTN_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
