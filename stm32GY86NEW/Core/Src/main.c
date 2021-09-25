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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDRESS  0b1101000
#define HMC5883L_ADDRESS 0b0011110
#define MS5611_ADDRESS   0b1110111
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
float accel_x_g;
float accel_y_g;
float accel_z_g;
float mpu_temp_c;
float gyro_x_rad;
float gyro_y_rad;
float gyro_z_rad;
float magn_x_gs;
float magn_y_gs;
float magn_z_gs;
float pressure_float;
float baro_temp_float;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void mpu6050_read_sensors(void) {

	// read the sensor values
	uint8_t rx_buffer[20];
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDRESS,0x3B,1, &rx_buffer, 20, 1000);

	// extract the raw values
	int16_t  accel_x  = rx_buffer[0]  << 8 | rx_buffer[1];
	int16_t  accel_y  = rx_buffer[2]  << 8 | rx_buffer[3];
	int16_t  accel_z  = rx_buffer[4]  << 8 | rx_buffer[5];
	int16_t  mpu_temp = rx_buffer[6]  << 8 | rx_buffer[7];
	int16_t  gyro_x   = rx_buffer[8]  << 8 | rx_buffer[9];
	int16_t  gyro_y   = rx_buffer[10] << 8 | rx_buffer[11];
	int16_t  gyro_z   = rx_buffer[12] << 8 | rx_buffer[13];
	int16_t  magn_x   = rx_buffer[14] << 8 | rx_buffer[15];
	int16_t  magn_y   = rx_buffer[16] << 8 | rx_buffer[17];
	int16_t  magn_z   = rx_buffer[18] << 8 | rx_buffer[19];

	// convert accelerometer readings into G's
	accel_x_g = accel_x / 8192.0f;
	accel_y_g = accel_y / 8192.0f;
	accel_z_g = accel_z / 8192.0f;


	// convert gyro readings into Radians per second
	gyro_x_rad = gyro_x / 939.650784f;
	gyro_y_rad = gyro_y / 939.650784f;
	gyro_z_rad = gyro_z / 939.650784f;

	// convert magnetometer readings into Gauss's
	magn_x_gs = magn_x / 660.0f;
	magn_y_gs = magn_y / 660.0f;
	magn_z_gs = magn_z / 660.0f;

}

void ms5611_read_sensors(void) {


	uint8_t rx_buffer[3];


		// read the pressure
		HAL_I2C_Mem_Read (&hi2c1, MS5611_ADDRESS,0x00,1, &rx_buffer, 3, 1000);
		// extract the raw value
		uint32_t pressure  = rx_buffer[0] << 16 | rx_buffer[1] << 8 | rx_buffer[2];

		// convert the pressure reading
		pressure_float = (float) pressure;

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// configure the MPU6050 (gyro/accelerometer)
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, 0x6B, 1,0x00, 1, 100) ; // exit sleep
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, 0x19, 1,109, 1, 100) ;  // sample rate = 8kHz / 110 = 72.7Hz
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, 0x1B, 1,0x18, 1, 100) ; // gyro full scale = +/- 2000dps
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, 0x1C, 1,0x08, 1, 100) ; // accelerometer full scale = +/- 4g

	// configure the HMC5883L (magnetometer)
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, 0x6A, 1,0x00, 1, 100) ;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, 0x37, 1,0x02, 1, 100) ;
	HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS, 0x00, 1,0x18, 1, 100) ;
	HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS, 0x01, 1,0x60, 1, 100) ;
	HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS, 0x02, 1,0x00, 1, 100) ;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, 0x37, 1,0x00, 1, 100) ;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, 0x6A, 1,0x20, 1, 100) ;

	// configure the MPU6050 to automatically read the magnetometer
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, 0x25, 1,HMC5883L_ADDRESS | 0x80, 1, 000) ;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, 0x26, 1,0x03, 1, 100) ;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, 0x27, 1,6 | 0x80, 1, 100) ;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, 0x67, 1,1, 1, 100) ;

	// configure the MS5611 (barometer)
	HAL_I2C_Mem_Write(&hi2c1, MS5611_ADDRESS, 0x1E, 1,0x00, 1, 100) ;
	HAL_I2C_Mem_Write(&hi2c1, MS5611_ADDRESS, 0x48, 1,0x00, 1, 100) ;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  mpu6050_read_sensors();
	  HAL_Delay(100);
	  ms5611_read_sensors();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
