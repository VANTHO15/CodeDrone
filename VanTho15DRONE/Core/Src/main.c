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
#include "MPU6050.h"
#include "BMP180.h"
//#include "HMC5883L.h"
#include "stdio.h"
#include "math.h"

//#include <SimpleKalmanFilter.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define M_PI 3.1415926
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */


volatile uint16_t Test=0;
uint8_t RxAddress[] = {0x00,0xDD,0xCC,0xBB,0xAA};
uint8_t RxData[32], DoCao=0;

MPU6050_t MPU6050;

float Temperature = 0;
float Pressure = 0;
float Altitude = 0;
int16_t i=1,TrangThai=1,DenTa=50,Bam=0,CanBang=1, NangHa=-1,getpitch, getroll;  // NangHa= -1
float LenXuong=1, Base=0;
float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
//Vector HMC;

long MucDoCao[50]={1000, 1590, 1600 ,1625 ,1650,1675, 1700,1725, 1750,1775, 1800,1825, 1850,1875, 1900,1925,
		1950,1975, 2000,2025, 2050,2075, 2100,2125,2150,2175,2200,2225,2250,2275,
		2300,2325, 2350,2375, 2400,2425, 2450,2475, 2500,2525, 2550,2575, 2600 };

// PID
volatile float DoCaoMongMuon=0, DoCaoHienTai=0,GocHienTai=0, RollOfset,PitchOfset,LanDau=0,RollPID=0,PitchPID=0;

int pid_max_roll = 400;
int pid_max_pitch = 400;
float pid_p_gain_yaw = 4.0;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0.0;
int pid_max_yaw = 400;

// Yaw
float compassHorizontal_X,compassHorizontal_Y, actualCompassHeading, accelTotalVector;
float declination=-1.5,PitchHienTaiAccel,RollHienTaiAccel; // độ lệch của Việt Nam so với trục trái đất

volatile float RollMongMuon=0, RollHienTai=0, PitchMongMuon=0, PitchHienTai=0, YawMongMuon=0, YawHienTai=0;
volatile int16_t PwmTop=1000,PwmBack=1000,PwmLeft=1000,PwmRight=1000,StateMachine=0,PwmTop_t=0,PwmBack_t=0,PwmLeft_t=0,PwmRight_t=0;

// PID Roll
volatile float RollPreE, RollE,RollViPhan,RollTichPhan,RollPwmPID,RollKP=2.5,RollKI=0.02,RollKD= 1 ; // 1.8 0.2
// PID Pitch
volatile float PitchPreE,PitchE,PitchViPhan,PitchTichPhan,PitchPwmPID,PitchKP=2.5,PitchKI=0.02,PitchKD= 1 ;  // 0.5 0.7


void PrintPwmToESC(uint16_t pwmTop,uint16_t pwmBack,uint16_t pwmLefft, uint16_t pwmRight);
void IndependentWatchdog();
void ReceiveDataFromNRF24L01();
void ReadDataFromMPU6050();
//void ReadDataFromHMC5883L();
void ReadDataFromBMP180();
void ReadYawAndCalib();
float course_deviation(float course_b, float course_c);

uint8_t ChuongTrinhKhoiDongCamBien();
void ChuongTrinhKhoiDongDongCo();

int16_t GetPWMRollPID();
int16_t GetPWMPitchPID();

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PrintPwmToESC(uint16_t pwmTop,uint16_t pwmBack,uint16_t pwmLeft, uint16_t pwmRight)
{
	 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pwmTop);
	 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwmBack);
	 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwmLeft);
	 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,pwmRight);
}
void IndependentWatchdog()
{
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    IWDG->KR = 0xAAAA;
}
void ReceiveDataFromNRF24L01()
{
	if(isDataAvailable(2)==1)
	  {
		  NRF24_Receive(RxData);
		  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		  StateMachine=RxData[0];
		  if((StateMachine==3))
		  {
			  CanBang=0;
			  PwmTop=1590;
			  PwmBack=1590;
			  PwmLeft=1590;
			  PwmRight=1590;
			  PrintPwmToESC( PwmTop, PwmBack, PwmLeft,  PwmRight);
		  }
		  if(StateMachine==1)
		  {


			  if(RxData[1]==1) //  Trái Lên  / Nâng �?ộ Cao
			  {
				  LenXuong =LenXuong + 0.1;
				  if(LenXuong > 42)
				  {
					  LenXuong= 42;
				  }
				  i = (int16_t)LenXuong;
			  }
			  else
			  if(RxData[1]==0) // Trái Xuống / Hạ �?ộ Cao
			  {
				  LenXuong =LenXuong - 0.1;
				  if(LenXuong < 0)
				  {
					  LenXuong = 0;
				  }
				  i = (int16_t)LenXuong;
			  }
			  else
			  if(RxData[2]==1) // Phải Phải / �?ông Cơ �?i Phải
			  {
				  TrangThai=7;
			  }
			  else
			  if(RxData[2]==0) // Phải Trái / �?ộng Cơ �?i Trái
			  {
				  TrangThai=6;
			  }
			  else
			  if(RxData[3]==1) // Phải Lên  / �?ộng Cơ Tiến
			  {
				  TrangThai=2;
			  }
			  else
			  if(RxData[3]==0) // Phải Xuống / �?ộng Cơ Lùi
			  {
				  TrangThai=3;
			  }
			  else
			  if(RxData[4]==1) // Trái Phải / �?ộng Cơ Quay Phải
			  {
				  TrangThai=5;
			  }
			  else
			  if(RxData[4]==0) // Trái Trái / �?ộng Cơ Quay Trái
			  {
				  TrangThai=4;
			  }
			  else
			  {
				  TrangThai=1;   // �?ứng yên
			  }

		  }

		  NangHa=RxData[5];

	  }
}
void ReadDataFromMPU6050()
{
	MPU6050_Read_All(&hi2c1, &MPU6050);

	RollHienTai = MPU6050.KalmanAngleX ;
	PitchHienTai = MPU6050.KalmanAngleY + 1.2;

//	RollHienTai = roundf(MPU6050.KalmanAngleX * 100)/ 100;
//	PitchHienTai = roundf((MPU6050.KalmanAngleY+0.6) * 100)/ 100;

}
//void ReadDataFromHMC5883L()
//{
//	 HMC = HMC5883L_readNormalize();
//	 GocHienTai=HMC.compas;
//}
void ReadDataFromBMP180()
{
//	 Temperature = BMP180_GetTemp();
//	 Pressure = BMP180_GetPress (0);
	 Altitude = BMP180_GetAlt(0);
	 DoCaoHienTai=Altitude;
}
//void ReadYawAndCalib()
//{
//	ReadDataFromHMC5883L();
//	/*
//	 * Giá trị compass sẽ thay đổi khi góc roll và pitch thay đổi.
//	 * Do đó các giá trị compass x&y cần được hiệu chỉnh lại theo
//	 * các góc roll và pitch để đạt được vị trí phương ngang ảo.
//	 * The 0.0174533 value is phi/180 as the functions are in radians in stead of degrees.
//	 */
//	compassHorizontal_X = (float)HMC.XAxis * cos(PitchHienTai * -0.0174533) + (float)HMC.YAxis * sin(RollHienTai * 0.0174533) * sin(PitchHienTai * -0.0174533) - (float)HMC.ZAxis * cos(RollHienTai * 0.0174533) * sin(PitchHienTai * -0.0174533);
//	compassHorizontal_Y = (float)HMC.YAxis * cos(RollHienTai * 0.0174533) + (float)HMC.ZAxis * sin(RollHienTai * 0.0174533);
//
//	if (compassHorizontal_Y < 0)
//	{
//		actualCompassHeading = 180 + (180 + ((atan2(compassHorizontal_Y, compassHorizontal_X)) * (180 / 3.14)));
//	}
//	else
//	{
//		actualCompassHeading = (atan2(compassHorizontal_Y, compassHorizontal_X)) * (180 / 3.14);
//	}
//	actualCompassHeading += declination;                                 //Add the declination to the magnetic compass heading to get the geographic north.
//	if (actualCompassHeading < 0) actualCompassHeading += 360;         //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
//	else if (actualCompassHeading >= 360) actualCompassHeading -= 360;
//
//	YawHienTai +=MPU6050.Gz/250.0;
//	if (YawHienTai < 0) YawHienTai += 360;
//    else if (YawHienTai >= 360) YawHienTai -= 360;
//
//	YawHienTai -= course_deviation(YawHienTai, actualCompassHeading) / 1200.0;       //Calculate the difference between the gyro and compass heading and make a small correction.
//	if (YawHienTai < 0) YawHienTai += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
//	 else if (YawHienTai >= 360) YawHienTai -= 360;
//}
////The following subrouting calculates the smallest difference between two heading values.
//float course_deviation(float course_b, float course_c) {
//  course_a = course_b - course_c;
//  if (course_a < -180 || course_a > 180) {
//    if (course_c > 180)base_course_mirrored = course_c - 180;
//    else base_course_mirrored = course_c + 180;
//    if (course_b > 180)actual_course_mirrored = course_b - 180;
//    else actual_course_mirrored = course_b + 180;
//    course_a = actual_course_mirrored - base_course_mirrored;
//  }
//  return course_a;
//}
void ChuongTrinhKhoiDongDongCo()
{
	//  Bắt �?ầu PWM
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}
uint8_t ChuongTrinhKhoiDongCamBien()
{
	// Khởi động động cơ
	//ChuongTrinhKhoiDongDongCo();
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	// NRF24L01
	 NRF24_Init();
	 NRF24_RxMode(RxAddress, 10);
	// Setup MPU6050
	 while (MPU6050_Init(&hi2c1) == 1);
	 // setup Cảm Biến �?p Xuất
	 BMP180_Start();
	// Setup Cảm Biến La Bàn
//	 HMC5883L_setRange(HMC5883L_RANGE_1_3GA);
//	 HMC5883L_setMeasurementMode(HMC5883L_CONTINOUS);
//	 HMC5883L_setDataRate(HMC5883L_DATARATE_15HZ);
//	 HMC5883L_setSamples(HMC5883L_SAMPLES_1);
//	 HMC5883L_setOffset(0, 0);
	// Sâu khi Setup Xong Thì Led 2 Sẽ Sáng
	 HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	 for(int i=1;i<=5;i++)
	 {
		 HAL_GPIO_TogglePin(COI_GPIO_Port, COI_Pin);
		 HAL_Delay(1000);
	 }

	 ReadDataFromMPU6050();

	 return 1;
}

int16_t GetPWMRollPID()
{
	//RollHienTai = MPU6050.KalmanAngleX;
	RollPreE = RollE;
	RollE = RollMongMuon - RollHienTai ;
	RollViPhan = (RollE-RollPreE)/0.01;
	RollTichPhan += RollE * 0.01;
	if(RollTichPhan > pid_max_roll) RollTichPhan = pid_max_roll;
	else if(RollTichPhan < pid_max_roll * -1) RollTichPhan = pid_max_roll * -1;

	RollPwmPID = RollKP*RollE + RollTichPhan*RollKI + RollKD*RollViPhan;

	if(RollPwmPID > pid_max_roll) RollPwmPID = pid_max_roll;
	else if(RollPwmPID < (pid_max_roll * -1)) RollPwmPID = pid_max_roll * -1;

	return RollPwmPID;
}
int16_t GetPWMPitchPID()
{
	//PitchHienTai = MPU6050.KalmanAngleY;
	PitchPreE = PitchE;
	PitchE =  PitchMongMuon - PitchHienTai ;
	PitchViPhan = (PitchE-PitchPreE)/0.01;
	PitchTichPhan += PitchE * 0.01;
	if(PitchTichPhan > pid_max_pitch) PitchTichPhan = pid_max_pitch;
    else if(PitchTichPhan < (pid_max_pitch * -1)) PitchTichPhan = pid_max_pitch * -1;

	PitchPwmPID = PitchKP*PitchE  + PitchTichPhan*PitchKI + PitchKD*PitchViPhan;


	return PitchPwmPID;
}
// timer 0.01s
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{

		 if(NangHa==1)
		  {
			 if(CanBang==1)
			 {
				  Base = MucDoCao[i];
				   getpitch = GetPWMPitchPID();
				   getroll = GetPWMRollPID();

				  PwmTop = Base -  getpitch - getroll;
				  PwmBack = Base  + getpitch + getroll;
				  PwmLeft = Base - getpitch + getroll;
				  PwmRight = Base + getpitch - getroll ;

		     }

		  }

	}
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //SimpleKalmanFilter(1, 1, 0.001);
  while(ChuongTrinhKhoiDongCamBien()!=1)
  {
	  // Reset lại Drone
	  HAL_GPIO_WritePin(COI_GPIO_Port, COI_Pin, 1);
  }
  // bắt đầu ngắt timer 2
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_GPIO_WritePin(COI_GPIO_Port, COI_Pin, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // xử lý Independent Watchdog
	  IndependentWatchdog();
	  ReceiveDataFromNRF24L01();

	  ReadDataFromMPU6050();
	  //ReadDataFromHMC5883L();
	  ReadDataFromBMP180();
	  switch(TrangThai)
	  {
		  case 1:   // �?ứng Im
		  {
				  if(Bam== 1)
				  {

					  PwmTop +=DenTa;
					  PwmBack -=DenTa;
					  PwmLeft +=DenTa;
					  PwmRight -=DenTa;
				  }
				  else
				  if(Bam== 2)
				  {
					  PwmTop -=DenTa;
					  PwmBack +=DenTa;
					  PwmLeft -=DenTa;
					  PwmRight +=DenTa;

				  }
				  else
				  if(Bam== 3)
				  {
					  PwmTop -=DenTa;
					  PwmBack +=DenTa;
					  PwmLeft +=DenTa;
					  PwmRight -=DenTa;

				  }
				  else
				  if(Bam== 4)
				  {
					  PwmTop +=DenTa;
					  PwmBack -=DenTa;
					  PwmLeft -=DenTa;
					  PwmRight +=DenTa;

				  }
				  else
				  if(Bam== 5)
				  {
					  PwmTop -=DenTa;
					  PwmBack -=DenTa;
					  PwmLeft +=DenTa;
					  PwmRight +=DenTa;

				  }
				  else
				  if(Bam== 6)
				  {
					  PwmTop +=DenTa;
					  PwmBack +=DenTa;
					  PwmLeft -=DenTa;
					  PwmRight -=DenTa;
				  }

			  CanBang=1;
			  Bam=0;
			  break;
		  }
		  case 2:  // Lên
		  {
			  CanBang=0;
			  if(Bam==0)
			  {
				  Bam=1;
				  PwmTop -=DenTa;
				  PwmBack +=DenTa;
				  PwmLeft -=DenTa;
				  PwmRight +=DenTa;

			  }
			  break;
		  }
		  case 3: // Xuống
		  {
			  CanBang=0;
			  if(Bam==0)
			  {
				  Bam=2;
				  PwmTop +=DenTa;
				  PwmBack -=DenTa;
				  PwmLeft +=DenTa;
				  PwmRight -=DenTa;
			  }
			  break;
		  }
		  case 4: // Trái
		  {
			  CanBang=0;
			  if(Bam==0)
			  {
				  Bam=3;
				  PwmTop +=DenTa;
				  PwmBack -=DenTa;
				  PwmLeft -=DenTa;
				  PwmRight +=DenTa;
			  }
			  break;
		  }
		  case 5: // Phải
		  {
			  CanBang=0;
			  if(Bam==0)
			  {
				  Bam=4;
				  PwmTop -=DenTa;
				  PwmBack +=DenTa;
				  PwmLeft +=DenTa;
				  PwmRight -=DenTa;
			  }
			  break;
		  }
		  case 6: // Quay Trái
		  {
			  CanBang=0;
			  if(Bam==0)
			  {
				  Bam=5;
				  PwmTop +=DenTa;
				  PwmBack +=DenTa;
				  PwmLeft -=DenTa;
				  PwmRight -=DenTa;
			  }
			  break;
		  }
		  case 7: // Quay phải
		  {
			  CanBang=0;
			  if(Bam==0)
			  {
				  Bam=6;
				  PwmTop -=DenTa;
				  PwmBack -=DenTa;
				  PwmLeft +=DenTa;
				  PwmRight +=DenTa;
			  }
			  break;
		  }
	  }
	  if(NangHa == 1)
	  {

		  if(PwmTop >= 2500 ) PwmTop = 2500;
		  if(PwmBack >= 2500 ) PwmBack = 2500;
		  if(PwmLeft >= 2500 ) PwmLeft = 2500;
		  if(PwmRight >= 2500 ) PwmRight = 2500;

		  if(PwmTop <= 1000 ) PwmTop = 1000;
		  if(PwmBack <= 1000 ) PwmBack = 1000;
		  if(PwmLeft <= 1000 ) PwmLeft = 1000;
		  if(PwmRight <= 1000 ) PwmRight =1000;
	  }
	  else
		  if(NangHa == 2)
	  {
		  PwmTop = 1000;
		  PwmBack = 1000;
		  PwmLeft = 1000;
		  PwmRight = 1000;
	  }

	// ReadYawAndCalib();
	 PrintPwmToESC( PwmTop, PwmBack, PwmLeft,  PwmRight);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 1000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 159;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, COI_Pin|RGB3_Pin|RGB2_Pin|RGB1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZ_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUZZ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_CSN_Pin */
  GPIO_InitStruct.Pin = NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_CE_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DRDY_Pin INTA_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|INTA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : COI_Pin RGB3_Pin RGB2_Pin RGB1_Pin */
  GPIO_InitStruct.Pin = COI_Pin|RGB3_Pin|RGB2_Pin|RGB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
