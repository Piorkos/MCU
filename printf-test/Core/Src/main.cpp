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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "printf_to_uart.c"
#include "Imu.hpp"
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

/* USER CODE BEGIN PV */
//-====
//-====
/*flags*/
uint8_t MPU9250_DataRdyFlag = 0;
uint8_t initDataRdy = 0;
uint8_t magCalibrateFlag = 1;

/*mag calibration data*/
static float mx_centre;
static float my_centre;
static float mz_centre;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

float gravity[3];
float MagConst[3];
float gyroBias[3];
//====
//====
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//-====
//-====
void getGyroData();
void getAccelData();
void getCompassData();
void getRawCompassData();
void calibrateMag();
void getRawGyroData();
//====
//====
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//-====
Imu *myMPU = new Imu (&hi2c1);
//====
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
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

//  -====
//  -====Uruchomienie TIM16
  HAL_TIM_Base_Start_IT(&htim16);

  uint8_t deviceID;
  deviceID = myMPU->getDeviceID();
  if (deviceID == 0x71)
  {
	  myMPU->initialize();//initialize
  }
  else
  {
	  printf("ID wrong");
	  return 0;
  }

  printf("Device OK, reading data \r\n");
//  ====
//  ====

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  -====
//  -====
  if(magCalibrateFlag)
  {
	  calibrateMag();
	  printf("Mag Calibration done! \r\n");
	  HAL_Delay(4000);
	  printf("Put the device to rest!! \r\n");
  }

  HAL_Delay(4000);

  //enable interrupt
  myMPU->enableInterrupt();

  uint8_t counter = 0;

  float sMag[3] = {0.0f,0.0f,0.0f};
  float sAcc[3] = {0.0f,0.0f,0.0f};
  float sGyro[3] = {0.0f,0.0f,0.0f};
  myMPU->readIntStatus();
  while (1)
  {
//	  if (!initDataRdy)
//	  {
//		  if (MPU9250_DataRdyFlag)
//		  {
//			  getRawGyroData();
//			  getAccelData();
//			  getCompassData();
//			  if(counter<100)
//			  {
//				  for (int i=0;i<3;i++)
//				  {
//					  sMag[i] = sMag[i] + Mxyz[i];
//					  sAcc[i] = sAcc[i] + Axyz[i];
//					  sGyro[i] = sGyro[i] + Gxyz[i];
//				  }
//				  counter = counter+1;
//			  }
//			  if (counter==100)
//			  {
//				  for (int i=0;i<3;i++)
//				  {
//					  MagConst[i] = sMag[i]/100;
//					  gravity[i] = sAcc[i]/100;
//					  gyroBias[i] = sGyro[i]/100;
//				  }
//				  initDataRdy = 1;
//			  }
//		  }
//		  else
//		  {
//			  /*data not ready*/
////			  printf("Data not Ready \r\n");
//		  }
//	  }
//	  else
//	  {
//		  if (MPU9250_DataRdyFlag)
//		  {
//			  /*get data*/
//			  getGyroData();
//			  getAccelData();
//			  getCompassData();
//			  printf("%f %f %f %f %f %f %f %f %f \r\n",Gxyz[0],Gxyz[1],Gxyz[2],Axyz[0],Axyz[1],Axyz[2],Mxyz[0],Mxyz[1],Mxyz[2]);
//
//			  /*clear flag and interrupt status*/
//			  MPU9250_DataRdyFlag = 0;
//			  myMPU->readIntStatus();
//
//			  /*collect steady state gyro measurement by averaging intial 50 gyromeasurement.*/
//
//			  /*mag initial data collected, lets go!!*/
//		  }
//		  else
//		  {
//			  /*data not ready*/
//			  printf("Data not Ready \r\n");
//		  }
//	  }
//	  ====
//	  ====
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/* USER CODE BEGIN 4 */
//====
//====Callback wywołany gdy timer odliczy ustawiony czas
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	getGyroData();
	getAccelData();
	getCompassData();
	printf("Gyro: %f %f %f Accel: %f %f %f Compass: %f %f %f \r\n",Gxyz[0],Gxyz[1],Gxyz[2],Axyz[0],Axyz[1],Axyz[2],Mxyz[0],Mxyz[1],Mxyz[2]);

	if(htim->Instance == TIM16)
	{
		printf("---timer--- \n");
	}
}

void getGyroData()
{
  int16_t gx, gy, gz;
  myMPU->getRotation(&gx, &gy, &gz);
  Gxyz[0] = (float) gx * 500 / 32768;//131 LSB(??/s)
  Gxyz[1] = (float) gy * 500 / 32768;
  Gxyz[2] = (float) gz * 500 / 32768;
  Gxyz[0] = Gxyz[0] - gyroBias[0];
  Gxyz[1] = Gxyz[1] - gyroBias[1];
  Gxyz[2] = Gxyz[1] - gyroBias[2];
  //High Pass Filter -> remove all values that are less than 0.05dps.
  for (int i=0;i<3;i++){
    if(Gxyz[i]<0.05){
      Gxyz[i]=0;
    }
  }
}

void getRawGyroData()
{
	int16_t gx, gy, gz;
	myMPU->getRotation(&gx, &gy, &gz);
	Gxyz[0] = (float) gx * 500 / 32768;//131 LSB(??/s)
	Gxyz[1] = (float) gy * 500 / 32768;
	Gxyz[2] = (float) gz * 500 / 32768;
}

void getAccelData(){
	int16_t ax, ay, az;
	myMPU->getAcceleration(&ax,&ay,&az);
	Axyz[0] = (float) ax / 16384;//16384  LSB/g
	Axyz[1] = (float) ay / 16384;
	Axyz[2] = (float) az / 16384;
}

void getCompassData(){
	uint8_t dataReady = myMPU->getCompassDataReady();
	if (dataReady == 1){
		int16_t mx, my, mz;
		myMPU->getMagData(&mx,&my,&mz);
		//14 bit output.
		Mxyz[0] = (float) mx * 4912 / 8192;
		Mxyz[1] = (float) my * 4912 / 8192;
		Mxyz[2] = (float) mz * 4912 / 8192;
		Mxyz[0] = Mxyz[0] - mx_centre;
		Mxyz[1] = Mxyz[1] - my_centre;
		Mxyz[2] = Mxyz[2] - mz_centre;

		/*frame transformation -> coz mag is mounted on different axies with gyro and accel*/
		float temp = Mxyz[0];
		Mxyz[0] = Mxyz[1];
		Mxyz[1] = temp;
		Mxyz[2] = Mxyz[2]*(-1);
	}
}

void getRawCompassData(){
	uint8_t dataReady = myMPU->getCompassDataReady();
	if (dataReady == 1){
		int16_t mx, my, mz;
		myMPU->getMagData(&mx,&my,&mz);
		//14 bit output.
		Mxyz[0] = (float) mx * 4912 / 8192;
		Mxyz[1] = (float) my * 4912 / 8192;
		Mxyz[2] = (float) mz * 4912 / 8192;
	}else{
		printf("Mag data not ready, using original data");
	}
}

void calibrateMag(){
  uint16_t ii = 0, sample_count = 0;
  float mag_max[3] = {1,1,1};
  float mag_min[3] = {-1,-1,-1};

  printf("Mag Calibration: Wave device in a figure eight until done! \r\n");
  HAL_Delay(2000);

  sample_count = 100;
  for(ii = 0; ii < sample_count; ii++) {
    getRawCompassData();  // Read the mag data
    for (int jj = 0; jj < 3; jj++) {
      if(Mxyz[jj] > mag_max[jj]) mag_max[jj] = Mxyz[jj];
      if(Mxyz[jj] < mag_min[jj]) mag_min[jj] = Mxyz[jj];
    }
    HAL_Delay(200);
  }

  // Get hard iron correction
  mx_centre  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  my_centre  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mz_centre  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts


  // Get soft iron correction estimate
  /*
  mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts
  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;
  dest2[0] = avg_rad/((float)mag_scale[0]);
  dest2[1] = avg_rad/((float)mag_scale[1]);
  dest2[2] = avg_rad/((float)mag_scale[2]);
  */
}
//====
//====
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
