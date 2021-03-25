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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "print_to_uart.c"
#include "Imu.hpp"
#include "w25qxx/w25qxx.h"
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
uint32_t btn_delay{200000};
uint32_t counter_1{1000000};
uint32_t counter_2{1000000};
uint32_t counter_3{1000000};

/*mag calibration data*/
float Axyz[3];
float Gxyz[3];

float gyroBias[3];

//external flash storage
uint32_t samples_saved{0};
uint32_t selected_sample{1};

bool write_to_flash{false};
bool read_from_flash{false};

//====
//====
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//-====
//-====
void accel_serialise(float *a_xyz, uint8_t *accel_int_8);
void accel_parse(uint8_t *accel, float *accel_parsed);
void gyro_serialise(float *g_xyz, uint8_t *g_xyz_uint_8);
void gyro_parse(uint8_t *g_xyz_uint_8, float *g_xyz);

void getGyroData();
void getAccelData();
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  //  -====
  //  -====
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

  HAL_Delay(4000);

  //enable interrupt
  printf("MPU enable \n");
  myMPU->enableInterrupt();
  myMPU->readIntStatus();

  printf("Init FLASH \n");
  W25qxx_Init();
  W25qxx_EraseSector(1);

  printf("--WHILE-- \n");
  while (1)
  {
	  if(counter_1 < (btn_delay + 1))
	  {
		  ++counter_1;
	  }
	  if(counter_2 < (btn_delay + 1))
	  {
		  ++counter_2;
	  }
	  if(counter_3 < (btn_delay + 1))
	  {
		  ++counter_3;
	  }

	  if(write_to_flash)
	  {
		  write_to_flash = false;

		  uint8_t a_xyz_uint_8[6];
		  uint8_t g_xyz_uint_8[6];
		  accel_serialise(Axyz, a_xyz_uint_8);
		  gyro_serialise(Gxyz, g_xyz_uint_8);

		  uint32_t offset{samples_saved * 12};

		  W25qxx_WriteSector(a_xyz_uint_8, 1, offset, 6);
		  W25qxx_WriteSector(g_xyz_uint_8, 1, (offset + 6), 6);

		  ++samples_saved;
	  }

	  if(read_from_flash)
	  {
		  if(samples_saved > 0)
		  {
			  read_from_flash = false;

			  uint8_t read_buffer_1[6];
			  uint8_t read_buffer_2[6];

			  uint32_t offset{selected_sample};
			  offset = (offset - 1) * 12;

			  W25qxx_ReadSector(read_buffer_1, 1, offset, 6);
			  W25qxx_ReadSector(read_buffer_2, 1, (offset + 6), 6);
			  //		  printf("r[0]: %d, r[1]: %d \n", read_buffer[0], read_buffer[1]);
			  //		  printf("r[0]: %d, r[1]: %d, r[2]: %d, r[3]: %d, r[4]: %d, r[5]: %d \n", read_buffer[0], read_buffer[1], read_buffer[2], read_buffer[3], read_buffer[4], read_buffer[5]);

			  float a_xyz[3];
			  float g_xyz[3];
			  accel_parse(read_buffer_1, a_xyz);
			  gyro_parse(read_buffer_2, g_xyz);
			  printf("DATA FROM EXTERNAL FLASH MEMORY: \n");
			  printf("\t GYRO : X = %f, Y = %f, Z = %f ACCEL: X = %f, Y = %f, Z = %f \n", g_xyz[0], g_xyz[1], g_xyz[2], a_xyz[0], a_xyz[1], a_xyz[2]);
		  }
		  else
		  {
			  printf("No data saved yet. \n");
		  }
	  }
	  //  ====
	  //  ====
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
//Converts captured FLOAT accelerometer data into 2 * UINT8 variables which can be stored in external Flash memory
void accel_serialise(float *a_xyz, uint8_t *a_xyz_uint_8)
{
	uint16_t accel_int_16;

	for (int i = 0; i < 3; ++i)
	{
		accel_int_16 = 0;

		a_xyz[i] = a_xyz[i] + 16;
		a_xyz[i] = a_xyz[i] * 1000;

		accel_int_16 = static_cast<uint16_t>(a_xyz[i]);
		a_xyz_uint_8[2 * i] = accel_int_16 & 0xff;
		a_xyz_uint_8[(2 * i) + 1] = (accel_int_16 >> 8);
	}
}

//Converts stored in Flash memory data into accelerometer data format in FLOAT
void accel_parse(uint8_t *a_xyz_uint_8, float *a_xyz)
{
	uint16_t accel_int_16;

	for (int i = 0; i < 3; ++i)
	{
		accel_int_16 = 0;

		accel_int_16 = (a_xyz_uint_8[(2 * i) + 1] << 8);
		accel_int_16 = accel_int_16 | a_xyz_uint_8[2*i];

		a_xyz[i] = static_cast<float>(accel_int_16);
		a_xyz[i] = a_xyz[i] / 1000;
		a_xyz[i] = a_xyz[i] - 16;
	}
}

void gyro_serialise(float *g_xyz, uint8_t *g_xyz_uint_8)
{
	uint16_t gyro_int_16;

	for (int i = 0; i < 3; ++i)
	{
		gyro_int_16 = 0;

		g_xyz[i] = g_xyz[i] + 1000;
		g_xyz[i] = g_xyz[i] * 10;

		gyro_int_16 = static_cast<uint16_t>(g_xyz[i]);
		g_xyz_uint_8[2 * i] = gyro_int_16 & 0xff;
		g_xyz_uint_8[(2 * i) + 1] = (gyro_int_16 >> 8);
	}
}

//Converts stored in Flash memory data into accelerometer data format in FLOAT
void gyro_parse(uint8_t *g_xyz_uint_8, float *g_xyz)
{
	uint16_t gyro_int_16;

	for (int i = 0; i < 3; ++i)
	{
		gyro_int_16 = 0;

		gyro_int_16 = (g_xyz_uint_8[(2 * i) + 1] << 8);
		gyro_int_16 = gyro_int_16 | g_xyz_uint_8[2*i];

		g_xyz[i] = static_cast<float>(gyro_int_16);
		g_xyz[i] = g_xyz[i] / 10;
		g_xyz[i] = g_xyz[i] - 1000;
	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BTN_1_Pin)
	{
		if(counter_1 > btn_delay)
		{
			counter_1 = 0;
			printf("BTN 1 \n");

			getGyroData();
			getAccelData();

			write_to_flash = true;

			printf("GYRO: %f %f %f \t ACCEL: %f %f %f \n",Gxyz[0],Gxyz[1],Gxyz[2],Axyz[0],Axyz[1],Axyz[2]);
		}
	}
	if(GPIO_Pin == BTN_2_Pin)
	{
		if(counter_2 > btn_delay)
		{
			counter_2 = 0;
			printf("BTN 2 \n");

			if(selected_sample < samples_saved)
			{
				++selected_sample;
			}
			else
			{
				selected_sample = 1;
			}

			printf("selected sample: %d / %d \n", static_cast<int>(selected_sample), static_cast<int>(samples_saved));
		}
	}
	if(GPIO_Pin == BTN_3_Pin)
	{
		if(counter_3 > btn_delay)
		{
			counter_3 = 0;
			printf("BTN 3 \n");

			read_from_flash = true;
		}
	}
}
void getGyroData()
{
  int16_t gx, gy, gz;
  myMPU->getRotation(&gx, &gy, &gz);
  Gxyz[0] = (float) gx * 1000 / 32768;//65.6 LSB(??/s)
  Gxyz[1] = (float) gy * 1000 / 32768;
  Gxyz[2] = (float) gz * 1000 / 32768;
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
	Gxyz[0] = (float) gx * 1000 / 32768;//65.5 LSB(??/s)
	Gxyz[1] = (float) gy * 1000 / 32768;
	Gxyz[2] = (float) gz * 1000 / 32768;
}

void getAccelData(){
	int16_t ax, ay, az;
	myMPU->getAcceleration(&ax,&ay,&az);
	Axyz[0] = (float) ax / 2048;//2048  LSB/g
	Axyz[1] = (float) ay / 2048;
	Axyz[2] = (float) az / 2048;
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
