/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CO/CO_app_STM32.h"
#include "CO/OD.h"
#include "CO/301/CO_Emergency.h"
#include <stdio.h>
#include <stdbool.h>
#include "bme280.h"
#include "flashEeprom.h"
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
configFlash_t eeprom = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
BMP280_t bmp280;
float Temp, Pressure;

void I2C_Scanner(void)
{
    HAL_StatusTypeDef result;
    uint8_t i;

    printf("Skanowanie magistrali I2C...\r\n");

    for (i = 1; i < 128; i++)
    {
        // Shift adres o 1 w lewo, ponieważ HAL oczekuje 8-bitowego adresu (7-bitowy << 1)
        result = HAL_I2C_IsDeviceReady(&hi2c1, (i << 1), 1, 10);
        if (result == HAL_OK)
        {
            printf("Znaleziono urządzenie na adresie 0x%02X\r\n", i);
        }
        HAL_Delay(2);
    }

    printf("Skanowanie zakończone.\r\n");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t activeError = false;
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
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  CANopenNodeSTM32 canOpenNodeSTM32;

  canOpenNodeSTM32.CANHandle = &hcan;
  canOpenNodeSTM32.HWInitFunction = MX_CAN_Init;
  canOpenNodeSTM32.timerHandle = &htim2;
  canOpenNodeSTM32.desiredNodeID = 2;
  canOpenNodeSTM32.baudrate = 250;

  while(canopen_app_init(&canOpenNodeSTM32) != HAL_OK);

  if(BMP280_Init(&bmp280, &hi2c1, 0x76) != HAL_OK)
	 Error_Handler();

  uint32_t sendDataTime = HAL_GetTick();
  uint32_t connectionCheckTime = HAL_GetTick();

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, !canOpenNodeSTM32.outStatusLEDGreen);
	  canopen_app_process();

	  if ((HAL_GetTick() - sendDataTime) >= OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.eventTimer && activeError == false)
	  {
		if(BMP280_ReadPressureAndTemperature(&bmp280, &Pressure, &Temp) == HAL_OK)
		{
			OD_PERSIST_COMM.x6000_temp = (int32_t)(Temp * 100);
			OD_PERSIST_COMM.x6001_pressure = (uint32_t)(Pressure * 100);

			//OD_set_u32(OD_find(OD, 0x6000), 0x00, Temp, false);
			//OD_set_u32(OD_find(OD, 0x6001), 0x00, Pressure, false);
		}

		sendDataTime = HAL_GetTick();
	  }

	  if((HAL_GetTick() - connectionCheckTime) >= 1000)
	  {
		  if(BMP280_CheckConnection(&bmp280) != HAL_OK)
		  {
			  activeError = true;
			  OD_PERSIST_COMM.x6000_temp = INT32_MAX;
			  OD_PERSIST_COMM.x6001_pressure = UINT32_MAX;
			  CO_errorReport(canOpenNodeSTM32.canOpenStack->em, CO_ERR_REG_COMMUNICATION, CO_EMC_HARDWARE, 0);
		  }
		  else
		  {
			  activeError = false;
		  }

		  connectionCheckTime = HAL_GetTick();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Handle CANOpen app interrupts
  if (htim == canopenNodeSTM32->timerHandle)
  {
      canopen_app_interrupt();
  }
}

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

  return ch;
}
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
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
