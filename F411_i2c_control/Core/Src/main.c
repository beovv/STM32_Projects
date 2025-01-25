/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t NewLine[] = "\r\n Writing I2C data: \r\n";
uint8_t StartMSG1[] = "\r\n Starting I2C_1 Scanning: \r\n";
uint8_t StartMSG2[] = "\r\n Starting I2C_2 Scanning: \r\n";
char buf[8] = {0,};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t Buf20[20]   = {0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x1C};
	uint8_t Init10[20]  = {0x30,0x03,0x43,0x40,0x40,0x11,0x06,0x00,0x01,0xB1,0x41,0x22,0xA9,0x41,0x22,0xFF,0xFF,0x00,0x30,0x00};
	uint8_t Init11[20]  = {0x30,0x03,0x43,0x40,0x40,0x11,0x06,0x01,0x01,0xB1,0x41,0x22,0xA9,0x41,0x22,0xFF,0xFF,0x00,0x30,0x00};
	uint8_t Init12[20]  = {0x30,0x03,0x43,0x40,0x40,0x11,0x06,0x02,0x01,0xB1,0x41,0x22,0xA9,0x41,0x22,0xFF,0xFF,0x00,0x30,0x00};
	char strs[20][20] = {"POWER_CTRL","AMP_DAC_CTRL","DAC_CTRL","VOL_LEFT_CTRL","VOL_RIGHT_CTRL",
						"SAI_CTRL1","SAI_CTRL2","SLOT_LEFT_CTRL","SLOT_RIGHT_CTRL","LIM_LEFT_CTRL1",
						"LIM_LEFT_CTRL2","LIM_LEFT_CTRL3","LIM_RIGHT_CTRL1","LIM_RIGHT_CTRL2","LIM_RIGHT_CTRL3",
						"CLIP_LEFT_CTRL","CLIP_RIGHT_CTRL","FAULT_CTRL1","FAULT_CTRL2","SOFT_RESET"};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	uint8_t i = 0, ret;
	uint8_t Buf0[1];
	uint8_t Buf1[1];
	uint8_t Buf2[1];
	uint8_t dB10[1];
	uint8_t dB11[1];
	uint8_t dB12[1];
	char log[55] = {0,};
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(50);
  HAL_UART_Transmit(&huart1, StartMSG1, sizeof(StartMSG1), 10000);
  for(i=16; i<19; i++)
  {
     ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
      //if (ret != HAL_OK) // No ACK Received At That Address
      //{
      //    HAL_UART_Transmit(&huart1, Space, sizeof(Space), 10000);
      //}
      //else if(ret == HAL_OK)
       if (ret == HAL_OK)
     {
    	  sprintf(buf, "0x%X \r\n", i);
          HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), 10000);
      }
  };
  HAL_UART_Transmit(&huart1, StartMSG2, sizeof(StartMSG2), 10000);
  HAL_Delay(50);
  for(i=16; i<19; i++)
  {
     ret = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 3, 5);
      //if (ret != HAL_OK) // No ACK Received At That Address
      //{
      //    HAL_UART_Transmit(&huart1, Space, sizeof(Space), 10000);
      //}
      //else if(ret == HAL_OK)
       if (ret == HAL_OK)
     {
    	  sprintf(buf, "0x%X \r\n", i);
          HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), 10000);
      }
  };
  HAL_Delay(50);
  HAL_UART_Transmit(&huart1, NewLine, sizeof(NewLine), 10000);
  HAL_Delay(100);
  for(i=0; i<20; i++)
  {
	  dB10[0] = Init10[i]; dB11[0] = Init11[i]; dB12[0] = Init12[i];
	  HAL_I2C_Mem_Write(&hi2c1, (0x10 << 1), Buf20[i], I2C_MEMADD_SIZE_8BIT, dB10, 1, 100);
	  HAL_Delay(2);
	  HAL_I2C_Mem_Read(&hi2c1, (0x10 << 1), Buf20[i], I2C_MEMADD_SIZE_8BIT, Buf0, 1, 100);
	  HAL_I2C_Mem_Write(&hi2c1, (0x11 << 1), Buf20[i], I2C_MEMADD_SIZE_8BIT, dB11, 1, 100);
	  HAL_Delay(2);
	  HAL_I2C_Mem_Read(&hi2c1, (0x11 << 1), Buf20[i], I2C_MEMADD_SIZE_8BIT, Buf1, 1, 100);
	  HAL_I2C_Mem_Write(&hi2c1, (0x12 << 1), Buf20[i], I2C_MEMADD_SIZE_8BIT, dB12, 1, 100);
	  HAL_Delay(2);
	  HAL_I2C_Mem_Read(&hi2c1, (0x12 << 1), Buf20[i], I2C_MEMADD_SIZE_8BIT, Buf2, 1, 100);
	  sprintf(log, " 0x%X 0x%X 0x%X : %s I2C1 Adrs 0x%X\r\n", Buf0[0], Buf1[0], Buf2[0], strs[i], Buf20[i]);
	  HAL_UART_Transmit(&huart1, (uint8_t *)log, strlen(log), 100);
	  HAL_I2C_Mem_Write(&hi2c2, (0x10 << 1), Buf20[i], I2C_MEMADD_SIZE_8BIT, dB10, 1, 100);
	  HAL_Delay(2);
	  HAL_I2C_Mem_Read(&hi2c2, (0x10 << 1), Buf20[i], I2C_MEMADD_SIZE_8BIT, Buf0, 1, 100);
	  HAL_I2C_Mem_Write(&hi2c2, (0x11 << 1), Buf20[i], I2C_MEMADD_SIZE_8BIT, dB11, 1, 100);
	  HAL_Delay(2);
	  HAL_I2C_Mem_Read(&hi2c2, (0x11 << 1), Buf20[i], I2C_MEMADD_SIZE_8BIT, Buf1, 1, 100);
	  HAL_I2C_Mem_Write(&hi2c2, (0x12 << 1), Buf20[i], I2C_MEMADD_SIZE_8BIT, dB12, 1, 100);
	  HAL_Delay(2);
	  HAL_I2C_Mem_Read(&hi2c2, (0x12 << 1), Buf20[i], I2C_MEMADD_SIZE_8BIT, Buf2, 1, 100);
	  sprintf(log, " 0x%X 0x%X 0x%X : %s I2C2 Adrs 0x%X\r\n", Buf0[0], Buf1[0], Buf2[0], strs[i], Buf20[i]);
	  HAL_UART_Transmit(&huart1, (uint8_t *)log, strlen(log), 100);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
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
  hi2c2.Init.ClockSpeed = 100000;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
