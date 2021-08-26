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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "stdbool.h"
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
SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//static const uint32_t I2C_DELAY = 1000;
//static const uint8_t I2C_ADDR = 0b1101000 << 1;
//static const uint8_t I2C_REG_ADDR = 0x41;

uint8_t GPSdata[256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void printVCP(char* buf);
FRESULT AppendToFile(char* path, size_t path_len, char* msg, size_t msg_len);
void BlinkLED(uint32_t blink_delay, uint8_t num_blinks);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	CDC_Transmit_FS((uint8_t*)GPSdata, 256);
	HAL_UART_Receive_IT(&huart3, (uint8_t*)GPSdata, 256);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	FRESULT fres;
//	char* msg = "Successfully written to sd card:D\n";
//	char log_path[] = "/LOG.TXT";
//	uint32_t currentTick = HAL_GetTick();
//	uint8_t* magData;
//	HAL_StatusTypeDef ret;
//	char msg[20] = "empty";

	uint32_t currentTick = HAL_GetTick();

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
  MX_USB_DEVICE_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart3, (uint8_t*)GPSdata, 256);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //NB: to transfer data to serial, do the following:
	  //uint8_t GPSdata[3] = {0x45, 0x34, 0x24};
	  //CDC_Transmit_FS((uint8_t*)GPSdata, 3);
	  //And to append to file use
	  //AppendToFile(log_path, strlen(log_path), (char*)GPSdata, 3);

//	  if((HAL_GetTick() - currentTick) >= 1000)
//		{
//			//Turn LED on while writing to file
//			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
//			fres = AppendToFile(log_path, strlen(log_path), msg, strlen(msg));
//			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
//
//			currentTick = HAL_GetTick();
//			printVCP((char*)"fileappend called\n\r");
//
//			//If error writing to card, blink 3 times
//			if(fres != FR_OK)
//			{
//				BlinkLED(200, 3);
//				printVCP((char*)"error no SD card present\n\r");
//			}
//		}

//	  HAL_Delay(1000);
//
//	  ret =  HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)I2C_ADDR, (uint8_t*)&I2C_REG_ADDR, 1, I2C_DELAY);
//	  if(ret != HAL_OK){
//		  printVCP((char*)"transmit error\n\r");
//	  }
//
//	  ret = HAL_I2C_Master_Receive(&hi2c2, (uint16_t)I2C_ADDR, (uint8_t*)magData, 2, I2C_DELAY);
//	  if(ret != HAL_OK){
//	  		  printVCP((char*)"read error\n\r");
//	  	  }
//	  uint32_t hold = ((magData[0] << 4) | (magData[1]));
//	  sprintf((char*)msg, "%d\n\r", (int)hold);
//	  printVCP((char*)msg);

	  if((HAL_GetTick() - currentTick) >= 1000)
	  {
		  printVCP((char*)"ok\n\r");
		  currentTick = HAL_GetTick();
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RED_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void printVCP(char* buf)
{
	char* eol = "\r\n";
	strcat(buf, eol);
	CDC_Transmit_FS((uint8_t*)buf, strlen(buf)); //use sizeof() as it returns size of array of anytype, strlen() expects char pointer in its argument
}

//Append string to file given a path
FRESULT AppendToFile(char* path, size_t path_len, char* msg, size_t msg_len)
{
	FATFS fs;
	FIL myfile;
	UINT testByte;
	FRESULT stat;

	//Bounds check on strings(i.e. check if strings are null terminated
	if((path[path_len] != 0) || (msg[msg_len] != 0))
	{
		return FR_INVALID_NAME;
	}

	//Re-initialize SD (allows SD card to be hot pluggable)
	if ( BSP_SD_Init() != MSD_OK)
	{
		return FR_NOT_READY;
	}

	//Re-initialize FATFS
	if(FATFS_UnLinkDriver(SDPath) != 0)
	{
		return FR_NOT_READY;
	}

	if(FATFS_LinkDriver(&SD_Driver, SDPath) != 0)
	{
		return FR_NOT_READY;
	}

	//Re-initialize and re-mount the FAT filesystem
	stat = f_mount(&fs, SDPath, 0);
	if(stat != FR_OK)
	{
		//unmount file system
		f_mount(0, SDPath, 0);
		//return error back to caller
		return stat;
	}

	//Open file for appending
	stat = f_open(&myfile, path, FA_WRITE | FA_OPEN_APPEND);
	if(stat != FR_OK)
	{
		f_mount(0, SDPath, 0);
		return stat;
	}

	//Write message to end of file
	stat = f_write(&myfile, msg, msg_len, &testByte);
	if(stat != FR_OK)
	{
		f_mount(0,SDPath, 0);
		return stat;
	}

	//Sync, close file, unmount
	stat = f_close(&myfile);
	f_mount(0, SDPath, 0);

	return stat;
}

//Blink onboard LED
void BlinkLED(uint32_t blink_delay, uint8_t num_blinks)
{
	for(int i = 0; i < num_blinks; i++)
	{
		HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
		HAL_Delay(blink_delay);
	}
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
