/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include "usbd_cdc_if.h"
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

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

/* USER CODE BEGIN PV */
static const uint32_t I2C_DELAY = 1000;         // Time (ms) to wait for I2C
static const uint8_t PCT_I2C_ADDR = 0x37 << 1;  // Use 8-bit address
static const uint8_t PCT_REG_TEMP = 0x00;       // Temperature register
static const uint16_t PCT_ERROR = 0xFFFF;        // I2C/PCT error code
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint16_t ReadPCTTemperature(uint8_t i2c_addr);
FRESULT AppendToFile(char* path, size_t path_len, char* msg, size_t msg_len);
void BlinkLED(uint32_t blink_delay, uint8_t num_blinks);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  FRESULT fres;
  uint16_t raw_temp;
  float temp_c;
  char log_path[] = "/TEMPLOG.TXT";
  char buf[20];
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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // Attempt to read temperature from sensor
    raw_temp = ReadPCTTemperature(PCT_I2C_ADDR);
    if ( raw_temp == PCT_ERROR ) {
      BlinkLED(100, 5);
    } else {

      // Convert raw to 2's complement, since temperature can be negative
      if ( raw_temp > 0x3FF ) {
        raw_temp |= 0xF800;
      }

      // Convert to float temperature value (Celsius)
      temp_c = (int16_t)raw_temp * 0.125;

      // Convert temperature to decimal format (without float conversion)
      temp_c *= 100;
      sprintf((char*)buf,
            "%u.u C\r\n",
            ((unsigned int)temp_c / 100),
            ((unsigned int)temp_c % 100));

      // Print temperature to console
      CDC_Transmit_FS((uint8_t*)buf, strlen(buf));

      // Turn LED on while writing to file
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      fres = AppendToFile(log_path, strlen(log_path), buf, strlen(buf));
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

      // If error writing to card, blink 3 times
      if ( fres != FR_OK) {
        BlinkLED(200, 3);
      }
    }

    // Wait before sampling again
    HAL_Delay(1000);

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
  ...
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  ...
}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{
  ...
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  ...
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  ...
}

/* USER CODE BEGIN 4 */

// Read temperature from PCT2075
uint16_t ReadPCTTemperature(uint8_t i2c_addr) {

  HAL_StatusTypeDef ret;
  uint8_t buf[2];
  uint16_t val;

  // Tell PCT2075 that we want to read from the temperature register
  buf[0] = PCT_REG_TEMP;
  ret = HAL_I2C_Master_Transmit(&hi2c1, PCT_I2C_ADDR, buf, 1, I2C_DELAY);

  // If the I2C device has just been hot-plugged, reset the peripheral
  if ( ret == HAL_BUSY ) {
    if (HAL_I2C_DeInit(&hi2c1) != HAL_OK){
      Error_Handler();
    }
    MX_I2C1_Init();
  }

  // Throw error if communication not OK
  if ( ret != HAL_OK ) {
    return PCT_ERROR;
  }

  // Read 2 bytes from the temperature register
  ret = HAL_I2C_Master_Receive(&hi2c1, PCT_I2C_ADDR, buf, 2, I2C_DELAY);
  if ( ret != HAL_OK ) {
    return PCT_ERROR;
  }

  // Combine the bytes and return raw value
  val = ((uint16_t)buf[0] << 3) | (buf[1] >> 5);

  return val;
}

// Append string to file given at path
FRESULT AppendToFile(char* path, size_t path_len, char* msg, size_t msg_len) {

  FATFS fs;
  FIL myFILE;
  UINT testByte;
  FRESULT stat;

  // Bounds check on strings
  if ( (path[path_len] != 0) || (msg[msg_len] != 0) ) {
    return FR_INVALID_NAME;
  }

  // Re-initialize SD
  if ( BSP_SD_Init() != MSD_OK ) {
    return FR_NOT_READY;
  }

  // Re-initialize FATFS
  if ( FATFS_UnLinkDriver(SDPath) != 0 ) {
    return FR_NOT_READY;
  }
  if ( FATFS_LinkDriver(&SD_Driver, SDPath) != 0 ) {
    return FR_NOT_READY;
  }

  // Mount filesystem
  stat = f_mount(&fs, SDPath, 0);
  if (stat != FR_OK) {
    f_mount(0, SDPath, 0);
    return stat;
  }

  // Open file for appending
  stat = f_open(&myFILE, path, FA_WRITE | FA_OPEN_APPEND);
  if (stat != FR_OK) {
    f_mount(0, SDPath, 0);
    return stat;
  }

  // Write message to end of file
  stat = f_write(&myFILE, msg, msg_len, &testByte);
  if (stat != FR_OK) {
    f_mount(0, SDPath, 0);
    return stat;
  }

  // Sync, close file, unmount
  stat = f_close(&myFILE);
  f_mount(0, SDPath, 0);

  return stat;
}

// Blink onboard LED
void BlinkLED(uint32_t blink_delay, uint8_t num_blinks) {
  for ( int i = 0; i < num_blinks; i++ ) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    HAL_Delay(blink_delay);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
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
