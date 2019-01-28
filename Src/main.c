/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include "stm32f0xx_hal_gpio.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// If USE_SWD_PRINTF is defined, the target device will not work without
// ST-Link connected via SWD.
//#define USE_SWD_PRINTF
//#define USE_CDC_PRINTF
//#define USE_SWD

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

int boot_to_dfu = 0;

void __initialize_hardware_early(void)
{
  if (boot_to_dfu) {
    boot_to_dfu = 0;
    ((void (*)(void))0x1fffc400)();
  } else {
    SystemInit();
  }
}

#ifdef USE_SWD_PRINTF
extern void initialise_monitor_handles();
#endif

#ifdef USE_CDC_PRINTF
#define printf CDC_Printf
#endif

int CDC_Printf(const char* format, ...) {
  char buf[128];
  va_list arg;
  va_start(arg, format);
  int size = vsnprintf(buf, 126, format, arg);
  if (buf[size - 1] == '\n') {
    buf[size - 1] = '\r';
    buf[size] = '\n';
    size++;
  }
  buf[size] = 0;
  CDC_Puts(buf);
  return size;
}

void ActivateAddress() {
  HAL_GPIO_WritePin(NADR_GPIO_Port, NADR_Pin, GPIO_PIN_RESET);
}

void InactivateAddress() {
  HAL_GPIO_WritePin(NADR_GPIO_Port, NADR_Pin, GPIO_PIN_SET);
}

void SetAddress(uint16_t address) {
  HAL_GPIO_WritePin(ACLK_GPIO_Port, ACLK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ARCK_GPIO_Port, ARCK_Pin, GPIO_PIN_RESET);
  int i;
  for (i = 0; i < 16; ++i) {
    HAL_GPIO_WritePin(ADAT_GPIO_Port, ADAT_Pin, (address & 0x8000) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    address <<= 1;
    //
    HAL_GPIO_WritePin(ACLK_GPIO_Port, ACLK_Pin, GPIO_PIN_SET);
    //
    HAL_GPIO_WritePin(ACLK_GPIO_Port, ACLK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ARCK_GPIO_Port, ARCK_Pin, GPIO_PIN_SET);
    //
    HAL_GPIO_WritePin(ARCK_GPIO_Port, ARCK_Pin, GPIO_PIN_RESET);
  }
}

void ActivateData() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15
#ifdef USE_SWD
			  ;
#else
			  |GPIO_PIN_13|GPIO_PIN_14;
#endif
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}

void InactivateData() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15
#ifdef USE_SWD
			  ;
#else
                          |GPIO_PIN_13|GPIO_PIN_14;
#endif
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}

void ActivateControl() {
  HAL_GPIO_WritePin(NW_GPIO_Port, NW_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NOE_GPIO_Port, NOE_Pin, GPIO_PIN_SET);

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = NW_Pin|NOE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NW_GPIO_Port, &GPIO_InitStruct);
}

void InactivateControl() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = NW_Pin|NOE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NW_GPIO_Port, &GPIO_InitStruct);
}

void LockRAM() {
  HAL_GPIO_WritePin(Lock_GPIO_Port, Lock_Pin, GPIO_PIN_SET);
  ActivateAddress();
  ActivateControl();
}

void WriteRAM(uint16_t address, uint16_t data) {
  SetAddress(address);

  ActivateData();
  uint16_t a = (data & 0x07ff) | ((data << 2) & 0xe000);
  uint16_t f = (data >> 14) & 0x0003;
#ifdef USE_SWD
  GPIOA->ODR = (GPIOA->ODR & 0xffff7800) | (a & 0x87ff);
#else
  GPIOA->ODR = (GPIOA->ODR & 0xffff1800) | a;
#endif
  GPIOF->ODR = (GPIOF->ODR & 0xfffffffc) | f;

  HAL_GPIO_WritePin(NW_GPIO_Port, NW_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NW_GPIO_Port, NW_Pin, GPIO_PIN_SET);

  InactivateData();
}

uint16_t ReadRAM(uint16_t address) {
  SetAddress(address);
  HAL_GPIO_WritePin(NOE_GPIO_Port, NOE_Pin, GPIO_PIN_RESET);
  uint16_t a = GPIOA->IDR;
  uint16_t f = GPIOF->IDR;
  HAL_GPIO_WritePin(NOE_GPIO_Port, NOE_Pin, GPIO_PIN_SET);
  return (a & 0x07ff) | ((a >> 2) & 0x3800) | (f << 14);
}

void UnlockRAM() {
  InactivateData();
  InactivateControl();
  InactivateAddress();
  HAL_GPIO_WritePin(Lock_GPIO_Port, Lock_Pin, GPIO_PIN_RESET);
}

void SetLEDOn() {
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void SetLEDOff() {
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void ActivateSWD() {
  HAL_GPIO_WritePin(Lock_GPIO_Port, Lock_Pin, GPIO_PIN_SET);

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF0_SWDIO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Alternate = GPIO_AF0_SWCLK;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
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
#ifdef USE_SWD_PRINTF
  initialise_monitor_handles();
#endif

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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  SetLEDOff();

  uint16_t address = 0;
  uint16_t data = 0;
  uint16_t burst = 0;
  uint8_t buf[64];
  while (1)
  {
    if (CDC_GetReceivedChar() != '@') {
      CDC_Puts("-");
      continue;
    }
    switch (CDC_GetReceivedChar()) {
    case 'L':  // LED ON
      SetLEDOn();
      break;
    case 'l':  // LED OFF
      SetLEDOff();
      break;
    case 'B':  // Boot to DFU
      boot_to_dfu = 1;
      NVIC_SystemReset();
      break;
    case 'S':  // Activate SWD
      ActivateSWD();
      break;
    case 'O':  // Lock
      LockRAM();
      break;
    case 'C':  // Unlock
      UnlockRAM();
      break;
    case 'A':  // Set address <High, Low>
      address = (CDC_GetReceivedChar() << 8) | CDC_GetReceivedChar();
      break;
    case 'a':  // Reset address to 0
      address = 0;
      break;
    case 'W':  // Write <Len - 1, [High, Low]+> and increment address
      burst = CDC_GetReceivedChar() + 1;
      for (size_t i = 0; i < burst; ++i) {
        data = (CDC_GetReceivedChar() << 8) | CDC_GetReceivedChar();
        WriteRAM(address++, data);
      }
      break;
    case 'w':  // Write <High, Low> and increment address
      data = (CDC_GetReceivedChar() << 8) | CDC_GetReceivedChar();
      WriteRAM(address++, data);
      break;
    case 'R':  // Read <Len - 1> and increment address
      burst = CDC_GetReceivedChar() + 1;
      if (burst > 16) {
        CDC_Puts("-");
        continue;
      }
      buf[0] = '!';
      for (size_t i = 0; i < burst; ++i) {
        data = ReadRAM(address++);
        buf[1 + i * 2] = data >> 8;
        buf[2 + i * 2] = data;
      }
      buf[1 + burst * 2] = '+';
      while (USBD_OK != CDC_Transmit_FS(buf, 2 + burst * 2));
      continue;
    case 'r':  // Read and increment address
      data = ReadRAM(address++);
      buf[0] = '!';
      buf[1] = data >> 8;
      buf[2] = data;
      buf[3] = '+';
      while (USBD_OK != CDC_Transmit_FS(buf, 4));
      continue;
    case 'x':
      {
	LockRAM();
	for (int i = 0; i < 0x10000; ++i)
	  WriteRAM(i, i);
	for (int i = 0; i < 0x10000; ++i) {
	  data = ReadRAM(i);
	  if (data == i)
	    continue;
	  CDC_Printf("@$%04x: $%04x\n", i, data);
	}
	UnlockRAM();
      }
      break;
    case 'y':
      {
	LockRAM();
	for (int i = 0; i < 0x10000; ++i)
	  WriteRAM(i, i);
	UnlockRAM();
      }
      break;
    case 'z':
      {
	LockRAM();
	for (int i = 0; i < 0x10000; ++i) {
	  data = ReadRAM(i);
	  if (data == i)
	    continue;
	  CDC_Printf("@$%04x: $%04x\n", i, data);
	}
	UnlockRAM();
      }
      break;
    default:
      CDC_Puts("-");
      continue;
    }
    CDC_Puts("+");
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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /**Enable the SYSCFG APB clock 
  */
  __HAL_RCC_CRS_CLK_ENABLE();
  /**Configures CRS 
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|ADAT_Pin|ACLK_Pin|ARCK_Pin 
                          |Lock_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NADR_GPIO_Port, NADR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PF0 PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 PA7 
                           PA8 PA9 PA10 PA13 
                           PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15
#ifdef USE_SWD
			  ;
#else
			  |GPIO_PIN_13|GPIO_PIN_14;
#endif
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADAT_Pin ACLK_Pin ARCK_Pin NADR_Pin 
                           Lock_Pin */
  GPIO_InitStruct.Pin = ADAT_Pin|ACLK_Pin|ARCK_Pin|NADR_Pin 
                          |Lock_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : NW_Pin NOE_Pin */
  GPIO_InitStruct.Pin = NW_Pin|NOE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
