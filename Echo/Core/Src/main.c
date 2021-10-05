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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
# include <string.h>
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
/* Definitions for hReadData */
osThreadId_t hReadDataHandle;
const osThreadAttr_t hReadData_attributes = {
  .name = "hReadData",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for hWriteData */
osThreadId_t hWriteDataHandle;
const osThreadAttr_t hWriteData_attributes = {
  .name = "hWriteData",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
void* SharedData;
uint8_t SharedFlag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void ReadData(void *argument);
void WriteData(void *argument);

/* USER CODE BEGIN PFP */
void USART1_Init(void);
void USART1_Enable(void);
static void strTransmit(const char * str, uint8_t size);
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
  /* USER CODE BEGIN 2 */
  USART1_Init();
  USART1_Enable();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of hReadData */
  hReadDataHandle = osThreadNew(ReadData, NULL, &hReadData_attributes);

  /* creation of hWriteData */
  hWriteDataHandle = osThreadNew(WriteData, NULL, &hWriteData_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief   Configure USART1 for ST virtual COM port (VCP)
 * @note
 * @param   None
 * @retval  None
 */
void
USART1_Init(void)
{
  /* Configure USART1 */
  /* Enable USART1 clock */
  RCC->APB2ENR = RCC_APB2ENR_USART1EN;

  /* Select oversampling by 16 mode */
  USART1->CR1 &= ~USART_CR1_OVER8;

  /* Select one sample bit method */
  USART1->CR3 |= USART_CR3_ONEBIT;

  /* Select 1 Start bit, 9 Data bits, n Stop bit */
  USART1->CR1 |= USART_CR1_M;

  /* Select 1 stop bit */
  USART1->CR2 &= ~USART_CR2_STOP;

  /* Enable parity control */
  USART1->CR1 |= USART_CR1_PCE;

  /* Select odd parity */
  USART1->CR1 |= USART_CR1_PS;

  /* Set baud rate = 115200 Bps
   * USARTDIV = Fck / (16 * baud_rate)
   *          = 16000000 / (16 * 9600) = 104.16
   *
   * DIV_Fraction = 16 * 0.16 = 2.56 = 3 = 0x3
   * DIV_Mantissa = 104 = 0x68
   *
   * BRR          = 0x683 */
  USART1->BRR = 0x683;
}

/**
 * @brief   Enable USART1 transmitter and receiver
 * @note
 * @param   None
 * @retval  None
 */
void USART1_Enable(void)
{
  /* Enable USART1 */
  USART1->CR1 |= USART_CR1_UE;

  /* Enable transmitter */
  USART1->CR1 |= USART_CR1_TE;

  /* Enable receiver */
  USART1->CR1 |= USART_CR1_RE;
}

/**
 * @brief   String transmit
 * @note
 * @param   str, size
 * @retval  None
 */
static void strTransmit(const char * str, uint8_t size)
{
  /* Check null pointers */
  if(NULL != str)
    {
      /* Send all string characters */
      for(int idx = 0; idx < size; idx++)
      {
	/* Check USART status register */
	while(!(USART1->SR & USART_SR_TXE))
	{
	  /* Wait for transmission buffer empty flag */
	}

	/* Write data into transmit data register */
	USART1->DR = str[idx];
      }
    }
  else
    {
      /* Null pointers, do nothing */
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_ReadData */
/**
  * @brief  Function implementing the hReadData thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ReadData */
void ReadData(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint8_t data[100];
  uint8_t rx;
  uint8_t idx;
  idx = 0;
  /* Infinite loop */
  for(;;)
    {
      /* Check USART receiver */
      if((USART1->SR & USART_SR_RXNE) == USART_SR_RXNE)
	{
	  rx = USART1->DR;
	  if(rx == '\r')
	    {
	      data[idx] = '\n';
	      idx++;
	      data[idx] = '\r';
	      idx++;
	      data[idx] = '\0';
	      idx++;

	      SharedData = pvPortMalloc(idx);
	      memcpy(SharedData, data, idx);
	      SharedFlag = 1;

	      idx = 0;
	    }
	  else
	    {
	      data[idx] = rx;
	      idx++;
	    }
	}
      else
	{
	  /* No new data received */
	}
      osDelay(1);
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_WriteData */
/**
* @brief Function implementing the hWriteData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WriteData */
void WriteData(void *argument)
{
  /* USER CODE BEGIN WriteData */
  /* Infinite loop */
  for(;;)
    {
      if(SharedFlag == 1)
	{
	  strTransmit(SharedData, strlen(SharedData));
	  vPortFree(SharedData);
	  SharedData = NULL;
	  SharedFlag = 0;
	}

      osDelay(1);
    }
  /* USER CODE END WriteData */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
