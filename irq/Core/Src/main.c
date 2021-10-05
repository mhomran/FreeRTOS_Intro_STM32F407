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
#include <string.h>
#include "queue.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLES_NO 10
#define Avg_Slope .0025
#define V25 0.76
#define VSENSE 2.96/4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim7;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
QueueHandle_t Queue1, Queue2, ReceiveQueue;
SemaphoreHandle_t Sem, TempMutex;
float temp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void USART1_Init(void);
void USART1_Enable(void);
static void strTransmit(const char * str, uint8_t size);

void TIM7_IRQHandler(void);
void TaskA(void* arguments);
void TaskB(void* arguments);
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
  MX_ADC1_Init();
  MX_TIM7_Init();
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
  Sem = xSemaphoreCreateBinary();
  TempMutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  Queue1 = xQueueCreate(SAMPLES_NO, sizeof(uint16_t));
  Queue2 = xQueueCreate(SAMPLES_NO, sizeof(uint16_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(TaskA,
  	     "Serial Task",
  	     128,
  	     NULL,
  	     osPriorityNormal,
  	     NULL);
  xTaskCreate(TaskB,
    	     "Serial Task",
    	     256,
    	     NULL,
    	     osPriorityNormal,
    	     NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  HAL_TIM_Base_Start_IT(&htim7);
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1600;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

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

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
  uint16_t value;
  BaseType_t TaskWoken;

  HAL_ADC_Start(&hadc1);
  if(HAL_ADC_PollForConversion(&hadc1, 0xFFFF) != HAL_TIMEOUT)
    {
      value = HAL_ADC_GetValue(&hadc1);

      if(xQueueIsQueueFullFromISR(Queue1) == pdFALSE)
	{
	  xQueueSendFromISR(Queue1, &value, NULL);
	}
      else
	{
	  xQueueSendFromISR(Queue2, &value, NULL);
	}

      if(xQueueIsQueueFullFromISR(Queue1) == pdTRUE)
	{
	  ReceiveQueue = Queue1;
	  xSemaphoreGiveFromISR(Sem, &TaskWoken);
	}
      else if(xQueueIsQueueFullFromISR(Queue2) == pdTRUE)
      	{
      	  ReceiveQueue = Queue2;
      	  xSemaphoreGiveFromISR(Sem, &TaskWoken);
      	}
    }
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
  portYIELD_FROM_ISR(TaskWoken);
  /* USER CODE END TIM7_IRQn 1 */
}

void
TaskA(void* arguments)
{
  uint16_t rx;
  uint32_t avg;

  while(1)
    {
      xSemaphoreTake(Sem, portMAX_DELAY);

      avg = 0;
      while(xQueueReceive(ReceiveQueue, &rx, 0) == pdTRUE)
	{
	  avg += rx;
	}

      avg /= SAMPLES_NO;

      xSemaphoreTake(TempMutex, portMAX_DELAY);
      temp = ((VSENSE*avg - V25) / Avg_Slope) + 25;
      xSemaphoreGive(TempMutex);
    }
}

void
TaskB(void *argument)
{
  uint8_t data[100];
  uint8_t idx = 0;

  uint8_t rx;
  uint8_t i;
  uint8_t match;
  const char sAvg[] = "avg";

  while(1)
    {
      // Read serial input from user and echo it back
      if((USART1->SR & USART_SR_RXNE) == USART_SR_RXNE)
      	{
	  rx = USART1->DR;

	  if(rx == '\r')
	    {
	      match = 1;
	      if(idx == strlen(sAvg))
		{
		  for(i = 0; i < strlen(sAvg); i++)
		    {
		      if(sAvg[i] != data[i])
			{
			  match = 0;
			  break;
			}
		    }
		}
	      else
		{
		  match = 0;
		}

	      printf("\r\n");
	      if(match == 1)
		{
		  xSemaphoreTake(TempMutex, portMAX_DELAY);
		  printf("%f\r\n", temp);
		  xSemaphoreGive(TempMutex);
		}

	      idx = 0;
	    }
	  else
	    {
	      strTransmit((char*)&rx, sizeof(rx));
	      data[idx] = rx;
	      idx = idx + 1 % 100;
	    }
      	}

      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
int
_write(int file, char *ptr, int len)
{
  strTransmit(ptr, len);

  return len;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
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
