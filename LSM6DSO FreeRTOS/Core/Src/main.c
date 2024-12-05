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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iks01a3_motion_sensors.h"
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */
typedef struct LSM6DSO_DATA{
	IKS01A3_MOTION_SENSOR_Axes_t axes_gyro;
	IKS01A3_MOTION_SENSOR_Axes_t axes_acce;
} LSM6DSO_data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for gettingDataLSM6 */
osThreadId_t gettingDataLSM6Handle;
const osThreadAttr_t gettingDataLSM6_attributes = {
  .name = "gettingDataLSM6",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 768 * 4
};
/* Definitions for sendingDataLSM6 */
osThreadId_t sendingDataLSM6Handle;
const osThreadAttr_t sendingDataLSM6_attributes = {
  .name = "sendingDataLSM6",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for QueueDataLSM6DSO */
osMessageQueueId_t QueueDataLSM6DSOHandle;
uint8_t QueueDataLSM6DSOBuffer[ 16 * sizeof( LSM6DSO_data ) ];
osStaticMessageQDef_t QueueDataLSM6DSOControlBlock;
const osMessageQueueAttr_t QueueDataLSM6DSO_attributes = {
  .name = "QueueDataLSM6DSO",
  .cb_mem = &QueueDataLSM6DSOControlBlock,
  .cb_size = sizeof(QueueDataLSM6DSOControlBlock),
  .mq_mem = &QueueDataLSM6DSOBuffer,
  .mq_size = sizeof(QueueDataLSM6DSOBuffer)
};
/* USER CODE BEGIN PV */
int32_t gyrxcalib = 0, gyrycalib = 0, gyrzcalib = 0, accxcalib = 0, accycalib = 0, acczcalib = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartgettingDataLSM6DSO(void *argument);
void StartsendingDataLSM6(void *argument);

/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len);
void CalibrationLSM6DSO();
LSM6DSO_data CalibratedGet(LSM6DSO_data mov_data);
LSM6DSO_data InitLSM6DSO_Struct(LSM6DSO_data _struct);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
int DataIdx;
for (DataIdx = 0; DataIdx < len; DataIdx++)
{
//__io_putchar(*ptr++);
ITM_SendChar(*ptr++);
}
return len;
}

void CalibrationLSM6DSO(){
	LSM6DSO_data calib_data;
	calib_data = InitLSM6DSO_Struct(calib_data);


	for(int i = 0; i<50; i++){
		IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LSM6DSO_0, MOTION_GYRO, &calib_data.axes_gyro);
		IKS01A3_MOTION_SENSOR_GetAxes(1, MOTION_ACCELERO, &calib_data.axes_acce);
	}


	for(int i = 0; i<100; i++){
		IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LSM6DSO_0, MOTION_GYRO, &calib_data.axes_gyro);
		IKS01A3_MOTION_SENSOR_GetAxes(1, MOTION_ACCELERO, &calib_data.axes_acce);
		gyrxcalib += calib_data.axes_gyro.x;
//		printf("%d : %ld\n", i, gyrxcalib);
		gyrycalib += calib_data.axes_gyro.y;
		gyrzcalib += calib_data.axes_gyro.z;
		accxcalib += calib_data.axes_acce.x;
		accycalib += calib_data.axes_acce.y;
		acczcalib += calib_data.axes_acce.z;
	}

	gyrxcalib = gyrxcalib/100;
	gyrycalib = gyrycalib/100;
	gyrzcalib = gyrzcalib/100;
	accxcalib = accxcalib/100;
	accycalib = accycalib/100;
	acczcalib = acczcalib/100;
	printf("%ld\n", gyrxcalib);
}

LSM6DSO_data InitLSM6DSO_Struct(LSM6DSO_data _struct){
	_struct.axes_acce.x = 0;
	_struct.axes_acce.y = 0;
	_struct.axes_acce.z = 0;
	_struct.axes_gyro.x = 0;
	_struct.axes_gyro.y = 0;
	_struct.axes_gyro.z = 0;
	return _struct;
}

LSM6DSO_data CalibratedGet(LSM6DSO_data mov_data){
	mov_data.axes_acce.x = mov_data.axes_acce.x - accxcalib;
	mov_data.axes_acce.y = mov_data.axes_acce.y - accycalib;
	mov_data.axes_acce.z = mov_data.axes_acce.z - acczcalib;
	mov_data.axes_gyro.x = mov_data.axes_gyro.x - gyrxcalib;
	mov_data.axes_gyro.y = mov_data.axes_gyro.y - gyrycalib;
	mov_data.axes_gyro.z = mov_data.axes_gyro.z - gyrzcalib;
	return mov_data;
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM16_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_GYRO);
  IKS01A3_MOTION_SENSOR_Init(1, MOTION_ACCELERO);

  IKS01A3_MOTION_SENSOR_Enable(IKS01A3_LSM6DSO_0, MOTION_GYRO);
  IKS01A3_MOTION_SENSOR_Enable(1, MOTION_ACCELERO);
//  IKS01A3_LIS2DW12_0;
  CalibrationLSM6DSO();
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_Base_Start_IT(&htim2);
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

  /* Create the queue(s) */
  /* creation of QueueDataLSM6DSO */
  QueueDataLSM6DSOHandle = osMessageQueueNew (16, sizeof(LSM6DSO_data), &QueueDataLSM6DSO_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of gettingDataLSM6 */
  gettingDataLSM6Handle = osThreadNew(StartgettingDataLSM6DSO, NULL, &gettingDataLSM6_attributes);

  /* creation of sendingDataLSM6 */
  sendingDataLSM6Handle = osThreadNew(StartsendingDataLSM6, NULL, &sendingDataLSM6_attributes);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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
  htim2.Init.Prescaler = 31999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 15999;
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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 31999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|LD3_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD3_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
  GPIO_InitStruct.Pin = STLINK_RX_Pin|STLINK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

/* USER CODE BEGIN Header_StartgettingDataLSM6DSO */
/**
* @brief Function implementing the gettingDataLSM6 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartgettingDataLSM6DSO */
void StartgettingDataLSM6DSO(void *argument)
{
  /* USER CODE BEGIN StartgettingDataLSM6DSO */
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
	  LSM6DSO_data mov_data;
	  mov_data = InitLSM6DSO_Struct(mov_data);
	  IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LSM6DSO_0, MOTION_GYRO, &mov_data.axes_gyro);
	  IKS01A3_MOTION_SENSOR_GetAxes(1, MOTION_ACCELERO, &mov_data.axes_acce);
	  mov_data = CalibratedGet(mov_data);
	  printf("Xgyro: %ld | Ygyro: %ld | Zgyro: %ld | Xacc: %ld | Yacc: %ld | Zacc: %ld\n",
	         mov_data.axes_gyro.x, mov_data.axes_gyro.y, mov_data.axes_gyro.z,
	         mov_data.axes_acce.x, mov_data.axes_acce.y, mov_data.axes_acce.z);
	  printf("Get at : %ld\n", osKernelGetTickCount());
	  osMessageQueuePut(QueueDataLSM6DSOHandle, &mov_data, 1, osWaitForever);
	  osDelay(1);
  }
  /* USER CODE END StartgettingDataLSM6DSO */
}

/* USER CODE BEGIN Header_StartsendingDataLSM6 */
/**
* @brief Function implementing the sendingDataLSM6 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartsendingDataLSM6 */
void StartsendingDataLSM6(void *argument)
{
  /* USER CODE BEGIN StartsendingDataLSM6 */
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
	  LSM6DSO_data send_data;
	  for(int i = 0; i<16; i++){
		  send_data = InitLSM6DSO_Struct(send_data);
		  osMessageQueueGet(QueueDataLSM6DSOHandle, &send_data, (uint8_t*)1, osWaitForever);
		  printf("SEND : Xgyro: %ld | Ygyro: %ld | Zgyro: %ld | Xacc: %ld | Yacc: %ld | Zacc: %ld\n",
				  send_data.axes_gyro.x, send_data.axes_gyro.y, send_data.axes_gyro.z,
				  send_data.axes_acce.x, send_data.axes_acce.y, send_data.axes_acce.z);
	  }
	  printf("Send at : %ld\n", osKernelGetTickCount());
	  osDelay(1);
  }
  /* USER CODE END StartsendingDataLSM6 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM16) {
		osThreadFlagsSet(gettingDataLSM6Handle, 1);
	}else if(htim->Instance == TIM2){
		osThreadFlagsSet(sendingDataLSM6Handle, 1);
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
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
