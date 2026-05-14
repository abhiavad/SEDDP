/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include "RM3100.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// this struct is the structure of the data sent through the i2cQueue. It has enough into to tell TaskHigh where to put new info in the i2c register
// ONLY WORKS WITH FLOATS!!!
typedef struct {
	uint8_t position;
	float value;
} dataLoggerQueue_t;

// this struct has values put into it only by RM3100Reader. The Controller task reads it.
typedef struct {
	float bX;
	float bY;
	float bZ;
	float timestamp;
} rmReadingQueue_t;

// this struct can only be written to by MLXReader. The Controller task reads it.
typedef struct {
	float roll;
	float pitch;
	float timestamp;
} mlxReadingQueue_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for DataLogger */
osThreadId_t DataLoggerHandle;
const osThreadAttr_t DataLogger_attributes = {
  .name = "DataLogger",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MLXReader */
osThreadId_t MLXReaderHandle;
const osThreadAttr_t MLXReader_attributes = {
  .name = "MLXReader",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for RM3100Reader */
osThreadId_t RM3100ReaderHandle;
const osThreadAttr_t RM3100Reader_attributes = {
  .name = "RM3100Reader",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for MTQDriver */
osThreadId_t MTQDriverHandle;
const osThreadAttr_t MTQDriver_attributes = {
  .name = "MTQDriver",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Controller */
osThreadId_t ControllerHandle;
const osThreadAttr_t Controller_attributes = {
  .name = "Controller",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dataLoggerQueue */
osMessageQueueId_t dataLoggerQueueHandle;
const osMessageQueueAttr_t dataLoggerQueue_attributes = {
  .name = "dataLoggerQueue"
};
/* Definitions for rmReadingQueue */
osMessageQueueId_t rmReadingQueueHandle;
const osMessageQueueAttr_t rmReadingQueue_attributes = {
  .name = "rmReadingQueue"
};
/* Definitions for mtqQueue */
osMessageQueueId_t mtqQueueHandle;
const osMessageQueueAttr_t mtqQueue_attributes = {
  .name = "mtqQueue"
};
/* Definitions for mlxReadingQueue */
osMessageQueueId_t mlxReadingQueueHandle;
const osMessageQueueAttr_t mlxReadingQueue_attributes = {
  .name = "mlxReadingQueue"
};
/* Definitions for MagneticMutex */
osMutexId_t MagneticMutexHandle;
const osMutexAttr_t MagneticMutex_attributes = {
  .name = "MagneticMutex"
};
/* Definitions for i2c1Sem */
osSemaphoreId_t i2c1SemHandle;
const osSemaphoreAttr_t i2c1Sem_attributes = {
  .name = "i2c1Sem"
};
/* Definitions for ControllerStart */
osEventFlagsId_t ControllerStartHandle;
const osEventFlagsAttr_t ControllerStart_attributes = {
  .name = "ControllerStart"
};
/* Definitions for MTQStart */
osEventFlagsId_t MTQStartHandle;
const osEventFlagsAttr_t MTQStart_attributes = {
  .name = "MTQStart"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
void StartDataLogger(void *argument);
void StartMLXReader(void *argument);
void StartRM3100Reader(void *argument);
void StartMTQDriver(void *argument);
void StartController(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2C_EnableListen_IT(&hi2c3); // enable listening on the I2C3 channel to respond to Moteino requests
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of MagneticMutex */
  MagneticMutexHandle = osMutexNew(&MagneticMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of i2c1Sem */
  i2c1SemHandle = osSemaphoreNew(1, 1, &i2c1Sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of dataLoggerQueue */
  dataLoggerQueueHandle = osMessageQueueNew (16, sizeof(dataLoggerQueue_t), &dataLoggerQueue_attributes);

  /* creation of rmReadingQueue */
  rmReadingQueueHandle = osMessageQueueNew (6, sizeof(rmReadingQueue_t), &rmReadingQueue_attributes);

  /* creation of mtqQueue */
  mtqQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &mtqQueue_attributes);

  /* creation of mlxReadingQueue */
  mlxReadingQueueHandle = osMessageQueueNew (6, sizeof(mlxReadingQueue_t), &mlxReadingQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */



  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DataLogger */
  DataLoggerHandle = osThreadNew(StartDataLogger, NULL, &DataLogger_attributes);

  /* creation of MLXReader */
  MLXReaderHandle = osThreadNew(StartMLXReader, NULL, &MLXReader_attributes);

  /* creation of RM3100Reader */
  RM3100ReaderHandle = osThreadNew(StartRM3100Reader, NULL, &RM3100Reader_attributes);

  /* creation of MTQDriver */
  MTQDriverHandle = osThreadNew(StartMTQDriver, NULL, &MTQDriver_attributes);

  /* creation of Controller */
  ControllerHandle = osThreadNew(StartController, NULL, &Controller_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of ControllerStart */
  ControllerStartHandle = osEventFlagsNew(&ControllerStart_attributes);

  /* creation of MTQStart */
  MTQStartHandle = osEventFlagsNew(&MTQStart_attributes);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10D19CE4;
  hi2c3.Init.OwnAddress1 = 36;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|D12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, D1_NSLP_Pin|D1_PH_Pin|D1_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin D12_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|D12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RM3100_DRDY_Pin */
  GPIO_InitStruct.Pin = RM3100_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RM3100_DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_NSLP_Pin D1_PH_Pin D1_EN_Pin */
  GPIO_InitStruct.Pin = D1_NSLP_Pin|D1_PH_Pin|D1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDataLogger */
/**
  * @brief  Function implementing the DataLogger thread.
  * @param  argument: Not used
  * @retval None
  *
  * For now, this function only logs whatever data is sent to it into the I2C register,
  * which are meant to be transmitted to the Moteino.
  * Flash memory logging is not yet implemented but one possible implementation is just writing the entire
  * I2C register into the flash memory.
  */
/* USER CODE END Header_StartDataLogger */
void StartDataLogger(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint8_t I2cregisters[I2C_REGISTER_SIZE]; // todo: instead of this, make a struct type in the moteino slave file, make an object here, and change the functions to change the object, not a hard-coded array. i don't like loosely declaring global variables
  dataLoggerQueue_t msg;
  /* Infinite loop */
  for(;;)
  {
	  if(osMessageQueueGet(dataLoggerQueueHandle, &msg, 0, osWaitForever) == osOK) {
		  if(msg.position > I2C_REGISTER_SIZE || msg.position < 0) // make sure that no stupid mistakes were made
		  {
			  printf("Exception: data logger was called to store %f at %d, but maximum address is %d", msg.value, msg.position, I2C_REGISTER_SIZE);
			  while(1); // TODO: DO NOT leave this in for the real thing. it WILL ruin your mission by freezing the ADCS forever. keep it for development ONLY
		  }
		  else
		  {
			  PrepareFloatForI2CSending(&(msg.value), &(msg.position), I2cregisters); //todo: maybe implement global timestamp update each time we update anything?...
			  SetMoteinoTXData(I2cregisters);
		  }
	  }
	  osDelay(5); // give it a little breathing room, but idk this may or may not be a good idea
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMLXReader */
/**
* @brief Function implementing the MLXReader thread.
* @param argument: Not used
* @retval None
* Not yet implemented.
*/
/* USER CODE END Header_StartMLXReader */
void StartMLXReader(void *argument)
{
  /* USER CODE BEGIN StartMLXReader */
	  /* Infinite loop */
	  for(;;)
	  {
		osEventFlagsSet(ControllerStartHandle, MLXREADER_EVENT);
		  // do nothing for now, just flash an LED
		HAL_GPIO_TogglePin(D12_GPIO_Port, D12_Pin);
	//	printf("Hello!\n");
	    osDelay(50);
	  }
  /* USER CODE END StartMLXReader */
}

/* USER CODE BEGIN Header_StartRM3100Reader */
/**
* @brief Function implementing the RM3100Reader thread.
* @param argument: Not used
* @retval None
* As of right now, all this task does it is reads the RM3100 values three times, then puts them into the messageQueue to the Controller task
* and logs them too. Although some data processing is left in the task as commented code, this task should not yet take that into account.
*/
/* USER CODE END Header_StartRM3100Reader */
void StartRM3100Reader(void *argument)
{
  /* USER CODE BEGIN StartRM3100Reader */

	// create new magnetometer
	static Magnetometer3Axis magnetometer;
	NewMagnetometer3Axis(&magnetometer, RM3100_DRDY_GPIO_Port, RM3100_DRDY_Pin, RM3100Address);

//	static float currentMagField[3]= {0};
//	static float currentMagFieldDer[3]= {0};
//	static float omega[3] = {0};
//	static float div = 1;

	// the RM3100 needs some attention before it's used (this chunk of code is from ChatGPT)
	uint8_t counts[2] = {0x00, 0xC8}; // 200 cycles
	HAL_I2C_Mem_Write(&hi2c1, (0x21<<1), 0x04, 1, counts, 2, 100); // CCX
	HAL_I2C_Mem_Write(&hi2c1, (0x21<<1), 0x06, 1, counts, 2, 100); // CCY
	HAL_I2C_Mem_Write(&hi2c1, (0x21<<1), 0x08, 1, counts, 2, 100); // CCZ

	// these are variables useful for sending data to the i2c register
	dataLoggerQueue_t bx;
	dataLoggerQueue_t by;
	dataLoggerQueue_t bz;
	dataLoggerQueue_t timestamp;
	bx.position = I2C_BX_POS;
	by.position = I2C_BY_POS;
	bz.position = I2C_BZ_POS;
	timestamp.position = I2C_TIMESTAMP_POS;

	// item to send to the rmReadingQueue
	rmReadingQueue_t reading;
	/* Infinite loop */
	for(;;)
	{
		osMutexAcquire(MagneticMutexHandle, osWaitForever); // get the magnetic mutex before doing anything
		osDelay(10); // give it 10 milliseconds before taking magnetic readings, just in case the MTQs haven't fully turned off yet
	    for(uint8_t i=0;i<RM3100_READINGS_PER_LOOP;i++) {
	      // updating gains and reading the sensor can only be done if the semaphore is acquired (shared with MLXReader)
	      osSemaphoreAcquire(i2c1SemHandle, osWaitForever);
		  RM3100_updateGains(&magnetometer); // maybe could be moved out of the for loop to potentially reduce calculation time
		  RM3100_pollAndReadMagField(&magnetometer, xTaskGetTickCount()/1000.0);
		  osSemaphoreRelease(i2c1SemHandle);

		  // this doesn't require communication on I2C1 so we don't need the semaphore for it
		  RM3100_getMagField(&magnetometer, &(bx.value), &(by.value), &(bz.value));

//		  printf("Tick count: %lu\n", xTaskGetTickCount());

		  // the below is code that was previously used to calculate omega and such. It should now only be used by Controller
		  /*
		  RM3100_getMagFieldDerivative(&magnetometer, &currentMagFieldDer[0], &currentMagFieldDer[1], &currentMagFieldDer[2]);

		  // get omega vector (yes i checked and the cross product is correct, even chatgpt said it's good)
	  	  omega[0] = currentMagField[1] * currentMagFieldDer[2] - currentMagField[2] * currentMagFieldDer[1];
	  	  omega[1] = currentMagField[2] * currentMagFieldDer[0] - currentMagField[0] * currentMagFieldDer[2];
	  	  omega[2] = currentMagField[0] * currentMagFieldDer[1] - currentMagField[1] * currentMagFieldDer[0];

	  	  div = currentMagField[0]*currentMagField[0] + currentMagField[1]*currentMagField[1] + currentMagField[2]*currentMagField[2];
		  if(div == 0) printf("No magnetic field present\n"); else {
			  printf("omega X: %f\n", omega[0]/div);
			  printf("omega Y: %f\n", omega[1]/div);
			  printf("omega Z: %f\n", omega[2]/div);
		  }*/

		  // store the readings
		  reading.bX = bx.value;
		  reading.bY = by.value;
		  reading.bZ = bz.value;
		  reading.timestamp = peekBuffer(&(magnetometer.timeBuffer_), -1);
		  timestamp.value = reading.timestamp;

		  // send the B field to the datalogger
		  osMessageQueuePut(dataLoggerQueueHandle, &bx, 0, 0);
		  osMessageQueuePut(dataLoggerQueueHandle, &by, 0, 0);
		  osMessageQueuePut(dataLoggerQueueHandle, &bz, 0, 0);
		  osMessageQueuePut(dataLoggerQueueHandle, &timestamp, 0, 0);

		  // send the full XYZ reading to the Controller task
		  osMessageQueuePut(rmReadingQueueHandle, &reading, 0, 0);

		  // give it at least 20 milliseconds between measurements
		  osDelay(20);
		}
	  osMutexRelease(MagneticMutexHandle);

	  // basically tell the Controller task that all the data is ready for it to process
	  osEventFlagsSet(ControllerStartHandle, RM3100READER_EVENT);
	  osDelay(LOOP_LENGTH_MS/3);
	}
  /* USER CODE END StartRM3100Reader */
}

/* USER CODE BEGIN Header_StartMTQDriver */
/**
* @brief Function implementing the MTQDriver thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMTQDriver */
void StartMTQDriver(void *argument)
{
  /* USER CODE BEGIN StartMTQDriver */
	uint8_t counter = 0;
	/* Infinite loop */

//	HAL_GPIO_WritePin(D1_NSLP_GPIO_Port, D1_NSLP_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(D1_PH_GPIO_Port, D1_PH_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(D1_EN_GPIO_Port, D1_EN_Pin, GPIO_PIN_RESET);
	for(;;)
	{
	  // wait for the Controller task to let it loose
	  osEventFlagsWait(MTQStartHandle, CONTROLLER_READY_EVENT, osFlagsWaitAll, osWaitForever);
	  osMutexAcquire(MagneticMutexHandle, osWaitForever);
	  counter++;
	  osDelay(LOOP_LENGTH_MS/2);
	  osMutexRelease(MagneticMutexHandle);
	  osDelay(1);
	  // at the end of the loop, before this task goes to sleep, reset the timer to 0
	}
  /* USER CODE END StartMTQDriver */
}

/* USER CODE BEGIN Header_StartController */
/**
* @brief Function implementing the Controller thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartController */
void StartController(void *argument)
{
  /* USER CODE BEGIN StartController */
	/*
	typedef struct {
		float bX;
		float bY;
		float bZ;
		float timestamp;
	} rmReadingQueue_t;

	// this struct can only be written to by MLXReader. The Controller task reads it.
	typedef struct {
		float roll;
		float pitch;
		float timestamp;
	} mlxReadingQueue_t;
	*/

  rmReadingQueue_t RMReadings[3];
  /* Infinite loop */
  for(;;)
  {
	// wait for the RM3100 reader and the MLX reader to be done...
	osEventFlagsWait(ControllerStartHandle, RM3100READER_EVENT | MLXREADER_EVENT, osFlagsWaitAll, osWaitForever);
	printf("\033[H");
	printf("Controller event has fired!\n");
	// extract the RM readings from the queue, remember FIFO
	osMessageQueueGet(rmReadingQueueHandle, &RMReadings[0], 0, 1);
	osMessageQueueGet(rmReadingQueueHandle, &RMReadings[1], 0, 1);
	osMessageQueueGet(rmReadingQueueHandle, &RMReadings[2], 0, 1);

	printf("Get all RMReader values from its queue\nThey are at timestamps: %f, %f, %f\n", RMReadings[0].timestamp, RMReadings[1].timestamp, RMReadings[2].timestamp);
	printf("The time difference in measurements is: %f, %f\n", RMReadings[1].timestamp-RMReadings[0].timestamp, RMReadings[2].timestamp-RMReadings[1].timestamp);
	printf("The readings are:\n");
	for(uint8_t i=0;i<3;i++)
		printf("bX: %f\nbY: %f\nbZ: %f\ntime: %f\n\n", RMReadings[i].bX, RMReadings[i].bY, RMReadings[i].bZ, RMReadings[i].timestamp);

	printf("Free stack of Controller: %lu bytes\n", osThreadGetStackSpace(ControllerHandle));
	printf("Free stack of MLXReader: %lu bytes\n", osThreadGetStackSpace(MLXReaderHandle));
	printf("Free stack of RM3100Reader: %lu bytes\n", osThreadGetStackSpace(RM3100ReaderHandle));
	printf("Free stack of MTQDriver: %lu bytes\n", osThreadGetStackSpace(MTQDriverHandle));
	printf("Free stack of DataLogger: %lu bytes\n", osThreadGetStackSpace(DataLoggerHandle));

    // at the end of the task, set the MTQStart event flag to 1
    osEventFlagsSet(MTQStartHandle, CONTROLLER_READY_EVENT);
    osDelay(1);
  }
  /* USER CODE END StartController */
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
  if (htim->Instance == TIM6)
  {
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
