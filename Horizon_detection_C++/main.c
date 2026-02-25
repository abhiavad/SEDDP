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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>
#include "MLX90640_API.h"
#include "horizon_detection.h"
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
#define  FPS2HZ   0x02
#define  FPS4HZ   0x03
#define  FPS8HZ   0x04
#define  FPS16HZ  0x05
#define  FPS32HZ  0x06

#define  MLX90640_ADDR 0x33
#define	 RefreshRate FPS16HZ
#define  TA_SHIFT 8 //Default shift for MLX90640 in open air

#define M_PI 3.14159265358979323846
static uint16_t eeMLX90640[832];
static float mlx90640To[768];
uint16_t frame[834];             
float package[778];              
float emissivity = 0.95f;
// float tr = 23.15f;              reflected temperature, needs updating


paramsMLX90640 mlx90640;         

AttitudeResult current_att;      

int status;
int RefreshRate = 0x04;

volatile bool uart_ready = true;
volatile bool interruptArduino = false;
volatile bool interruptArduino_package = false;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//void send_message_UART(const char *message){
//	HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
//}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) { // With the help of DMA this callback allows another message to be sent (to not have overlap)
    if (huart->Instance == USART2) {
        uart_ready = true;
    }
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // Listens for an interrupt from the Arduino
//{
//  if (GPIO_Pin == GPIO_PIN_0)
//  {
//	  interruptArduino = true;
//  }
//}

void send_message_UART(const char *format, ...) {
    char buffer[128];  // Adjust buffer size as needed
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    // Send buffer over UART
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);  // Replace with your actual UART sending function
}

void print_eeprom_data(uint16_t *eeMLX90640, int size) {
    for (int i = 0; i < size; i++) {
        send_message_UART("EEPROM[%03d]: 0x%04X\n", i, eeMLX90640[i]);
        HAL_Delay(5);  // Small delay to prevent UART buffer overflow
    }
}

void printMLX90640Params(const paramsMLX90640 *params) {
    send_message_UART("kVdd: %d\n", params->kVdd);
    send_message_UART("vdd25: %d\n", params->vdd25);
    send_message_UART("KvPTAT: %.6f\n", params->KvPTAT);
    send_message_UART("KtPTAT: %.6f\n", params->KtPTAT);
    send_message_UART("vPTAT25: %u\n", params->vPTAT25);
    send_message_UART("alphaPTAT: %.6f\n", params->alphaPTAT);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
/* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart2, (uint8_t *)"Starting Earth Sensor Init...\n", 30, HAL_MAX_DELAY);

  // 1. Check if hardware is physically there
  if (HAL_I2C_IsDeviceReady(&hi2c1, (MLX90640_ADDR << 1), 3, 100) != HAL_OK) {
      send_message_UART("Error: MLX90640 not detected on I2C1\n");
  } else {
      send_message_UART("I2C Device Ready\n");
  }

  // 2. Set Basic Hardware Configuration
  MLX90640_SetResolution(MLX90640_ADDR, 0x03); 
  MLX90640_SetChessMode(MLX90640_ADDR);        
  
  // Apply 8Hz (0x04) and capture the result in 'status'
  status = MLX90640_SetRefreshRate(MLX90640_ADDR, 0x04); 
  
  if (status != 0) {
      send_message_UART("Refresh Rate Set error:%d\r\n", status);
  } else {
      send_message_UART("Refresh Rate changed to 8Hz!\n");
  }

  // 3. Load Calibration Data from Sensor EEPROM
  status = MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);
  if (status != 0) {
      send_message_UART("EEPROM Dump Failed! Code: %d\n", status);
  } else {
      send_message_UART("EEPROM Loaded successfully\n");
  }

  // 4. Extract Parameters into the RAM structure
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0) {
      send_message_UART("Parameter Extraction Failed! Code: %d\n", status);
  } else {
      send_message_UART("Thermal Parameters Extracted\n");
  }

  // 5. Initialize algorithm variables
  startTime = HAL_GetTick();
  frameCounter = 0;
  /* USER CODE END 2 */
/* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
 {
   /* 1. Poll the sensor for new data. 
      This function replaces your manual status checks. */
   int status = MLX90640_GetFrameData(MLX90640_ADDR, mlx90640Frame);
   
   if (status >= 0) 
   {
       /* 2. Melexis API: Raw data -> Temperature Array (mlx90640To) */
       MLX90640_CalculateTo(mlx90640Frame, &mlx90640, 0.95f, 23.15f, mlx90640To);
       
       /* 3. YOUR ALGORITHM: Analyze the temperatures for Pitch/Roll */
       AttitudeResult current_att = calculate_attitude(mlx90640To);
       
       /* 4. Result Handling */
       if (current_att.valid) {
    // 1. Toggle LED
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); 
    
    // 2. Signal Arduino if needed
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); 
    // (Remember to reset the pin later or use a pulse)
}
   }
   else 
   {
       // Optional: Error reporting if I2C fails
       send_message_UART("GetFrameData ERROR: %d\n", status);
   }
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
 }
  /* USER CODE END 3 */
	//Calculate the time of the test after taking the measurement
	float currTime = HAL_GetTick();
	testTime = currTime - startTime;

	float vdd = MLX90640_GetVdd(frame, &mlx90640);
	float Ta = MLX90640_GetTa(frame, &mlx90640);

//	char msg[64];
//	snprintf(msg, sizeof(msg), "Ta: %.2f, Vdd: %.2f,\n",Ta, vdd);
//	send_message_UART(msg);
//	send_message_UART("Ta: %.2f, Vdd: %.2f,\n",Ta, vdd);

	float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
	//printf("vdd:  %f Tr: %f\r\n",vdd,tr);
	MLX90640_CalculateTo(frame, &mlx90640, emissivity , tr, mlx90640To);


	//for(int i = 0; i < 768; i++){
//		char msg[64];
//		snprintf(msg, sizeof(msg), "%2.2f, ",mlx90640To[i]);
//		send_message_UART(msg);
		//HAL_Delay(1);
	//}

    double data[768];
    for (int i = 0; i < 768; ++i) {
        data[i] = (double)mlx90640To[i];
    }
    direction = compute_pitch(&data);
//    direction = 0.0;
    nadir = compute_roll(data, direction);
//    nadir = 0.0;
    direction *= 180 / M_PI;
    nadir *= 180 / M_PI;

	//SEND THE DATA HERE ...

	for(int i = 0; i < 768; i++){ 					// Pixel temperature data (in degC)
		package[i] = mlx90640To[i];
	}
	package[768] = frame[832]; 						// Control register 1 (0x800D)
	package[769] = frame[833]; 						// Last measured subpage
	package[770] = direction;							// Pitch (in deg)
	package[771] = nadir;						// Roll (in deg)
	package[772] = Ta;								// Ta (in degC)
	package[773] = vdd;								// Vdd (in V)
	package[774] = testTime;						// Duration of the current test (in milis)
	package[775] = (float)interruptArduino_package;	// bool of the interrupt pin from Arduino
	package[776] = (float)frameCounter;				// Counter of the frames (for continuity check)

	uint8_t raw_bytes[777 * 4 + 2]; // extra 2 for \r\n
	memcpy(raw_bytes, package, 777 * 4);
	raw_bytes[777 * 4] = '\r';
	raw_bytes[777 * 4 + 1] = '\n';

	if (uart_ready) {
	    uart_ready = false;
	    HAL_UART_Transmit_DMA(&huart2, raw_bytes, sizeof(raw_bytes));
	}
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
