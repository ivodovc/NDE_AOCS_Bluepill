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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "MPU6050.h"
#include "Fan_Controller.h"
#include "fan_logic.h" // Petr
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
float fan_speed_1, fan_speed_2 = 0;

void control_rotation(float desired_rotation, float actual_rotation) {
	// Prepracovat ako PID kontroler, ak to nebude fungovat
	if (actual_rotation-desired_rotation > 7.5) {
		fan_speed_1 = -1000;
		fan_speed_2 = -1000;
		set_fan_speed(fan_speed_1);
		set_fan_speed2(fan_speed_2);
	}else if (actual_rotation-desired_rotation < -7.5){
		fan_speed_1 = 1000;
		fan_speed_2 = 1000;
		set_fan_speed(fan_speed_1);
		set_fan_speed2(fan_speed_2);
	}else{
		fan_speed_1 = 0;
		fan_speed_2 = 0;
		set_fan_speed(fan_speed_1);
		set_fan_speed2(fan_speed_2);
	}
}

// ADC cast
uint16_t readValue1;
uint16_t readValue2;
uint16_t readValue3;
uint16_t readValue4;
uint16_t readValue5;

uint16_t rawValues[5];
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        readValue1 = rawValues[0];
        readValue2 = rawValues[1];
        readValue3 = rawValues[2];
        readValue4 = rawValues[3];
        readValue5 = rawValues[4];
    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float GyroX, GyroY, GyroZ;
float AccelX, AccelY, AccelZ;
float GyroRoll, GyroPitch, AccelRoll, AccelPitch;
uint32_t ch1_counter_val, ch2_counter_val;

// RX buffer for UART
#define RX_BFR_SIZE 100
char RxBuffer[RX_BFR_SIZE];

uint16_t global_mode = 0; // 0 is stabilization, 1 is light tracking, 2 is rotation
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *) RxBuffer, RX_BFR_SIZE);

  // Set up HC05 bluetooth controller
  printf("Setting up Bluetooth module\n");

  // reset module to original state
  char at_orgl[] = "AT+ORGL\r\n";
  HAL_UART_Transmit_IT(&huart1, (uint8_t*)&at_orgl, strlen(at_orgl));

  // set HC06 module name
  char at_set_name[] = "AT+NAME=NDEProjekt\r\n";
  HAL_UART_Transmit_IT(&huart1, (uint8_t*)&at_set_name, strlen(at_set_name));

  MPU6050_t MPU6050;

  // Initialize MPU6050 (see library sample code)
  while (MPU6050_Init(&hi2c1) == 1){
	  printf("Unable to Init MPU6050\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  init_fan(&(TIM3->CCR1), &(TIM3->CCR2), &(TIM3->CCR3), &(TIM3->CCR4));
  HAL_TIM_Base_Start_IT(&htim2);

  //ch1_counter_val = 0;//(TIM1 -> ARR);
  //ch2_counter_val = 0;

  // calibrate ADC
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(10);
  // end of calibration

  fan_speed_1 = 0;
  fan_speed_2 = 0;
  while (1)
  {

	 HAL_ADC_Start_DMA(&hadc1, (uint16_t *) rawValues, 5);
	 HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // MPU6050 reading
	  MPU6050_Read_MPU(&hi2c1, &MPU6050);
	  Calculate_Accel_Values(&MPU6050);
	  Calculate_Gyro_Values(&MPU6050);
	  Get_Accel_Angles(&MPU6050);
	  Get_Gyro_Angles(&MPU6050, 0.01);
	  Comp_Filter_Results(&MPU6050);

	  // Writing results
	  GyroX = MPU6050.Gyro_X;
	  GyroY = MPU6050.Gyro_Y;
	  GyroZ = MPU6050.Gyro_Z;

	  AccelX = MPU6050.Accel_X;
	  AccelY = MPU6050.Accel_Y;
	  AccelZ = MPU6050.Accel_Z;

	  GyroRoll = MPU6050.Gyro_Roll;
	  GyroPitch = MPU6050.Gyro_Pitch;
	  AccelRoll = MPU6050.Accel_Roll;
	  AccelPitch = MPU6050.Accel_Pitch;


	  if (global_mode == 0){
		  control_rotation(0, GyroX);
	  }else if (global_mode == 1){
		  int16_t fan_speed_local1, fan_speed_local2;
		  light_tracking_logic(rawValues, &fan_speed_local1, &fan_speed_local2);
		  set_fan_speed(fan_speed_local1);
		  set_fan_speed2(fan_speed_local2);
		  char msg[64];
		  snprintf(msg, sizeof(msg), "Comparison of ADC values: %d, %d, %d, %d, %d\r\n", readValue1, readValue2, readValue3, readValue4, readValue5);

		 // Odeslání přes UART
		 HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	  }else if(global_mode == 2){
		  set_fan_speed(800);
		  set_fan_speed2(-800);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1001;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_BLUE_Pin|LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_BLUE_Pin LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin|LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    //__io_putchar(*ptr++);
	  ITM_SendChar(*ptr++);
  }
  return len;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	uint16_t leds[] = {LED_BLUE_Pin, LED_GREEN_Pin, LED_RED_Pin};
	static uint8_t i = 0;
	if (htim == &htim2)
	{
		if (global_mode == 0){
			HAL_GPIO_TogglePin(GPIOC, LED_BLUE_Pin);
			HAL_GPIO_WritePin(GPIOC, LED_GREEN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, LED_RED_Pin, GPIO_PIN_RESET);
		}else if (global_mode == 1){
			HAL_GPIO_WritePin(GPIOC, LED_BLUE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, LED_RED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);

		}else if (global_mode == 2){
			HAL_GPIO_WritePin(GPIOC, LED_BLUE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, LED_GREEN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_TogglePin(GPIOC, LED_RED_Pin);

		}

		/*HAL_GPIO_TogglePin(GPIOC, leds[i]);
		i++;
		if (i>2){
			i=0;
		}*/
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

		/* start the DMA again */
		printf("Size %d\n", Size);
		printf(RxBuffer);
		printf("Received: %s\n", RxBuffer);

		char reply[256];
		snprintf(reply, sizeof(reply), "UART Received text: %s\r\n", RxBuffer);

		HAL_UART_Transmit_IT(&huart1, reply, strlen(reply));
		char* to_send;
		if (strcmp(RxBuffer, "MODE_S\n") == 0){
			global_mode = 0;
			to_send = "setting mode to BASIC mode\r\n";
		}else if(strcmp(RxBuffer, "MODE_L\n") == 0){
			global_mode = 1;
			to_send = "setting mode to light tracking mode\r\n";
		}else if(strcmp(RxBuffer, "MODE_R\n") == 0){
			global_mode = 2;
			to_send = "setting mode to ROTATE mode\r\n";
		}else{
			global_mode = 9;
			to_send = "Unable to decode command: \r\n";
		}

		HAL_UART_Transmit_IT(&huart1, to_send, strlen(to_send));
		//process_command_string(RxBuffer, &global_command, global_args);
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *) RxBuffer, RX_BFR_SIZE);
		//__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

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
