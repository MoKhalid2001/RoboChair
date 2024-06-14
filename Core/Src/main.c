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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RIGHT_MOTOR_POS GPIO_PIN_12
#define RIGHT_MOTOR_NEG GPIO_PIN_13
#define LEFT_MOTOR_POS GPIO_PIN_14
#define LEFT_MOTOR_NEG GPIO_PIN_15
#define MODE_PIN GPIO_PIN_14

#define MANUAL_MODE 1
#define FACE_MODE 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch2;
DMA_HandleTypeDef hdma_tim3_ch3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// JOYSTICK VARIABLES :
uint32_t xMapped = 0;
int32_t Joystick[2];

// FACE VARIABLES :
volatile uint8_t Rx_byte_USART1;
volatile uint8_t Rx_buffer_USART1[10];
volatile uint8_t Rx_idx = 0;
int32_t FACE_SPEED = 300;
uint8_t ISFORWARD = 0;
uint8_t ISTOGGELTED = 1;

// GENERAL VARIABLES :
uint8_t SELECTED_MODE = 2;
int32_t CCR1_VALUE = 0;
int32_t CCR2_VALUE = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void MoveForward();
void TurnLeft();
void TurnRight();
void Stop();
int32_t MAP(int32_t au32_IN, int32_t au32_INmin, int32_t au32_INmax, int32_t au32_OUTmin, int32_t au32_OUTmax);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // UNUSED(huart);
	 if (huart->Instance == USART1) {
		 Rx_buffer_USART1[( Rx_idx++ % 10)] = Rx_byte_USART1;
		 HAL_UART_Receive_IT(&huart1, &Rx_byte_USART1, 1);
	      }
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  HAL_UART_Receive_IT(&huart1, &Rx_byte_USART1, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (SELECTED_MODE == MANUAL_MODE){
		  HAL_ADC_Start_DMA(&hadc1, Joystick, 2);

		  // Y-AXIS used for forward and backward control
		  if (Joystick[0] >= 2100){ // Move Forward
			  MoveForward();
			  CCR1_VALUE = MAP(Joystick[0], 2100, 4096, 0, 400);
			  CCR2_VALUE = MAP(Joystick[0], 2100, 4096, 0, 400);
		  }
		  else if (Joystick[0] <= 1900){ // Move Backword
			  MoveBackword();
			  CCR1_VALUE = MAP(Joystick[0], 1900, 0, 0, 400);
			  CCR2_VALUE = MAP(Joystick[0], 1900, 0, 0, 400);
		  }
		  else Stop();

		  // X-AXIS used for forward and backward control
		  if (Joystick[1] < 1900){ // Turn Left
			  xMapped = MAP(Joystick[1],1900, 0, 0, 400);

			  CCR1_VALUE += 2 * xMapped; // Increase right motor
			  CCR2_VALUE -= xMapped; // Decrease left motor

			  if (CCR1_VALUE > 400) CCR1_VALUE = 400;
			  if (CCR2_VALUE < 0) CCR2_VALUE = 0;
			  TurnLeft();
		  }
		  else if (Joystick[1] > 2150){ // Turn Right
			  xMapped = MAP(Joystick[1],2150, 4096, 0, 400);
			  CCR1_VALUE -= 2 * xMapped; // Decrease right motor
			  CCR2_VALUE += xMapped; // Increase left motor

			  if (CCR1_VALUE < 0) CCR1_VALUE = 0;
			  if (CCR2_VALUE > 400) CCR2_VALUE = 400;
			  TurnRight();
		  }
	  }
	  else if (SELECTED_MODE == FACE_MODE){
		  if (Rx_byte_USART1 == 'F'){
				CCR1_VALUE = FACE_SPEED;
				CCR2_VALUE = FACE_SPEED;
				MoveForward();
				ISFORWARD = 1;
			}
			else if (Rx_byte_USART1 == 'R'){
				CCR1_VALUE = FACE_SPEED;
				CCR2_VALUE = FACE_SPEED;
				if (ISFORWARD) {CCR1_VALUE -= 50; CCR2_VALUE +=50; MoveForward();}
				else TurnRight();

			}
			else if (Rx_byte_USART1 == 'L'){
				CCR1_VALUE = FACE_SPEED;
				CCR2_VALUE = FACE_SPEED;
				if (ISFORWARD) {CCR1_VALUE += 50; CCR2_VALUE -=50; MoveForward();}
				else TurnLeft();

			}
			else if (Rx_byte_USART1 == 'C'){
				if (!ISFORWARD) Stop();
				else if (ISFORWARD) {MoveForward(); CCR1_VALUE = FACE_SPEED; CCR2_VALUE = FACE_SPEED;}
			}
			else if (Rx_byte_USART1 == 'T'){
				if (ISTOGGELTED) ISTOGGELTED = 0;
				else ISTOGGELTED = 1;
				Rx_buffer_USART1[++Rx_idx] = 'C'; Rx_byte_USART1 = 'C';
				ISFORWARD = 0;
				Stop();
			}
			else if (Rx_byte_USART1 == 'S' || Rx_byte_USART1 == 'N'){
				Stop();
				ISFORWARD = 0;
			}

	  }
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,CCR1_VALUE);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,CCR2_VALUE);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim3.Init.Prescaler = 2-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 400-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RIGHT_MOTOR_POS_Pin|RIGHT_MOTOR_NEG_Pin|LEFT_MOTOR_POS_Pin|LEFT_MOTOR_NEG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RIGHT_MOTOR_POS_Pin RIGHT_MOTOR_NEG_Pin LEFT_MOTOR_POS_Pin LEFT_MOTOR_NEG_Pin */
  GPIO_InitStruct.Pin = RIGHT_MOTOR_POS_Pin|RIGHT_MOTOR_NEG_Pin|LEFT_MOTOR_POS_Pin|LEFT_MOTOR_NEG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int32_t MAP(int32_t au32_IN, int32_t au32_INmin, int32_t au32_INmax, int32_t au32_OUTmin, int32_t au32_OUTmax)
{
    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}
void Stop(){
	CCR1_VALUE = 0;
	CCR2_VALUE = 0;

	 HAL_GPIO_WritePin(GPIOB, RIGHT_MOTOR_POS, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOB, RIGHT_MOTOR_NEG, GPIO_PIN_RESET);

	 HAL_GPIO_WritePin(GPIOB, LEFT_MOTOR_POS, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOB, LEFT_MOTOR_NEG, GPIO_PIN_RESET);
}

void MoveForward(){
	 HAL_GPIO_WritePin(GPIOB, RIGHT_MOTOR_POS, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOB, RIGHT_MOTOR_NEG, GPIO_PIN_RESET);

	 HAL_GPIO_WritePin(GPIOB, LEFT_MOTOR_POS, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOB, LEFT_MOTOR_NEG, GPIO_PIN_RESET);

 }

void MoveBackword(){
	 HAL_GPIO_WritePin(GPIOB, RIGHT_MOTOR_POS, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOB, RIGHT_MOTOR_NEG, GPIO_PIN_SET);

	 HAL_GPIO_WritePin(GPIOB, LEFT_MOTOR_POS, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOB, LEFT_MOTOR_NEG, GPIO_PIN_SET);

 }

void TurnRight(){
	 HAL_GPIO_WritePin(GPIOB, RIGHT_MOTOR_POS, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOB, RIGHT_MOTOR_NEG, GPIO_PIN_RESET);

	 HAL_GPIO_WritePin(GPIOB, LEFT_MOTOR_POS, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOB, LEFT_MOTOR_NEG, GPIO_PIN_RESET);

 }

void TurnLeft(){
	 HAL_GPIO_WritePin(GPIOB, RIGHT_MOTOR_POS, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOB, RIGHT_MOTOR_NEG, GPIO_PIN_RESET);

	 HAL_GPIO_WritePin(GPIOB, LEFT_MOTOR_POS, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOB, LEFT_MOTOR_NEG, GPIO_PIN_RESET);

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
