/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//set color map--------------------------------led
#define MAX_LED 21
#define USE_BRIGHTNESS 1
#define PI 3.14159265

uint16_t pwmData[(24 * MAX_LED) + 50];
uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];

int datasentflag = 0;

//set nextion uart--------------------------------display
char Rxbuf[50];
char testbuf[50];
char testbuf2[50];

void SystemClock_Config(void);

//---------------------get data r g b sensor--------------------
int r = 0, g = 0, b = 0, r1 = 0, g1 = 0, b1 = 0, threshold = 40, status = 0,
		timer_counter = 0;
;

//------------------sensor variables ---------------------------
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;

float Temperature = 0;
float Humidity = 0;
uint8_t Presence = 0;
uint32_t adcvalue;

int time_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//--set uart to printf --------------------------------------------------------
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}

void delay(uint16_t time) {
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim2)) < time)
		;
}

//----------------------------------set led data bit-----------------------------
//send done color set stop dma
void set_led(int LEDnum, int red, int green, int blue) {

	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = green;
	LED_Data[LEDnum][2] = red;
	LED_Data[LEDnum][3] = blue;

}

void Set_Brightness(int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 45)
		brightness = 45;
	for (int i = 0; i < MAX_LED; i++) {
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j = 1; j < 4; j++) {
			float angle = 90 - brightness;  // in degrees
			angle = angle * PI / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j]) / (tan(angle));
		}
	}

#endif

}

//set color setting funcution
void WS2812_Send(void) {
	uint32_t indx = 0;
	uint32_t color;

	for (int i = 0; i < MAX_LED; i++) {
#if USE_BRIGHTNESS
		color =
				((LED_Mod[i][1] << 16) | (LED_Mod[i][2] << 8) | (LED_Mod[i][3]));
#else
		color = ((LED_Data[i][1] << 16) | (LED_Data[i][2] << 8)
				| (LED_Data[i][3]));
#endif

		for (int i = 23; i >= 0; i--) {
			if (color & (1 << i)) {
				pwmData[indx] = 60;  // 2/3 of 90
			}

			else
				pwmData[indx] = 30;  // 1/3 of 90

			indx++;
		}

	}

	for (int i = 0; i < 50; i++) {
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*) pwmData, indx);
	while (!datasentflag) {
	};
	datasentflag = 0;
}

//-----DHT ----function setting-------------------------
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/*********************************** DHT11 FUNCTIONS ********************************************/

#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_1

void DHT11_Start(void) {
	Set_Pin_Output(DHT11_PORT, DHT11_PIN);  // set the pin as output
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	delay(18000);   // wait for 18ms
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
	delay(20);   // wait for 20us
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);    // set as input
}

uint8_t DHT11_Check_Response(void) {
	uint8_t Response = 0;
	delay(40);
	if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
		delay(80);
		if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
			Response = 1;
		else
			Response = -1; // 255
	}
	while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
		;   // wait for the pin to go low

	return Response;
}

uint8_t DHT11_Read(void) {
	uint8_t i, j;
	for (j = 0; j < 8; j++) {
		while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
			;   // wait for the pin to go high
		delay(40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))   // if the pin is low
		{
			i &= ~(1 << (7 - j));   // write 0
		} else
			i |= (1 << (7 - j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
			;  // wait for the pin to go low
	}
	return i;
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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	for (int i = 0; i < MAX_LED; i++) {
		Set_Brightness(150);
		set_led(i, 0, 0, 0);
		WS2812_Send();
		HAL_Delay(100);
	}
	HAL_Delay(1000);

	for (int i = 12; i < MAX_LED; i++) {
		Set_Brightness(255);
		set_led(i, 240, 20, 110);
		WS2812_Send();
		HAL_Delay(10);
	}

	color_plant();

	HAL_Delay(1000);
	printf("test start \r\n");
	HAL_UART_Receive_IT(&huart1, (uint8_t*) Rxbuf, 3);
	HAL_ADC_Start_IT(&hadc1);		//ADC
	HAL_TIM_Base_Start(&htim2);  //timer
	HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		color_swap(r, g, b);
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 71;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(relay_GPIO_Port, relay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : relay_Pin */
  GPIO_InitStruct.Pin = relay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(relay_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

//---------------------------neopixel function-------------------------------

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
	datasentflag = 1;
}

void color_swap(int r, int g, int b) {
	if (r != r1 || g != g1 || b != b1) {
		r1 = r;
		g1 = g;
		b1 = b;
		for (int i = 0; i < 12; i++) {
			Set_Brightness(150);
			set_led(i, r, g, b);
			WS2812_Send();
			HAL_Delay(10);
		}
		//color_plant();
	}

}

//plant led on
void color_plant() {

	for (int i = 12; i < MAX_LED; i++) {
		Set_Brightness(255);
		set_led(i, 240, 20, 110);
		WS2812_Send();
		HAL_Delay(10);
	}
}

//-------------------------------nextion function----------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {  //344

	HAL_UART_Receive_IT(&huart1, (uint8_t*) Rxbuf, 4);
//	HAL_UART_Transmit(&huart2, (uint8_t*) Rxbuf, 4, 0xFFFF);

	for (int i = 0; i < 3; i++) {	//check_date
		printf("%d : %c \n\r", i, (char*) Rxbuf[i]);
	}

	if (Rxbuf[1] == 'R') {
		sprintf(testbuf2, "%d", (int*) Rxbuf[3]);
		printf("3 : %d \n\r", (int*) Rxbuf[3]);
		r = (int*) Rxbuf[3];
	}
	if (Rxbuf[1] == 'G') {
		sprintf(testbuf2, "%d", (int*) Rxbuf[3]);
		printf("3 : %d \n\r", (int*) Rxbuf[3]);
		g = (int*) Rxbuf[3];
	}
	if (Rxbuf[1] == 'B') {
		sprintf(testbuf2, "%d", (int*) Rxbuf[3]);
		printf("3 : %d \n\r", (int*) Rxbuf[3]);
		b = (int*) Rxbuf[3];
	}

	if (Rxbuf[1] == 'W') {
		sprintf(testbuf2, "%d", (int*) Rxbuf[3]);
		printf("3 : %d \n\r", (int*) Rxbuf[3]);
		threshold = (int*) Rxbuf[3];
	}

	if (Rxbuf[0] == 'e' || Rxbuf[1] == 'e') {
		if (status == 0) {
			status = 1;
		} else {
			status = 0;
		}
	}

	printf("\n\r");
	memset(Rxbuf, 0, sizeof(Rxbuf));

}

void Nextion_Send(char *Id, char *String) {
	uint8_t Cmd_End[3] = { 0xFF, 0xFF, 0xFF };
	char buf[50];
	int len = sprintf(buf, "%s.txt=\"%s\"", Id, String);
	HAL_UART_Transmit(&huart1, (uint8_t*) buf, len, 1000);
	HAL_UART_Transmit(&huart1, Cmd_End, sizeof(Cmd_End), 1000);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//-------get adc it -----------------------------------------------
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adcvalue = (uint32_t) HAL_ADC_GetValue(&hadc1); 		//ADC
	adcvalue = map(adcvalue, 0, 4096, 0, 100);
	if (adcvalue > threshold) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);		//off
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);		//on
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	printf("start_timer_intrrupt\r\n"); //2sec
	time_counter++;

	if (time_counter == 2) { // 2 = 2sec
		printf("start_timer_intrrupt\r\n");

//		adcvalue = (uint32_t) HAL_ADC_GetValue(&hadc1); 		//ADC

		DHT11_Start();
		Presence = DHT11_Check_Response();
		Rh_byte1 = DHT11_Read();
		Rh_byte2 = DHT11_Read();
		Temp_byte1 = DHT11_Read();
		Temp_byte2 = DHT11_Read();
		SUM = DHT11_Read();

		TEMP = Temp_byte1;
		RH = Rh_byte1;

		Temperature = (float) TEMP;
		Humidity = (float) RH;
		time_counter = 0;

		printf("Temperature : %.0f Humidity : %.0f\r\n", Temperature, Humidity);

		printf("adc : %d \r\n", adcvalue);

		printf("r : %d, g : %d, b : %d threshold : %d, status : %d \r\n", r, g,
				b, threshold, status);

		if (status == 0) {
			char test_buffer[30];
			sprintf(test_buffer, "%d %%", adcvalue);
			Nextion_Send("t5", test_buffer);

			sprintf(test_buffer, "%.1f %% ", Humidity);
			Nextion_Send("t4", test_buffer);

			sprintf(test_buffer, "%.1f C", Temperature);
			Nextion_Send("t3", test_buffer);
		}


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
