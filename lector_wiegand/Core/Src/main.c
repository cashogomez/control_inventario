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
 TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t valor1=0, valor2=0, periodo=0,valor3=0, valor4=0, periodo2=0;
uint8_t led1_PV=1, led2_PV=1;
volatile int16_t cuentaBits = 0;
volatile uint32_t tarjetaTempAlto = 0;
volatile uint32_t tarjetaTemp = 0;
volatile uint32_t ultimoWiegand = 0;
int16_t wiegandType = 0;
uint32_t codigo = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
uint32_t GetCardId(volatile uint32_t *codehigh, volatile uint32_t *codelow, uint8_t bitlength);
uint8_t wiegand_disponible();
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //imprimir_serial(cuentaBits);
	  if (wiegand_disponible()) {
		  imprimir_serial(codigo);
	  }
	  //HAL_Delay(500);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void imprimir_serial(uint32_t numero) {
	char buffer[50] ="";
	sprintf( buffer, "\nlectura HEX=0x%lX.DEC= %ld\n\r", numero, numero );
	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim-> Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(led1_PV==1)
		{
			led1_PV=0;
			if (led2_PV==1) {
				LeerD0();
			}
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
		}
		else if (led1_PV==0)
		{
			led1_PV=1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
		}

	}
	else if (htim-> Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if(led2_PV==1)
		{
			led2_PV=0;
			LeerD1();
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_RISING);
		}
		else if (led2_PV==0)
		{
			led2_PV=1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_FALLING);
		}
	}
}
void LeerD0() {
	cuentaBits++;
	if (cuentaBits > 31) {
		tarjetaTempAlto |= ((0x80000000 & tarjetaTemp) >> 31);	//	shift value to high bits
		tarjetaTempAlto <<= 1;
		tarjetaTemp <<= 1;
	}
	else {
		tarjetaTemp <<= 1;		// D0 represent binary 0, so just left shift card data
	}
	ultimoWiegand = HAL_GetTick();	// Keep track of last wiegand bit received
}
void LeerD1() {
	cuentaBits++;
	if (cuentaBits > 31) {
		tarjetaTempAlto |= ((0x80000000 & tarjetaTemp) >> 31);	// shift value to high bits
		tarjetaTempAlto <<= 1;
		tarjetaTemp |= 1;
		tarjetaTemp <<= 1;
	}
	else {
		tarjetaTemp |= 1;			// D1 represent binary 1, so OR card data with 1 then
		tarjetaTemp <<= 1;		// left shift card data
	}
	ultimoWiegand = HAL_GetTick();	// Keep track of last wiegand bit received
}
uint8_t wiegand_disponible() {
	uint32_t tarjetaID;
	uint32_t tiempo_wig = HAL_GetTick();
	if((tiempo_wig - ultimoWiegand) > 50)
	{
		if((cuentaBits == 24) || (cuentaBits == 26) || (cuentaBits == 32) || (cuentaBits == 34))
		{
			tarjetaTemp >>= 1; // shift right 1 bit to get back the real value - interrupt done 1 left shift in advance
			if(cuentaBits > 32) // bit count more than 32 bits, shift high bits right to make adjustment
			{
				tarjetaTempAlto >>= 1;
			}
			tarjetaID = GetCardId(&tarjetaTempAlto, &tarjetaTemp, cuentaBits);
			wiegandType = cuentaBits;
			cuentaBits = 0;
			tarjetaTemp = 0;
			tarjetaTempAlto = 0;
			codigo = tarjetaID;
			return 1;
		}
		else
		{
			// well time over 25 ms and bitCount !=8 , !=26, !=34 , must be noise or nothing then.
			ultimoWiegand = tiempo_wig;
			cuentaBits = 0;
			tarjetaTemp = 0;
			tarjetaTempAlto = 0;
			return 0;
		}
	}
	else {
		return 0;
	}
}
uint32_t GetCardId(volatile uint32_t *codehigh, volatile uint32_t *codelow, uint8_t bitlength)
{

	if(bitlength == 26)								// EM tag
	{
		return (*codelow & 0x1FFFFFE) >> 1;
	}

	if(bitlength == 34)								// Mifare
	{
		*codehigh = *codehigh & 0x03;				// only need the 2 LSB of the codehigh
		*codehigh <<= 30;							// shift 2 LSB to MSB
		*codelow >>= 1;
		return *codehigh | *codelow;
	}

	return *codelow;								// EM tag or Mifare without parity bits
}

/* USER
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
