/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <vector>
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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
#define TIM_PERIOD_MS 5
#define UART_RX_PERIOD_MS 10

uint8_t uartRx0[53] { 0 };
uint8_t uartRx1[53] { 0 };
uint8_t *uartRx[2] { &uartRx0[0], &uartRx1[0] };

uint8_t uartRxSaved[53] { 0 };
uint8_t rxArrNum { 0 };

uint8_t rxStarted { 0 };

uint8_t uartRxState[2] { 0 };

uint8_t uartRxFinished { 0 };

uint16_t uartRxSumm { 0 };

uint8_t RxMs { 0 };
uint16_t TxMs { 0 };

uint8_t uartTx[55] = {0x55,0xAA,  // Контрольные байты (0x55,0xAA)
					  50,         // Размер массива
					  0xFF,0xFF,  // Тех. состояние(0-есть плата, 1-нет платы)
					  0,0,  // Порт 1
					  0,0,
					  0,0,  // Порт 2
					  0,0,
					  0,0,  // Порт 3
					  0,0,
					  0,0,  // Порт 4
					  0,0,
					  0,0,  // Порт 5
					  0,0,
					  0,0,  // Порт 6
					  0,0,
					  0,0,  // Порт 7
					  0,0,
					  0,0,  // Порт 8
					  0,0,
					  0,0,  // Порт 9
					  0,0,
					  0,0,  // Порт 10
					  0,0,
					  0,0,  // Порт 11
					  0,0,
					  0,0,  // Порт 12
					  0,0,
					  0,0}; // Контрольная сумма
uint8_t uartTxSaved[55] = { 0 };



uint8_t uartTxState = 0;

uint8_t uartRxFlag = 0;
uint8_t uartTxFlag = 0;

uint16_t uartTxSumm = 0;

//SpiPort spiPort[12];
uint8_t spiState = 0;
uint8_t port = 0;

uint8_t timFlag = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
	uartInterrupt(rxArrNum);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	++RxMs;
	++TxMs;
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	HAL_UART_AbortReceive_IT(&huart2);
	uint32_t dr = USART2->RDR;
	uint32_t er = HAL_UART_GetError(&huart2);
	if (er & HAL_UART_ERROR_PE) {
		__HAL_UART_CLEAR_PEFLAG(&huart2);
	}
	if (er & HAL_UART_ERROR_NE) {
		__HAL_UART_CLEAR_NEFLAG(&huart2);
	}
	if (er & HAL_UART_ERROR_FE) {
		__HAL_UART_CLEAR_FEFLAG(&huart2);
	}
	if (er & HAL_UART_ERROR_ORE) {
		__HAL_UART_CLEAR_OREFLAG(huart);
	}
	if (er & HAL_UART_ERROR_DMA) {
		__HAL_UART_CLEAR_NEFLAG(&huart2);
	}
	//huart->ErrorCode = HAL_UART_ERROR_NONE;
}


void uartInterrupt(uint8_t num) {
		uartRxState[num] = 2;
		if (num) {
			uartRxState[0] = 1;
			HAL_UART_Receive_IT(&huart2, uartRx[0], 53);
		} else {
			uartRxState[1] = 1;
			HAL_UART_Receive_IT(&huart2, uartRx[1], 53);
		}
}


void uartProcessing(uint8_t num) {
	switch (uartRxState[num]) {
	case 1:
		if (huart2.RxXferCount < 0x0035 && huart2.RxXferCount > 0) {
			switch (rxStarted) {
			case 0:
				rxStarted = 1;
				RxMs = 0;
				break;
			default:
				if (RxMs > UART_RX_PERIOD_MS / 5) {
					rxStarted = 0;
					HAL_UART_AbortReceive_IT(&huart2);
					uartRxState[num] = 1;
					HAL_UART_Receive_DMA(&huart2, uartRx[num], 53);
				}
			}
		}
		break;
	case 2:
		rxStarted = 0;
		if (uartRx[num][0] == 0x55 && uartRx[num][1] == 0xAA && uartRx[num][2] == 48) {
			uartRxSumm = 0;
			for (uint8_t i = 0; i <= 50; i++) {
				uartRxSumm += uartRx[num][i];
			}
			if ((uint8_t) uartRxSumm == uartRx[num][51] && (uint8_t) (uartRxSumm >> 8) == uartRx[num][52] && uartRxFinished == 0) {
				uartRxFinished = 1;
				for (uint8_t i = 0; i <= 52; i++) {
					uartRxSaved[i] = uartRx[num][i];
				}
			}
		}
		uartRxState[num] = 0;
		num ? rxArrNum = 0 : rxArrNum = 1;
		break;
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6); //Vklychenie taimera 6
  uartRxState[0] = 1;
  HAL_UART_Receive_IT(&huart2, uartRx0, 53); // Priem po UART
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uartProcessing(0);
	  uartProcessing(1);






	  if (spiState == 0) {
			if (port == 0) {
				spiState = 1;
				spiPort[12].unSelect();

				spiPort[port].setTx(&uartRxSaved[4 * port + 3],
						&uartRxSaved[4 * port + 4], &uartRxSaved[4 * port + 5],
						&uartRxSaved[4 * port + 6]);
				spiPort[port].select();
				HAL_SPI_TransmitReceive_IT(&hspi1, spiPort[port].getTx(),
						spiPort[port].getRx(), 6);
			}

		} else if (spiState == 2) {
			if (port == 0) {
				spiState = 0;
				if (spiPort[port].rxCheck()) {
					++port;
					if (spiPort[port].prevCheck()) {
						uartTxFlag = 3;

						uartTx[4 * port + 5] = *(spiPort[port].getRx());
						uartTx[4 * port + 6] = *(spiPort[port].getRx() + 1);
						uartTx[4 * port + 7] = *(spiPort[port].getRx() + 2);
						uartTx[4 * port + 8] = *(spiPort[port].getRx() + 3);
						*(spiPort[0].getRx() + 4) ?
								uartTx[3] &= ~(1 << port) :
								uartTx[3] |= (1 << port);
					}
				}
			}
		}








	  if (TxMs > 300) {
			uartTxFlag = 1;
		}

		if (uartTxFlag != 0 && uartTxState == 0 && TxMs > 1) {
			uartTxState = 1;
			--uartTxFlag;

			for (int i = 0; i < 55; i++) {
				uartTxSaved[i] = uartTx[i];
			}

			uartTxSumm = 0;
			for (uint8_t i = 0; i < 53; i++) {
				uartTxSumm += uartTxSaved[i];
			}
			uartTxSaved[53] = (uint8_t) uartTxSumm;
			uartTxSaved[54] = (uint8_t) (uartTxSumm >> 8);

			uartTxMs = 0;
			HAL_UART_Transmit_IT(&huart2, (uint8_t*) uartTxSaved, 55);
		}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 23999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : PC5 PC6 PC7 PC8
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12
                           PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
