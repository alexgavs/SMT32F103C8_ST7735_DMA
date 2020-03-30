/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735.h"
#include "fonts.h"
#include "testimg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define APP_RX_DATA_SIZE 20
#define APP_TX_DATA_SIZE 64

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
uint16_t SizeBufferRX;
int64_t pcnt;

float bat,bank4,bank3,bank2,bank1;
uint8_t rcvuart;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t r = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void printBuffer(void)
{
int16_t h,w;
char strtxt[50]={};
char strtxt2[50]={};

w = ST7735_GetWidth()-1;
h = ST7735_GetHeight()-1;

//ST7735_SetRotation(0);
//ST7735_FillScreen(ST7735_BLACK);
ST7735_DrawFastHLine(0, 0, w, ST7735_BLUE);
ST7735_DrawFastHLine(0, h, w, ST7735_BLUE);
ST7735_DrawFastVLine(0, 0, h, ST7735_BLUE);
ST7735_DrawFastVLine(w, 0, h, ST7735_BLUE);

//ST7735_DrawLine(0, 0, w, 0, ST7735_WHITE); // верх
//ST7735_DrawLine(0, 0, 0, h, ST7735_WHITE); // лева�?
//ST7735_DrawLine(w, 0, w, h, ST7735_WHITE); // права�?
//ST7735_DrawLine(0, h, w, h, ST7735_WHITE); // низ


//ST7735_DrawLine(ST7735_GetWidth(),ST7735_GetHeight(), 0, ST7735_GetWidth(), ST7735_RED);

//ST7735_DrawLine(0, 0, ST7735_GetWidth(), ST7735_GetHeight(), ST7735_WHITE);
//ST7735_DrawLine(ST7735_GetWidth(), 0, 0, ST7735_GetHeight(), ST7735_WHITE);

//printo("\n 123.45f =", bank4);

sprintf(strtxt,"BUFFER %li",pcnt);

ST7735_DrawString(2, 2, strtxt, Font_7x10, ST7735_RED, ST7735_BLACK);

//sprintf(strtxt2,ByteToHex("0123"));

// sprintf(strtxt,"LEN: %i %s",sizeof(strtxt2), strtxt2);

ST7735_DrawString(2, 12, UserRxBufferFS, Font_7x10, ST7735_WHITE, ST7735_BLACK);

sprintf(strtxt,"RCVSTAT:  %i ",rcvuart);
ST7735_DrawString(2, 32, strtxt, Font_7x10, ST7735_GREEN, ST7735_BLACK);


//if (rcvuart == HAL_OK)
{


	sprintf(strtxt,"BATT:  %.2f ",bat);
ST7735_DrawString(2, 42, strtxt, Font_7x10, ST7735_WHITE, ST7735_BLACK);

sprintf(strtxt,"BANK4: %.2f ",bank4);
ST7735_DrawString(2, 52, strtxt, Font_7x10, ST7735_WHITE, ST7735_BLACK);
//
sprintf(strtxt,"BANK3: %.3f ",bank3);
ST7735_DrawString(2, 62, strtxt, Font_7x10, ST7735_WHITE, ST7735_BLACK);

sprintf(strtxt,"BANK2: %.3f ",bank2);
ST7735_DrawString(2, 72, strtxt, Font_7x10, ST7735_WHITE, ST7735_BLACK);

sprintf(strtxt,"BANK1: %.3f ",bank1);
ST7735_DrawString(2, 82, strtxt, Font_7x10, ST7735_WHITE, ST7735_BLACK);


//if (UserRxBufferFS[0]==90)
{

ST7735_DrawString(2, 92, "|ID| B4 | B3 |", Font_7x10, ST7735_WHITE, ST7735_BLACK);
//printf(strtxt,"|%02x|%02x%02x|%02x%02x|",
//			(uint16_t) UserRxBufferFS[1],
//			(uint16_t) UserRxBufferFS[2],
//			(uint16_t) UserRxBufferFS[3],
//			(uint16_t) UserRxBufferFS[14],
//			(uint16_t) UserRxBufferFS[15]);

sprintf(strtxt,"|%02x|%02x%02x|%02x%02x|",
			(uint16_t) UserRxBufferFS[1],
			(uint16_t) UserRxBufferFS[2],
			(uint16_t) UserRxBufferFS[3],
			(uint16_t) UserRxBufferFS[14],
			(uint16_t) UserRxBufferFS[15]);
ST7735_DrawString(2, 102, strtxt, Font_7x10, ST7735_WHITE, ST7735_BLACK);


ST7735_DrawString(2, 112, "|==| B2 | B1 |", Font_7x10, ST7735_WHITE, ST7735_BLACK);
           sprintf(strtxt,"|==|%02x%02x|%02x%02x|",
			(uint16_t) UserRxBufferFS[16],
			(uint16_t) UserRxBufferFS[17],
			(uint16_t) UserRxBufferFS[18],
			(uint16_t) UserRxBufferFS[19]);
ST7735_DrawString(2, 122, strtxt, Font_7x10, ST7735_WHITE, ST7735_BLACK);

ST7735_DrawString(2, 132, "|==| B0 | cr |", Font_7x10, ST7735_WHITE, ST7735_BLACK);
           sprintf(strtxt,"|==|%02x%02x|%02x%02x|",
			(uint16_t) UserRxBufferFS[12],
			(uint16_t) UserRxBufferFS[13],
			(uint16_t) UserRxBufferFS[6],
			(uint16_t) UserRxBufferFS[7]);
ST7735_DrawString(2, 142, strtxt, Font_7x10, ST7735_WHITE, ST7735_BLACK);


}
}




// ST7735_DrawString(2, 100, strtxt2, Font_7x10, ST7735_WHITE, ST7735_BLACK);

//ST7735_DrawString(0, 3*10, "Font_11x18, green, lorem ipsum", Font_11x18, ST7735_GREEN, ST7735_BLACK);
//ST7735_DrawString(0, 3*10+3*18, "Font_16x26", Font_16x26, ST7735_BLUE, ST7735_BLACK);

}


void calcBattery(void)
{
int8_t shiftAddress=0;
if (UserRxBufferFS[0]==90)
//if (rcvuart)
   {
	  bat  =0.0f +  (((UserRxBufferFS[03+shiftAddress]<<8) + UserRxBufferFS[02+shiftAddress]) / 57.8);
	  bank4=0.0f +  (((UserRxBufferFS[13+shiftAddress]<<8) + UserRxBufferFS[12+shiftAddress]) / 58.4);
	  bank3=0.0f +  (((UserRxBufferFS[15+shiftAddress]<<8) + UserRxBufferFS[14+shiftAddress]) / 58.4);
	  bank2=0.0f +  (((UserRxBufferFS[17+shiftAddress]<<8) + UserRxBufferFS[16+shiftAddress]) / 57.4);
	  bank1=0.0f + (((UserRxBufferFS[19+shiftAddress]<<8) + UserRxBufferFS[18+shiftAddress]) / 58.1);

//	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
   }



}


uint8_t ll=0;

void readUART(void)
{
	pcnt++;
	//	if (huart1.gState==HAL_UART_STATE_READY)
	if (rcvuart>0)   for (uint8_t t=0; t<APP_RX_DATA_SIZE; t++) UserRxBufferFS[t]=0; //clear buffer
    //HAL_UART_Receive_DMA(&huart1, UserRxBufferFS, APP_RX_DATA_SIZE+1);

	//while (!UserRxBufferFS[0]==90)
	{
		rcvuart = HAL_UART_Receive(&huart1, UserRxBufferFS, APP_RX_DATA_SIZE,120);
		//rcvuart = HAL_UART_Receive_IT(&huart1, UserRxBufferFS, APP_RX_DATA_SIZE);
		//rcvuart = HAL_UART_Receive_DMA(&huart1, UserRxBufferFS, APP_RX_DATA_SIZE);
		if (ll>10) { HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);ll=0;}
		printf("UART: %i",ll);
		ll++;
	};



//	if( HAL_UART_Receive_DMA(&huart1, UserRxBufferFS, APP_RX_DATA_SIZE) == HAL_OK )
 //if (HAL_UART_Receive_IT(&huart1, UserRxBufferFS, APP_RX_DATA_SIZE) == HAL_OK )
//	if( HAL_UART_Receive(&huart1, UserRxBufferFS, APP_RX_DATA_SIZE,100) == HAL_OK )
		{
//		rcvuart=1;
		// HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}
//	else
//		rcvuart=0;
		//HAL_UART_Receive_IT(&huart1, UserRxBufferFS, APP_RX_DATA_SIZE+1);
}




void checkButton(void)
{

	if (!HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin))
	{
		sprintf(UserRxBufferFS,"TEST BUFFER %i",pcnt);
		sprintf(UserTxBufferFS,"TEST BUFFER %i",pcnt++);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}

}



void demoTFT(void)
{
 ST7735_SetRotation(r);

 ST7735_FillScreen(ST7735_BLACK);

 for(int x = 0; x < ST7735_GetWidth(); x++)
 {
   ST7735_DrawPixel(x, 0, ST7735_WHITE);
   ST7735_DrawPixel(x, ST7735_GetHeight() - 1, ST7735_WHITE);
 }

 for(int y = 0; y < ST7735_GetHeight(); y++)
 {
   ST7735_DrawPixel(0, y, ST7735_WHITE);
   ST7735_DrawPixel(ST7735_GetWidth() - 1, y, ST7735_WHITE);
 }

 ST7735_DrawLine(0, 0, ST7735_GetWidth(), ST7735_GetHeight(), ST7735_WHITE);
 ST7735_DrawLine(ST7735_GetWidth(), 0, 0, ST7735_GetHeight(), ST7735_WHITE);

 HAL_Delay(2000);

 ST7735_FillScreen(ST7735_BLACK);

 for (int i = 0; i < ST7735_GetHeight(); i += 4)
 {
  ST7735_DrawFastHLine(0, i, ST7735_GetWidth() - 1, ST7735_WHITE);
 }

 for (int i = 0; i < ST7735_GetWidth(); i += 4)
 {
  ST7735_DrawFastVLine(i, 0, ST7735_GetHeight() - 1, ST7735_WHITE);
 }

 HAL_Delay(2000);

 // Check fonts
 ST7735_FillScreen(ST7735_BLACK);
 ST7735_DrawString(0, 0, "Font_7x10, red on black, lorem ipsum dolor sit amet", Font_7x10, ST7735_RED, ST7735_BLACK);
 ST7735_DrawString(0, 3*10, "Font_11x18, green, lorem ipsum", Font_11x18, ST7735_GREEN, ST7735_BLACK);
 ST7735_DrawString(0, 3*10+3*18, "Font_16x26", Font_16x26, ST7735_BLUE, ST7735_BLACK);
 HAL_Delay(2000);

 // Check colors
 ST7735_FillScreen(ST7735_BLACK);
 ST7735_DrawString(0, 0, "BLACK", Font_11x18, ST7735_WHITE, ST7735_BLACK);
 HAL_Delay(500);

 ST7735_FillScreen(ST7735_BLUE);
 ST7735_DrawString(0, 0, "BLUE", Font_11x18, ST7735_BLACK, ST7735_BLUE);
 HAL_Delay(500);

 ST7735_FillScreen(ST7735_RED);
 ST7735_DrawString(0, 0, "RED", Font_11x18, ST7735_BLACK, ST7735_RED);
 HAL_Delay(500);

 ST7735_FillScreen(ST7735_GREEN);
 ST7735_DrawString(0, 0, "GREEN", Font_11x18, ST7735_BLACK, ST7735_GREEN);
 HAL_Delay(500);

 ST7735_FillScreen(ST7735_CYAN);
 ST7735_DrawString(0, 0, "CYAN", Font_11x18, ST7735_BLACK, ST7735_CYAN);
 HAL_Delay(500);

 ST7735_FillScreen(ST7735_MAGENTA);
 ST7735_DrawString(0, 0, "MAGENTA", Font_11x18, ST7735_BLACK, ST7735_MAGENTA);
 HAL_Delay(500);

 ST7735_FillScreen(ST7735_YELLOW);
 ST7735_DrawString(0, 0, "YELLOW", Font_11x18, ST7735_BLACK, ST7735_YELLOW);
 HAL_Delay(500);

 ST7735_FillScreen(ST7735_WHITE);
 ST7735_DrawString(0, 0, "WHITE", Font_11x18, ST7735_BLACK, ST7735_WHITE);
 HAL_Delay(500);

 // Draw circles
 ST7735_FillScreen(ST7735_BLACK);
 for (int i = 0; i < ST7735_GetHeight() / 2; i += 2)
 {
  ST7735_DrawCircle(ST7735_GetWidth() / 2, ST7735_GetHeight() / 2, i, ST7735_YELLOW);
 }
 HAL_Delay(1000);

 ST7735_FillScreen(ST7735_BLACK);
 ST7735_FillTriangle(0, 0, ST7735_GetWidth() / 2, ST7735_GetHeight(), ST7735_GetWidth(), 0, ST7735_RED);
 HAL_Delay(1000);

 ST7735_FillScreen(ST7735_BLACK);
 ST7735_DrawImage(0, 0, 128, 128, (uint16_t*) test_img_128x128);
 HAL_Delay(3000);

 r++;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  ST7735_Init();
  ST7735_Backlight_On();

  ST7735_SetRotation(0);
  ST7735_FillScreen(ST7735_BLACK);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  demoTFT();
//	  readUART();
//	  calcBattery();
//	  printBuffer();
//	  checkButton();
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.Mode = UART_MODE_RX;
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
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ST7735_RES_Pin|ST7735_DC_Pin|ST7735_CS_Pin|ST7735_BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ST7735_RES_Pin ST7735_DC_Pin ST7735_CS_Pin ST7735_BL_Pin */
  GPIO_InitStruct.Pin = ST7735_RES_Pin|ST7735_DC_Pin|ST7735_CS_Pin|ST7735_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
