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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735.h"
#include "fonts.h"
#include "testimg.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define APP_RX_DATA_SIZE 20
#define APP_TX_DATA_SIZE 20
#define START_BYTE 		90 // 0x5A

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
uint16_t SizeBufferRX;
int32_t pcnt,pcnt2=0;

uint32_t crcRX,crcTX;

//int bat,bank4,bank3,bank2,bank1;
volatile float batf,bank4f,bank3f,bank2f,bank1f;
uint8_t _rxstat;
uint8_t _txstat;
volatile uint16_t border_color=ST7735_BLUE;
uint32_t ll;

uint8_t _x=5;
uint8_t _y=5;
uint8_t _c=1;
uint8_t _r=1;
uint8_t _lx;
uint8_t _ly;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t r = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
osThreadId myTaskReadUARTHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

/* USER CODE BEGIN PFP */

void border_blue(uint16_t color );
//void printbanks(char* bankname, char* txt, FontDef font, uint16_t color, uint16_t bgcolor);
//void printbankf(char* bankname, float voltage, FontDef font, uint16_t color, uint16_t bgcolor);
//void printBuffer(void);
void calcBattery(void);
void readUART(void);
void clearBuffer(void);
void blink(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void border_blue(uint16_t color )
{
	uint16_t h,w;
	w = ST7735_GetWidth()-1;
	h = ST7735_GetHeight()-1;

	ST7735_DrawFastVLine(0, 0, h, color); //left
	ST7735_DrawFastVLine(w, 0, h, color); //right
//
////	ST7735_DrawLine(0, 0, 0, h, color);
//
	ST7735_DrawFastHLine(0, 0, w, color); // top
	ST7735_DrawFastHLine(0, h, w, color); //bottom

	HAL_Delay(10);

}




char strtxt[18]={};

void printbank(char* bankname, FontDef font, uint16_t color,  uint16_t bgcolor, uint8_t size)
{
uint8_t xx;
uint8_t yy;

xx=_c*font.width-font.width;
yy= _r*font.height-font.height;

_lx=xx+_x;
_ly=yy+_y;

//ST7735_FillRectangle(xx+_x+size*font.width, yy+_y, ST7735_GetWidth()-1-xx+_x, font.height, bgcolor);
ST7735_DrawString(_lx, _ly, bankname, font, color, bgcolor);
_c+=size;
}

void printbankln(char* bankname, FontDef font, uint16_t color, const uint16_t bgcolor)
{
printbank(bankname, font, color, bgcolor,1);
_r++;
_c=1;
}


void printbanks(char* bankname2, char* txt, FontDef font, uint16_t color, uint16_t bgcolor)
{
sprintf((char*) strtxt,"%s: %s    ",(char*) bankname2, txt);
printbankln((char*)strtxt, font, color, bgcolor);
}


void printbankd(char* bankname, const uint16_t t1, uint16_t t2, FontDef font, uint16_t color, uint16_t bgcolor)
{
sprintf((char*) strtxt,"%s:%d  :%d",(char*)bankname, t1,t2);
printbankln((char*)strtxt, font, color, bgcolor);
}


void printbankf(char* bankname, const float voltage, FontDef font, uint16_t color, uint16_t bgcolor)
{
uint16_t b1=(uint16_t) voltage;
uint16_t b2=(uint16_t) ((float)(voltage-b1)*1000);
sprintf(strtxt,"%s:   %2d.%3d ",bankname, b1, b2);
printbankln(strtxt, font, color, bgcolor);
}


void printbankh(const uint8_t* data, const FontDef font, const uint16_t color, uint16_t bgcolor)
{
uint8_t txt[2]={};
sprintf(txt,"%02x",data);
printbank(txt, font, color, bgcolor,2);
}

char* HAL_ERR_String(uint16_t error)
{
char* state;
	switch (error) {
	case HAL_UART_STATE_RESET:  /*!< Peripheral is not yet Initialized Value is allowed for gState and RxState */
		state="OK";				break;
	case  HAL_ERROR:
		state="ERROR";			break;
	case  HAL_BUSY:
		state="HAL BUSY";		break;
	case HAL_TIMEOUT:
		state="HAL TIMEOUT";	break;
	case HAL_UART_STATE_READY:    /*!< Peripheral Initialized and ready for use Value is allowed for gState and RxState */
		state="UART READY";     break;
	case HAL_UART_STATE_BUSY:      /*!< an internal process is ongoing Value is allowed for gState only */
		state="UART BUSY";      break;
	case HAL_UART_STATE_BUSY_TX:   /*!< Data Transmission process is ongoing Value is allowed for gState only */
		state="UART BUSY_TX";   break;
	case HAL_UART_STATE_BUSY_RX:   /*!< Data Reception process is ongoing Value is allowed for RxState only */
		 state="UART BUSY_RX";  break;
	case HAL_UART_STATE_BUSY_TX_RX:  /*!< Data Transmission and Reception process is ongoing  Not to be used for neither gState nor RxState.Value is result of combination (Or) between gState and RxState values */
		 state="UART BUSY_TX_RX";break;
	case HAL_UART_STATE_TIMEOUT:     /*!< Timeout state  Value is allowed for gState only */
		 state="UART TIMEOUT";  break;
	case HAL_UART_STATE_ERROR:       /*!< Error  Value is allowed for gState only */
		 state="UART ERROR";    break;
	default:
		state = "PASS";	break;}

return state;
}



void printBuffer(void)
{
//
int16_t x;
int fontheight =Font_7x10.height;
int kern=1;
int nl=fontheight+kern;
x=1;



_c=1;
_r=1;

border_blue(border_color);
printbankd("ERR", (int) pcnt, (int)pcnt2, Font_7x10, ST7735_RED, ST7735_BLACK);_c=1;
sprintf(strtxt,"CRC  : %8X", HAL_CRC_Calculate(&hcrc, UserRxBufferFS, APP_RX_DATA_SIZE));
printbankln(strtxt, Font_7x10, ST7735_GREEN, ST7735_BLACK);

//sprintf(strtxt,"RXerr:            ",HAL_ERR_String(_rxstat));
//printbankln(strtxt,  Font_7x10, ST7735_YELLOW, ST7735_BLACK);
//_r--;
sprintf(strtxt,"RXerr: %8s",HAL_ERR_String(_rxstat));
printbankln(strtxt,  Font_7x10, ST7735_YELLOW, ST7735_BLACK);

//sprintf(strtxt,"TXerr:           ",HAL_ERR_String(_txstat));
//printbankln(strtxt,  Font_7x10, ST7735_YELLOW, ST7735_BLACK);
//_r--;
sprintf(strtxt,"TXerr: %8s",HAL_ERR_String(_txstat));
printbankln(strtxt,  Font_7x10, ST7735_YELLOW, ST7735_BLACK);


if (UserRxBufferFS[0]==START_BYTE)
{

printbankf("VBAT ",  batf,   Font_7x10, ST7735_GREEN, ST7735_BLACK);
printbankf("BANK1", bank1f, Font_7x10, ST7735_WHITE, ST7735_BLACK);
printbankf("BANK2", bank2f, Font_7x10, ST7735_WHITE, ST7735_BLACK);
printbankf("BANK3", bank3f, Font_7x10, ST7735_WHITE, ST7735_BLACK);
printbankf("BANK4", bank4f, Font_7x10, ST7735_WHITE, ST7735_BLACK);
_r++;


//ST7735_FillRectangle(xx+_x+size*font.width, yy+_y, ST7735_GetWidth()-1-xx+_x, font.height, bgcolor);
ST7735_DrawFastHLine(_lx-_x, _ly+7+_y, ST7735_GetWidth()-1, border_color);

printbankln("0102030405060708", Font_7x10, ST7735_WHITE, ST7735_BLACK);
printbankh(UserRxBufferFS[0], Font_7x10, ST7735_RED, ST7735_BLACK); // START-BYTE

printbankh(UserRxBufferFS[1], Font_7x10, ST7735_BLUE, ST7735_BLACK); //TOTAL VOLTAGE
printbankh(UserRxBufferFS[2], Font_7x10, ST7735_BLUE, ST7735_BLACK);

printbankh(UserRxBufferFS[3], Font_7x10, ST7735_DARK_GRAY, ST7735_BLACK); //??
printbankh(UserRxBufferFS[4], Font_7x10, ST7735_DARK_GRAY, ST7735_BLACK);

printbankh(UserRxBufferFS[5], Font_7x10, ST7735_DARK_GRAY, ST7735_BLACK); // ??
printbankh(UserRxBufferFS[6], Font_7x10, ST7735_DARK_GRAY, ST7735_BLACK);
_c=1; _r++;
printbankln("0910111213141516", Font_7x10, ST7735_WHITE, ST7735_BLACK);
printbankh(UserRxBufferFS[7], Font_7x10, ST7735_DARK_GRAY, ST7735_BLACK); //??
printbankh(UserRxBufferFS[8], Font_7x10, ST7735_DARK_GRAY, ST7735_BLACK);

printbankh(UserRxBufferFS[ 9], Font_7x10, ST7735_DARK_GRAY, ST7735_BLACK); //??
printbankh(UserRxBufferFS[10], Font_7x10, ST7735_DARK_GRAY, ST7735_BLACK);

printbankh(UserRxBufferFS[11], Font_7x10, ST7735_ORANGE, ST7735_BLACK); //bank4
printbankh(UserRxBufferFS[12], Font_7x10, ST7735_ORANGE, ST7735_BLACK);

printbankh(UserRxBufferFS[13], Font_7x10, ST7735_PINK, ST7735_BLACK); //bank3
printbankh(UserRxBufferFS[14], Font_7x10, ST7735_PINK, ST7735_BLACK);
_c=1; _r++;
printbankh(UserRxBufferFS[15], Font_7x10, ST7735_MAGENTA, ST7735_BLACK); //bank2
printbankh(UserRxBufferFS[16], Font_7x10, ST7735_MAGENTA, ST7735_BLACK);

printbankh(UserRxBufferFS[17], Font_7x10, ST7735_GREEN, ST7735_BLACK); //bank1
printbankh(UserRxBufferFS[18], Font_7x10, ST7735_GREEN, ST7735_BLACK);

printbankh(UserRxBufferFS[19], Font_7x10, ST7735_LIGHT_GRAY, ST7735_BLACK); //end
printbankh(UserRxBufferFS[20], Font_7x10, ST7735_LIGHT_GRAY, ST7735_BLACK);




}
}



void calcBattery(void)
{
int8_t shiftAddress=-1;
   {
	  batf  = (((UserRxBufferFS[03+shiftAddress]<<8) | UserRxBufferFS[02+shiftAddress]) / 57.8);
	  bank4f= (((UserRxBufferFS[13+shiftAddress]<<8) | UserRxBufferFS[12+shiftAddress]) / 58.4);
	  bank3f= (((UserRxBufferFS[15+shiftAddress]<<8) | UserRxBufferFS[14+shiftAddress]) / 58.4);
	  bank2f= (((UserRxBufferFS[17+shiftAddress]<<8) | UserRxBufferFS[16+shiftAddress]) / 57.4);
	  bank1f= (((UserRxBufferFS[19+shiftAddress]<<8) | UserRxBufferFS[18+shiftAddress]) / 58.1);
   }


}



  void readUART(void)
{
	border_color=ST7735_BLUE;
    _rxstat= HAL_UART_Receive_DMA(&huart1, UserRxBufferFS, APP_RX_DATA_SIZE);
    if (!_rxstat==0) {pcnt++; clearBuffer();}
}


void writeUART(void)
{
	if (UserRxBufferFS[0]=START_BYTE)
		{
			border_color=ST7735_YELLOW;
			for (uint8_t t=0; t<APP_RX_DATA_SIZE; t++) UserTxBufferFS[t]=65+t; //clear buffer
			UserTxBufferFS[APP_RX_DATA_SIZE-1]='\n';
			_txstat = HAL_UART_Transmit_DMA(&huart1, UserRxBufferFS, APP_RX_DATA_SIZE);    blink();
				if (!_txstat==0) pcnt2++;
 	 		CDC_Transmit_FS(UserRxBufferFS, APP_RX_DATA_SIZE);
		}
}


void clearBuffer(void)
{
	for (uint8_t t=0; t<=APP_RX_DATA_SIZE; t++) UserRxBufferFS[t]=0; //clear buffer
	border_color=ST7735_RED;
	//ST7735_InvertColors(1);

}

void checkButton(void)
{
	if (!HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)){for (int t=1; t<APP_RX_DATA_SIZE;t++) UserRxBufferFS[t]=t;	blink();pcnt2++; }
}


void blink(void){HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(huart1.RxXferSize == APP_RX_DATA_SIZE)
	{
		if(UserRxBufferFS[0] == START_BYTE)	calcBattery();
		else		{_rxstat = 1;/*clearBuffer();*/}
	}
	else
		clearBuffer();

}

// #include "demo.c"


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
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  ST7735_Init();
  ST7735_Backlight_On();

  ST7735_SetRotation(0);
  ST7735_FillScreen(ST7735_BLACK);


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTaskReadUART */
  osThreadDef(myTaskReadUART, StartTask02, osPriorityIdle, 0, 256);
  myTaskReadUARTHandle = osThreadCreate(osThread(myTaskReadUART), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  demoTFT();
	  readUART();
	  printBuffer();
	  writeUART();
	  checkButton();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  readUART();
	  writeUART();
	  printBuffer();
	  checkButton();
     osDelay(50);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTaskReadUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {  blink();
     osDelay(10);
  }
  /* USER CODE END StartTask02 */
}

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
