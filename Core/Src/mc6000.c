/*
 * mc6000.c
 *
 *  Created on: Apr 1, 2020
 *      Author: user
 */

#include "st7735.h"
#include "mc6000.h"


#define APP_RX_DATA_SIZE 20
#define APP_TX_DATA_SIZE 20
#define START_BYTE 		90 // 0x5A

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
uint16_t SizeBufferRX;
int32_t pcnt;
int32_t pcnt2;
uint32_t crcRX,crcTX;

volatile float batf,bank4f,bank3f,bank2f,bank1f;
uint8_t _rxstat;
uint8_t _txstat;
volatile uint16_t border_color; // =ST7735_BLUE;
uint32_t ll;

uint8_t _x=5;
uint8_t _y=5;
uint8_t _c=1;
uint8_t _r=1;
uint8_t _lx;
uint8_t _ly;
char strtxt[18]={};
uint8_t r = 0;




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
sprintf(strtxt,"%s:   %2d.%3d ",(char *) bankname, (uint16_t) b1, (uint16_t) b2);
printbankln(strtxt, font, color, bgcolor);
}
void printbankh(const uint8_t* data, const FontDef font, const uint16_t color, uint16_t bgcolor)
{
uint8_t txt[2]={};
sprintf(txt,"%02x",(uint16_t) data);
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
int fontheight =Font_7x10.height;
int kern=1;
int nl=fontheight+kern;

_c=1;
_r=1;

border_blue(border_color);
printbankd("ERR", (int) pcnt, (int)pcnt2, Font_7x10, ST7735_RED, ST7735_BLACK);_c=1;
sprintf(strtxt,"CRC  : %8X", (uint32_t) HAL_CRC_Calculate(&MC6000_crc_port, UserRxBufferFS, APP_RX_DATA_SIZE));
printbankln((char*) strtxt, Font_7x10, ST7735_GREEN, ST7735_BLACK);

//sprintf(strtxt,"RXerr:            ",HAL_ERR_String(_rxstat));
//printbankln(strtxt,  Font_7x10, ST7735_YELLOW, ST7735_BLACK);
//_r--;
sprintf(strtxt,"RXerr: %8s",HAL_ERR_String(_rxstat));
printbankln((char*)strtxt,  Font_7x10, ST7735_YELLOW, ST7735_BLACK);

//sprintf(strtxt,"TXerr:           ",HAL_ERR_String(_txstat));
//printbankln(strtxt,  Font_7x10, ST7735_YELLOW, ST7735_BLACK);
//_r--;
sprintf(strtxt,"TXerr: %8s",HAL_ERR_String(_txstat));
printbankln((char*)strtxt,  Font_7x10, ST7735_YELLOW, ST7735_BLACK);


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
printbankh((uint8_t)UserRxBufferFS[20], Font_7x10, ST7735_LIGHT_GRAY, ST7735_BLACK);




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
    _rxstat= HAL_UART_Receive_DMA(&MC6000_huart1, UserRxBufferFS, APP_RX_DATA_SIZE);
    if (!_rxstat==0) {pcnt++; clearBuffer();}
}
void writeUART(void)
{
	if (UserRxBufferFS[0]==START_BYTE)
		{
			border_color=ST7735_YELLOW;
			for (uint8_t t=0; t<APP_RX_DATA_SIZE; t++) UserTxBufferFS[t]=65+t; //clear buffer
			UserTxBufferFS[APP_RX_DATA_SIZE-1]='\n';
			_txstat = HAL_UART_Transmit_DMA(&MC6000_huart1, UserRxBufferFS, APP_RX_DATA_SIZE);    blink();
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
void blink(void)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(MC6000_huart1.RxXferSize == APP_RX_DATA_SIZE)
	{
		if(UserRxBufferFS[0] == START_BYTE)	calcBattery();
		else		{_rxstat = 1;/*clearBuffer();*/}
	}
	else
		clearBuffer();

}


