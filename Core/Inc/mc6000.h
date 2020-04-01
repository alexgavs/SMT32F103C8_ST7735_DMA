/*
 * mc6000.h
 *
 *  Created on: Apr 1, 2020
 *      Author: user
 */

#ifndef SRC_MC6000_H_
#define SRC_MC6000_H_

#include "st7735.h"
#include "main.h"

#define MC6000_crc_port 	hcrc	//hspi1, hspi2, hspi3...
#define MC6000_huart1		huart1

extern UART_HandleTypeDef MC6000_huart1;
extern CRC_HandleTypeDef MC6000_crc_port;



void border_blue(uint16_t color );
void printbank(char* bankname, FontDef font, uint16_t color,  uint16_t bgcolor, uint8_t size);
void printbankln(char* bankname, FontDef font, uint16_t color, const uint16_t bgcolor);
void printbanks(char* bankname2, char* txt, FontDef font, uint16_t color, uint16_t bgcolor);
void printbankd(char* bankname, const uint16_t t1, uint16_t t2, FontDef font, uint16_t color, uint16_t bgcolor);
void printbankf(char* bankname, const float voltage, FontDef font, uint16_t color, uint16_t bgcolor);
void printbankh(const uint8_t* data, const FontDef font, const uint16_t color, uint16_t bgcolor);
char* HAL_ERR_String(uint16_t error);
void printBuffer(void);
void calcBattery(void);
void readUART(void);
void writeUART(void);
void clearBuffer(void);
void checkButton(void);
void blink(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);

void calcBattery(void);
void readUART(void);
void clearBuffer(void);
void blink(void);

// volatile uint16_t border_color=ST7735_BLUE;

#endif /* SRC_MC6000_H_ */
