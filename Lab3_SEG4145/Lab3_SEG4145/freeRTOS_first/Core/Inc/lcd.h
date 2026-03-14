/*
 * i2c_peripheral.h
 *
 *  Created on: Jan 23, 2025
 *      Author: pgaur
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include <stm32l5xx.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>



typedef enum{
	LCD_Result_Fail = 0,
	LCD_Result_Success = 1
} I2C_Result;


// For LCD module
#define LCD_ADDRESS (0x3E << 1)
#define LCD_RGB_ADDR (0x2D << 1)
#define LCD_WAIT 1
#define LCD_DELAY_START_WAIT 15


#define REG_RED 0x01
#define REG_GREEN 0x02
#define REG_BLUE 0x03


typedef enum{
	LCD_CommandByte = 0x00,
	LCD_DataByte = 0x40
} LCD_ControlByte;

typedef enum{
	FIRST_LINE = 0x80,
	SECOND_LINE = 0xC0
} lcdLines;

typedef enum{
	CONTROL = 0,
	DATA = 1
} LCD_CommandType;


// Value getters
uint8_t getRGB_R(void);
uint8_t getRGB_G(void);
uint8_t getRGB_B(void);

char* getLCDLineOne(void);
char* getLCDLineTwo(void);

void LCD_Init(void);
bool LCD_DisplayText(const char* message, uint8_t line);
bool LCD_Print(const char* message, ...);
I2C_Result LCD_SendMessage(LCD_ControlByte ct, uint8_t command);
void setLCD_RGB(uint8_t r, uint8_t g, uint8_t b);
I2C_Result I2C_SendToSlave(uint16_t slaveAddress, uint8_t slaveRegister, uint8_t slaveData);


#endif /* INC_LCD_H_ */
