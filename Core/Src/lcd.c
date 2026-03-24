/*
 * lcd.c
 *
 *  Created on: Jan 23, 2025
 *      Author: pgaur
 */

#include "main.h"
#include "lcd.h"
#include "i2c2_bus.h"
#include "stm32l5xx_hal.h"
#include <string.h>

/* Seeed Grove LCD RGB Backlight — rgb_lcd.cpp / rgb_lcd.h (V4 PCA9633 @ 0x62, V5 @ 0x30). */
#define RGB_ADDR_V4_HAL       (0x62U << 1)
#define RGB_ADDR_V5_HAL       (0x30U << 1)

/* PCA9633 (V4): MODE1, MODE2, LEDOUT then PWM2/3/4 for B/G/R */
#define RGB_V4_REG_MODE1      0x00U
#define RGB_V4_REG_MODE2      0x01U
#define RGB_V4_REG_LEDOUT     0x08U
#define RGB_V4_PWM_BLUE       0x02U
#define RGB_V4_PWM_GREEN      0x03U
#define RGB_V4_PWM_RED        0x04U

static uint8_t rgb_board = 0U; /* 0=unknown, 4=V4, 5=V5 */
static uint16_t rgb_hal_addr;


extern I2C_HandleTypeDef hi2c2;

#define MAX_PRINT_LENGTH 128
static char print_buffer[MAX_PRINT_LENGTH];

uint8_t RGB_R_VALUE;
uint8_t RGB_G_VALUE;
uint8_t RGB_B_VALUE;

char LINE_ONE_TEXT[17];

char LINE_TWO_TEXT[17];

char* getLCDLineOne(void){
	char* cp = LINE_ONE_TEXT;
	return cp;
}

char* getLCDLineTwo(void){
	char* cp = LINE_TWO_TEXT;
	return cp;
}

uint8_t getRGB_R(void){
	return RGB_R_VALUE;
}

uint8_t getRGB_G(void){
	return RGB_G_VALUE;
}

uint8_t getRGB_B(void){
	return RGB_B_VALUE;
}

void LCD_Init(void){

	// wait 15ms
	HAL_Delay(LCD_DELAY_START_WAIT);

	// function set
	if(LCD_SendMessage(LCD_CommandByte, 0x28) == LCD_Result_Success){
		//DebugPrintln("Function Set Successful");
	}


	//display control
	if(LCD_SendMessage(LCD_CommandByte, 0x08) == LCD_Result_Success){
		//DebugPrintln("Display control Successful");
	}


	// display clear
	if(LCD_SendMessage(LCD_CommandByte, 0x01) == LCD_Result_Success){
		//DebugPrintln("Display clear Successful");
	}

	// entry mode set
	if(LCD_SendMessage(LCD_CommandByte, 0x06) == LCD_Result_Success){
		//DebugPrintln("Entry Mode Successful");
	}

	// display ON, cursor OFF, blink OFF
	if(LCD_SendMessage(LCD_CommandByte, 0x0C) == LCD_Result_Success){
		//DebugPrintln("Display ON Successful");
	}

	/*
	//display control
	if(LCD_SendMessage(LCD_CommandByte, 0x0C) == LCD_Result_Success){
		//DebugPrintln("Display control Successful");
	}

	// send sample data: character: D (LHLL LHLL)
	LCD_Print("Hello...");

	if(LCD_SendMessage(LCD_CommandByte, 0xC3) == LCD_Result_Success){
		//DebugPrintln("Display control Successful");
	}
	LCD_Print("D! Here");

	*/
	memset(LINE_ONE_TEXT, ' ', 16);
	memset(LINE_TWO_TEXT, ' ', 16);
	LINE_ONE_TEXT[16] = '\0';
	LINE_TWO_TEXT[16] = '\0';
}

bool LCD_DisplayText(const char* message, uint8_t line){



	bool result = 0;
	/* Keep display enabled before positioning cursor/address. */
	(void)LCD_SendMessage(LCD_CommandByte, 0x0C);
	result = LCD_SendMessage(LCD_CommandByte, line);
	if(result != LCD_Result_Success){
		//DebugPrintln("Display control Successful");

	}

	if (line == FIRST_LINE){
		memset(LINE_ONE_TEXT, ' ', 16);
		if (message != NULL) {
			strncpy(LINE_ONE_TEXT, message, 16);
		}
		LINE_ONE_TEXT[16] = '\0';

		result = LCD_Print(LINE_ONE_TEXT);
		if(result != LCD_Result_Success){
			//DebugPrintln("Display control Successful");
		}
	}else if (line == SECOND_LINE){
		//Set cursor to second line, second character
		memset(LINE_TWO_TEXT, ' ', 16);
		if (message != NULL) {
			strncpy(LINE_TWO_TEXT, message, 16);
		}
		LINE_TWO_TEXT[16] = '\0';
		result = LCD_Print(LINE_TWO_TEXT);
		if(result != LCD_Result_Success){
			//DebugPrintln("Display control Successful");
		}
	}

	return result;
}

bool LCD_Print(const char* message, ...) {
    va_list args;
    va_start(args, message);

    int n = vsnprintf(print_buffer, MAX_PRINT_LENGTH, message, args);
    uint16_t len = 0;
    if (n < 0) {
    	va_end(args);
    	return LCD_Result_Fail;
    }
    len = (n >= MAX_PRINT_LENGTH) ? (MAX_PRINT_LENGTH - 1U) : (uint16_t)n;


    for (uint16_t i = 0; i < len; i++) {
    	if (!LCD_SendMessage(LCD_DataByte, print_buffer[i])) {
			return LCD_Result_Fail;
		}
    }

    va_end(args);

	return LCD_Result_Success;
}


static void RGB_EnsureInited(void)
{
	if (rgb_board != 0U) {
		return;
	}

	/* Prefer V5 (newer boards): probe SGM31323 @ 0x30 */
	if (HAL_I2C_IsDeviceReady(&hi2c2, RGB_ADDR_V5_HAL, 3U, 15U) == HAL_OK) {
		rgb_board = 5U;
		rgb_hal_addr = RGB_ADDR_V5_HAL;
		(void)I2C_SendToSlave(rgb_hal_addr, 0x00U, 0x07U);
		HAL_Delay(1);
		(void)I2C_SendToSlave(rgb_hal_addr, 0x04U, 0x15U);
		return;
	}

	/* V4 PCA9633 @ 0x62 — same sequence as Seeed rgb_lcd::begin() */
	rgb_board = 4U;
	rgb_hal_addr = RGB_ADDR_V4_HAL;
	(void)I2C_SendToSlave(rgb_hal_addr, RGB_V4_REG_MODE1, 0x00U);
	(void)I2C_SendToSlave(rgb_hal_addr, RGB_V4_REG_LEDOUT, 0xFFU);
	(void)I2C_SendToSlave(rgb_hal_addr, RGB_V4_REG_MODE2, 0x20U);
}

void setLCD_RGB(uint8_t r, uint8_t g, uint8_t b)
{
	I2c2Bus_Lock();
	RGB_EnsureInited();

	if (rgb_board == 5U) {
		(void)I2C_SendToSlave(rgb_hal_addr, 0x06U, r);
		(void)I2C_SendToSlave(rgb_hal_addr, 0x07U, g);
		(void)I2C_SendToSlave(rgb_hal_addr, 0x08U, b);
	} else if (rgb_board == 4U) {
		(void)I2C_SendToSlave(rgb_hal_addr, RGB_V4_PWM_RED, r);
		(void)I2C_SendToSlave(rgb_hal_addr, RGB_V4_PWM_GREEN, g);
		(void)I2C_SendToSlave(rgb_hal_addr, RGB_V4_PWM_BLUE, b);
	}

	RGB_R_VALUE = r;
	RGB_G_VALUE = g;
	RGB_B_VALUE = b;
	I2c2Bus_Unlock();
}

I2C_Result I2C_SendToSlave(uint16_t slaveAddress, uint8_t slaveRegister, uint8_t slaveData){
	I2c2Bus_Lock();
	uint8_t commandBuffer[2] = {slaveRegister, slaveData};
	I2C_Result res = LCD_Result_Success;
	if (HAL_I2C_Master_Transmit(&hi2c2, slaveAddress, commandBuffer, 2, 50) != HAL_OK) {
		res = LCD_Result_Fail;
	}
	I2c2Bus_Unlock();
	return res;
}



I2C_Result LCD_SendMessage(LCD_ControlByte ct, uint8_t command){
	I2c2Bus_Lock();
	uint8_t commandBuffer[2] = {0, 0}; // Init with 0

	// create the control word.
	switch (ct){
		case LCD_CommandByte: {
			commandBuffer[0] = LCD_CommandByte;
			commandBuffer[1] = command;
			break;
		}
		case LCD_DataByte: {
			commandBuffer[0] = LCD_DataByte;
			commandBuffer[1] = command;
			break;
		}
		default:{
			commandBuffer[0] = 0x00;
			commandBuffer[1] = 0x00;
		}
	}

	I2C_Result rc = LCD_Result_Success;
	if (HAL_I2C_Master_Transmit(&hi2c2, LCD_ADDRESS, commandBuffer, 2, 50) != HAL_OK) {
		rc = LCD_Result_Fail;
	}
	/* HD44780-compatible controllers need extra time for clear/home commands. */
	if (rc == LCD_Result_Success && ct == LCD_CommandByte && (command == 0x01 || command == 0x02)) {
		HAL_Delay(2);
	}
	I2c2Bus_Unlock();
	return rc;
}



