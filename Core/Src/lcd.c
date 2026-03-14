/*
 * lcd.c
 *
 *  Created on: Jan 23, 2025
 *      Author: pgaur
 */

#include "main.h"
#include "lcd.h"
#include <string.h>


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


void setLCD_RGB(uint8_t r, uint8_t g, uint8_t b){


	I2C_SendToSlave(LCD_RGB_ADDR, REG_RED, r);
	I2C_SendToSlave(LCD_RGB_ADDR, REG_GREEN, g);
	I2C_SendToSlave(LCD_RGB_ADDR, REG_BLUE, b);
	return;
}

I2C_Result I2C_SendToSlave(uint16_t slaveAddress, uint8_t slaveRegister, uint8_t slaveData){

	uint8_t commandBuffer[2] = {slaveRegister, slaveData};
	if(HAL_I2C_Master_Transmit(&hi2c2, slaveAddress, commandBuffer, 2, 1) != HAL_OK){
		return LCD_Result_Fail;
	}

	// DebugPrintln("I2C_Debug: ADDR: %x \t REG: %x \t DATA: %x", slaveAddress, commandBuffer[0], commandBuffer[1]);
	return LCD_Result_Success;
}



I2C_Result LCD_SendMessage(LCD_ControlByte ct, uint8_t command){

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

	// Send it!

	if(HAL_I2C_Master_Transmit(&hi2c2, LCD_ADDRESS, commandBuffer, 2, 15) != HAL_OK){
		return LCD_Result_Fail;
	}
	/* HD44780-compatible controllers need extra time for clear/home commands. */
	if (ct == LCD_CommandByte && (command == 0x01 || command == 0x02)) {
		HAL_Delay(2);
	}
	return LCD_Result_Success;

}



