/*
 * LCD_GFX.c
 *
 * Created: 9/20/2021 6:54:25 PM
 *  Author: You
 */ 

#include "LCD_GFX.h"
#include "ST7735.h"
#include <math.h>

/******************************************************************************
* Local Functions
******************************************************************************/



/******************************************************************************
* Global Functions
******************************************************************************/

/**************************************************************************//**
* @fn			uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
* @brief		Convert RGB888 value to RGB565 16-bit color data
* @note
*****************************************************************************/
uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
	return ((((31*(red+4))/255)<<11) | (((63*(green+2))/255)<<5) | ((31*(blue+4))/255));
}

/**************************************************************************//**
* @fn			void LCD_drawPixel(uint8_t x, uint8_t y, uint16_t color)
* @brief		Draw a single pixel of 16-bit rgb565 color to the x & y coordinate
* @note
*****************************************************************************/
void LCD_drawPixel(uint8_t x, uint8_t y, uint16_t color) {
	LCD_setAddr(x,y,x,y);
	SPI_ControllerTx_16bit(color);
}

/**************************************************************************//**
* @fn			void LCD_drawChar(uint8_t x, uint8_t y, uint16_t character, uint16_t fColor, uint16_t bColor)
* @brief		Draw a character starting at the point with foreground and background colors
* @note
*****************************************************************************/
void LCD_drawChar(uint8_t x, uint8_t y, uint16_t character, uint16_t fColor, uint16_t bColor){
	uint16_t row = character - 0x20;		//Determine row of ASCII table starting at space
	int i, j;
	if ((LCD_WIDTH-x>7)&&(LCD_HEIGHT-y>7)){
		for(i=0;i<5;i++){
			uint8_t pixels = ASCII[row][i]; //Go through the list of pixels
			for(j=0;j<8;j++){
				if ((pixels>>j)&1==1){
					LCD_drawPixel(x+i,y+j,fColor);
				}
				else {
					LCD_drawPixel(x+i,y+j,bColor);
				}
			}
		}
	}
}


/******************************************************************************
* LAB 4 TO DO. COMPLETE THE FUNCTIONS BELOW.
* You are free to create and add any additional files, libraries, and/or
*  helper function. All code must be authentically yours.
******************************************************************************/

/**************************************************************************//**
* @fn			void LCD_drawCircle(uint8_t x0, uint8_t y0, uint8_t radius,uint16_t color)
* @brief		Draw a colored circle of set radius at coordinates
* @note
*****************************************************************************/
void LCD_drawCircle(uint8_t x0, uint8_t y0, uint8_t radius,uint16_t color)
{
	//Scan the square
	LCD_setAddr(x0-radius, y0-radius, x0+radius, y0+radius);
	
	//CLREA
	clear(LCD_PORT, LCD_TFT_CS);
	
	int curr_x = 0;
	int curr_y = 0;
	for(int i= (-1*radius); i<= radius; i++){
		for (int j=-radius; j <= radius; j++){
	     curr_x = x0+i;
		 curr_y = y0+j;
		 
		 if ((pow((curr_x-x0),2)+pow((curr_y-y0),2)) < pow(radius,2)){
			 SPI_ControllerTx_16bit_stream(color);
		 }
		 else{
			 SPI_ControllerTx_16bit_stream(WHITE);
		 }
		}
	}
	//set CS to high
	set(LCD_PORT, LCD_TFT_CS);
}


/**************************************************************************//**
* @fn			void LCD_drawLine(short x0,short y0,short x1,short y1,uint16_t c)
* @brief		Draw a line from and to a point with a color
* @note
*****************************************************************************/
void LCD_drawLine(short x0,short y0,short x1,short y1,uint16_t color)
{
	short dx = abs(x1 - x0);
	short dy = abs(y1 - y0);
	short sx, sy;

	if (x0 < x1) {
		sx = 1;
		} else {
		sx = -1;
	}

	if (y0 < y1) {
		sy = 1;
		} else {
		sy = -1;
	}

	int err = dx - dy;

	while (1) {
		LCD_drawPixel(x0, y0, color);

		if (x0 == x1 && y0 == y1) {
			break;
		}

		int e2 = 2 * err;
		if (e2 > -dy) {
			err -= dy;
			x0 += sx;
		}

		if (e2 < dx) {
			err += dx;
			y0 += sy;
		}
	}
	//set CS to high
	set(LCD_PORT, LCD_TFT_CS);
}

/**************************************************************************//**
* @fn			void LCD_drawBlock(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,uint16_t color)
* @brief		Draw a colored block at coordinates
* @note
*****************************************************************************/
void LCD_drawBlock(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,uint16_t color)
{
	//set address
	LCD_setAddr(x0,y0,x1,y1);

	// Fill this out
	clear(LCD_PORT, LCD_TFT_CS);	//CS pulled low to start communication

	uint8_t curr_x = x0;
	uint8_t curr_y = y0;
	for (uint8_t x_i=0; x_i<x1-x0; x_i++){
		curr_x = x_i;
		for(uint8_t y_i=0; y_i < y1-y0; y_i++){
			// Current point we're checking
			curr_y = y_i;

			// use stream
			SPI_ControllerTx_16bit_stream(color);
			
		}
	}
    //set CS to high
	set(LCD_PORT, LCD_TFT_CS);	
}
/**************************************************************************//**
* @fn			void LCD_setScreen(uint16_t color)
* @brief		Draw the entire screen to a color
* @note
*****************************************************************************/
void LCD_setScreen(uint16_t color) {
	LCD_setAddr(0,0,LCD_WIDTH,LCD_HEIGHT);
	//CLEAR
	clear(LCD_PORT, LCD_TFT_CS);

	for(int i=0; i< LCD_WIDTH; i++){
		for(int j=0; j< LCD_HEIGHT; j++){
			SPI_ControllerTx_16bit_stream(color);
		}
	}
	//set CS to high
	set(LCD_PORT, LCD_TFT_CS);
}

/**************************************************************************//**
* @fn			void LCD_drawString(uint8_t x, uint8_t y, char* str, uint16_t fg, uint16_t bg)
* @brief		Draw a string starting at the point with foreground and background colors
* @note
*****************************************************************************/
void LCD_drawString(uint8_t x, uint8_t y, char* str, uint16_t fg, uint16_t bg)
{
	// // Fill this out
	// uint16_t row = str - 0x20;
	// // iterate throguh string
	// int i = 0;
	// int j = 0;
	// char character;
	// int str_len = strlen(str);
	// //set address 

	// LCD_setAddr(x,y,5,8);
	
	// // Clear CS
	// clear(LCD_PORT, LCD_TFT_CS);	//CS pulled low to start communication

	// for (int s = 0; str[s] != '\0'; s++) {
	// 	character = str[s];
	// 	LCD_drawChar(x+6*s,y,character,fg,bg);

	// }

	uint16_t character;
	int i=0;
	for(i=0; i< strlen(str); i++){
		character = str[i];
		LCD_drawChar(x, y, character, fg, bg);
		x += 6;
	}

}