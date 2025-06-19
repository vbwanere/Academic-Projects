/*
 * Lab 4: PONG
 * Created: Nov 13, 2023
 * Author : vbwanere
 */

#include <avr/io.h> 
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "uart.h"    
#include "ST7735.h"  // LCD library.
#include "LCD_GFX.h" // LCD graphics library.

#define F_CPU 16000000UL // 16 MHz clock speed.
#include <util/delay.h>
#define BAUD_RATE 9600
#define BAUD_PRESCALER ((F_CPU / (BAUD_RATE * 16UL)) - 1)
/*#################### Include necessary libraries above #####################*/

/*******************************************************************/
//           Configuring ADC, LED, MCU, and Interrupts.            //
/*******************************************************************/
// /*
void configureProject() {
    lcd_init(); // Initialize the screen.
	cli(); // Disable global interrupts.

	//initialize ADC
	PRR &= ~(1<<PRADC); 

	// Vref = AVcc:
	ADMUX |= (1<<REFS0);
	ADMUX &= ~(1<<REFS1);

	// Clock running at 125kHz:
	ADCSRA |= (1<<ADPS0);
	ADCSRA |= (1<<ADPS1);
	ADCSRA |= (1<<ADPS2);

	// ADC channel 0:
	ADMUX &= ~(1<<MUX0);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX3);

	// auto trigger:
	ADCSRA |= (1<<ADATE);

	// set to free running
	ADCSRB &= ~(1<<ADTS0);
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS2);

	// disable digital input buffer on ADC PIN:
	DIDR0 |= (1<<ADC0D);

	// ENBALE ADC:
	ADCSRA |= (1<<ADEN);

	// start conversion
	ADCSRA |= (1<<ADSC);
	
	// configure the LED pins:
	DDRC |= (1<<DDC1);
	DDRC |= (1<<DDC2);
	
	// configure the buzzer pin:
    DDRD |= (1<<DDD3);

	// configure the WiFi module pins:
    DDRC &= ~(1<<DDC3);
	DDRC &= ~(1<<DDC4); 
	sei();
}
// */