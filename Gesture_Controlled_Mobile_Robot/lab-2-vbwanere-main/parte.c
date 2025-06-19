/*
 * Lab2 Part E
 * Created: 11/10/2023
 * Author : vbwanere
 */

#include <avr/io.h> 
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "uart.h"

// This is necessary for _delay_ms() to work properly. 
#define F_CPU 16000000UL /// CPU clock freq is independent clock prescaler
#include <util/delay.h>
#define BAUD_RATE 9600
#define BAUD_PRESCALER ((F_CPU / (BAUD_RATE * 16UL)) - 1)

/*######################## Include necessary libraries above #######################*/

////////////////////////////////////////////////////////////////////////////////
//        Part E. Decoding Morse Code Characters Generated from Part D        //
////////////////////////////////////////////////////////////////////////////////
// /*

void configure_hardware(){
    DDRB |= (1 << PORTB1);    // Set pin 9 as output.
    DDRB |= (1 << PORTB2);    // Set pin 10 as output.
    DDRB &= ~(1 << PORTB0);    // Set pin 8 as input.

    CLKPR |= (1 << CLKPCE);    // Enable clock prescaler update.
    
    // Set clock prescaler to 1/256.
    TCCR1B |= (1 << CS12);
    TCCR1B |= (1 << CS10);

    TCCR1B |= (1 << ICNC1);    // Activate noise canceler for input capture.
    TCCR1B |= (1 << ICES1);    // Set initial edge detection to rising edge.
    TIMSK1 |= (1<< ICIE1);    // Enable the input capture interrupt for Timer1.
    sei();    // Enable global interrupts.
}

char String[25];    // Define constants for time intervals:
char buffer[4];    // Morse code buffer
volatile int buffIdx = 0;
volatile char currentChar = ' ';

const char dict_Morse[36][6] = {
    ".-",    // A
    "-...",  // B
    "-.-.",  // C
    "-..",   // D
    ".",     // E
    "..-.",  // F
    "--.",   // G
    "....",  // H
    "..",    // I
    ".---",  // J
    "-.-",   // K
    ".-..",  // L
    "--",    // M
    "-.",    // N
    "---",   // O
    ".--.",  // P
    "--.-",  // Q
    ".-.",   // R
    "...",   // S
    "-",     // T
    "..-",   // U
    "...-",  // V
    ".--",   // W
    "-..-",  // X
    "-.--",  // Y
    "--..",  // Z
    "-----", // 0
    ".----", // 1
    "..---", // 2
    "...--", // 3
    "....-", // 4
    ".....", // 5
    "-....", // 6
    "--...", // 7
    "---..", // 8
    "----."  // 9
};

unsigned int switch_pressed = 0;
unsigned int switch_released = 0;

int main(void) {
    UART_init(BAUD_PRESCALER);
    configure_hardware();
    while (1) {
        if((TCNT1 - switch_released) > 45000) {
            TCNT1 = switch_released;
            decodeMorse();
        }
    }
}

ISR(TIMER1_CAPT_vect) { // Interrupt vector for Timer1 Input Capture event.
    if(TCCR1B & (1<<ICES1)) {
        switch_pressed = ICR1;
    }
    else{  
        switch_released = ICR1;  
        if((switch_released - switch_pressed) >= 468 && (switch_released - switch_pressed) <= 3000) {
            sprintf(String,". ");
            UART_putstring(String);
            buffer[buffIdx++] = '.';
            PORTB |= (1 << PORTB1);
            PORTB &= ~(1 << PORTB2);
        }
        else if((switch_released - switch_pressed) >= 3000 && (switch_released - switch_pressed) <= 6250){
            sprintf(String, "- ");
            UART_putstring(String);
            buffer[buffIdx++] = '-';
            PORTB |= (1 << PORTB2);
            PORTB &= ~(1 << PORTB1);
        }        
    }
    TCCR1B ^= (1<<ICES1);    // flip edge after detection.
}

void decodeMorse(){  //decoding morse code
    int i;
    for (i = 0; i < 36; i++) {
        if (strcmp(buffer, dict_Morse[i]) == 0) {
            currentChar = (i < 26) ? ('A' + i) : ('0' + (i - 26));
            UART_send(currentChar);
            break;
            i=0;
            switch_released = 0;    // reset switch released
            switch_pressed = 0;    // reset switch pressed
        }
    }
    buffIdx = 0;   // reset buffer index
    // clear buffer:
    buffer[3]=0;
    buffer[2]=0;
    buffer[1]=0;
    buffer[0]=0;
}
// */
/*------------------------------------------------------------------------------*/