/*
 * Lab2 Part D
 * Created: 10/5/2023 5:42 PM
 * Author : vbwanere
 */
#include <avr/io.h> 
#include <avr/interrupt.h>
#include "uart.h"

// This is necessary for _delay_ms() to work properly. 
#define F_CPU 16000000UL /// CPU clock freq is independent clock prescaler
#include <util/delay.h>
#define BAUD_RATE 9600
#define BAUD_PRESCALER ((F_CPU / (BAUD_RATE * 16UL)) - 1)

/*######################## Include necessary libraries above #######################*/

////////////////////////////////////////////////////////////////////////////////
//        Part D. Printing Morse Code Characters using Timer Counter          //
////////////////////////////////////////////////////////////////////////////////
/*
// Define constants for time intervals
#define DOT 30   // Minimum duration for a dot in milliseconds
#define DASH 200 // Minimum duration for a dash in milliseconds
#define SPACE 400 // Maximum duration for a space in milliseconds

volatile uint16_t pressDur = 0;
volatile uint8_t buttonState = 0; // 0: Button released, 1: Button pressed

// Function to configure the hardware
void configure_hardware() {
    DDRB |= (1 << PORTB5);   // Set pin 13 as output (on-board LED).
    DDRB &= ~(1 << PORTB0);  // Set pin 8 as input.
    PORTB |= (1 << PORTB0);  // Enable pull-up resistor on pin 8.
    TCCR1B |= (1 << ICNC1);  // Activate noise canceler for input capture.
    TCCR1B &= ~(1 << ICES1); // Set initial edge detection to falling edge.
    TIMSK1 |= (1 << ICIE1);  // Enable the input capture interrupt for Timer1.
    TCCR1B |= (1 << CS10);   // Start Timer1 with no prescaler.
    sei();                   // Enable global interrupts.
}

// Main function
int main(void) {
    configure_hardware();
    while (1) {
        // The main loop remains empty. The main action happens inside the ISR.
    }
}

// TIMER1_CAPT_vect: Interrupt vector for Timer1 Input Capture event.
ISR(TIMER1_CAPT_vect) {
    if (PINB & (1 << PB0)) { // If the button is not pressed.
        if (buttonState == 1) {
            // Button was pressed before.
            pressDur = TCNT1;    // calculate the duration of pressed state.

            if (pressDur >= DOT && pressDur < DASH) {
                // Detected a dot (30ms to 200ms)
                PORTB &= ~(1 << PB5); // Turn off the LED
            }
            else if (pressDur >= DASH && pressDur <= SPACE) {
                // Detected a dash (200ms to 400ms)
                // Add code here to handle dashes if needed.
            }

            buttonState = 0; // Button is released
        }

        TCCR1B &= ~(1 << ICES1); // Set edge detection to falling edge.
    }
    else { // If the button is pressed.
        if (buttonState == 0) { // Button was not pressed before.
            TCNT1 = 0; // Reset the counter.
        }
        PORTB |= (1 << PB5); // Turn on the LED
        TCCR1B |= (1 << ICES1); // Set edge detection to rising edge.
        buttonState = 1; // Button is now pressed.
    }

    TIFR1 |= (1 << ICF1); // Clear the input capture flag by writing a 1 to it.
}
/*------------------------------------------------------------------------------*/

