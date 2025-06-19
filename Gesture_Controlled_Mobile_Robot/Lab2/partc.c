/*
 * Lab2 Part C
 * Created: 10/5/2023
 * Author : vbwanere
 */
#include <avr/io.h> 
// Define the CPU frequency (16 MHz for the Arduino). 
// This is necessary for _delay_ms() to work properly. 
#define F_CPU 16000000UL 
#include <util/delay.h>
#include <avr/interrupt.h>

/*######################## Include necessary libraries above #######################*/


////////////////////////////////////////////////////////////////////////////////
//           Part C. Using Input Capture to detect the switch state           //
////////////////////////////////////////////////////////////////////////////////
// /*

// a function to configure the hardware:
void configure_hardware(){
    DDRB |= (1 << PORTB5);    // set pin 13 as output(on-board LED).
    DDRB &= ~(1 << PORTB0);   // set pin 8 as input.
    PORTB |= (1 << PORTB0);   // enable pull-up resistor on pin 8.
    
    TCCR1B |= (1 << ICNC1);    // activate noise canceler for input capture.
    TCCR1B &= ~(1 << ICES1);    // set initial edge detection to falling edge.
    TIMSK1 |= (1 << ICIE1);    // enable the input capture interrupt for Timer1.
    TCCR1B |= (1 << CS10);    // start Timer1 without prescaler.
    sei();    // enable global interrupts.
}

// main function:
int main(void) {
    configure_hardware();
    while (1) {
    } 
}

// TIMER1_CAPT_vect: Interrupt vector for Timer1 Input Capture event.
ISR(TIMER1_CAPT_vect) {
    if (PINB & (1 << PB0)) { // If button is released.
        PORTB &= ~(1 << PB5); // LED is off.
        TCCR1B &= ~(1 << ICES1); // Set edge detection on falling edge.
    }
    else { // if button is pressed.
        PORTB |= (1 << PB5); // Turn on the LED.
        TCCR1B |= (1 << ICES1); // Set edge detection on rising edge.
    }
    TIFR1 |= (1 << ICF1); // Clear the input capture flag.
}
// */
/*------------------------------------------------------------------------------*/
