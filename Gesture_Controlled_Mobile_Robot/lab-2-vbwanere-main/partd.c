/*
 * Lab2 Part D
 * Created: 10/10/2023
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
//        Part D. Printing Morse Code Characters using Input Capture          //
////////////////////////////////////////////////////////////////////////////////
/*

#define DOT 1875    // duration for a dot.
#define DASH 12500    // duration for a dash.
#define SPACE 25000    // duration for a space.

volatile uint16_t switch_pressed = 0;    // Start time of switch press.
volatile uint16_t switch_released = 0;    // End time of switch press.
uint8_t flag = 0;    // Flag to indicate if the switch has been pressed.
uint16_t overflow_count;    // Number of times Timer1 has overflowed.

void configure_hardware() {
    DDRB &= ~(1 << PORTB0);    // Set pin 8 as input
    PORTB |= (1 << PORTB0);    // Enable pull-up resistor on pin 8

    // Set the timer prescaler to 1 / 256:
    TCCR1B |= (1 << CS12); 
    TCCR1B &= ~(1 << CS11);
    TCCR1B &= ~(1 << CS10);

    TCCR1B &= ~(1 << ICES1);    // Input capture falling edge.
    TIMSK1 |= (1 << ICIE1);    // Enable Input Capture Interrupt.
    TIMSK1 |= (1 << TOIE1);    // Enable Timer1 Overflow Interrupt.
    sei();    // Enable Global Interrupts.
}

// Main function:
int main() {
    configure_hardware();
    UART_init(BAUD_PRESCALER);
    while (1) { // do nothing.
    }
    return 0;
}

// Interrupt vector for Timer1 Input Capture event.
ISR(TIMER1_CAPT_vect) {
    if (TCCR1B & (1<<ICES1)) {
        switch_pressed = ICR1;    // Capture the start time.
        flag = 1;    // Set flag to 1.
    }

    else {
        switch_released = ICR1;    // Capture the end time.
        uint16_t press_dur = switch_released - switch_pressed;
        if(press_dur < 0) { // Check for overflow.
            press_dur = press_dur + 62500;
        }
        else if (press_dur >= DOT && press_dur < DASH) {
            UART_send('.');
        }
        else if (press_dur >= DASH && press_dur <= SPACE) {
            UART_send('-');
        }
        flag=0;
    }
    TCCR1B ^= (1<<ICES1);    //flip edge after detection.
}
// Interrupt vector for Timer1 overflow event.
ISR(TIMER1_OVF_vect) {
    overflow_count++;
    uint32_t current_time = ((uint32_t)overflow_count << 16) + TCNT1;

    // Check if the switch has been pressed for more than 400ms.
    if(current_time - switch_released > 25000 && flag==0) { 
        UART_putstring(" ");
        switch_released = current_time;
    }
}
*/
/*------------------------------------------------------------------------------*/
