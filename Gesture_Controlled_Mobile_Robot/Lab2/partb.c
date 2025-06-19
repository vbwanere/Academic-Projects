/*
 * Lab2 Part B
 * Created: 10/5/2023 5:42:21 PM
 * Author : vbwanere
 */
#include <avr/io.h> 
// Define the CPU frequency (16 MHz for the Arduino). 
// This is necessary for _delay_ms() to work properly. 
#define F_CPU 16000000UL 
#include <util/delay.h>

/*######################## Include necessary libraries above #######################*/

////////////////////////////////////////////////////////////////////////////////
//               B. Detecting state of a low asserting push button            //
////////////////////////////////////////////////////////////////////////////////
// /*

// a function to press the button to turn LED on:
void press_button(){
    // PINB0 is pulled up to 5V when the button is not pressed.
    if(PINB & (1 << PINB0)){ // if button is not pressed.
        PORTB &= ~(1 << PORTB1);    // set pin 9 to LOW.
    }
    else{ // if button is pressed.
        PORTB |= (1 << PORTB1);   // set pin 9 to HIGH.
    }
}

int main(void){
    // set pin 9 as output:
    DDRB |= (1 << PORTB1);

    DDRB &= ~(1 << PORTB0);    // set pin 7 as input.
    PORTB |= (1 << PORTB0);    // enable pull-up resistor on pin 7.
    
    while (1){
        press_button(); // LED on pin 9 will turn on when button is pressed.
    }
    return 0; 
}
// */
/*------------------------------------------------------------------------------*/