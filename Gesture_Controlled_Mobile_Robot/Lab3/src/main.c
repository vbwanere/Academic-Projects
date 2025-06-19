/*
 * Lab 3: Theremin
 * Created: Oct 21, 2023
 * Author : vbwanere
 */
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "uart.h"    
 
#define F_CPU 16000000UL // CPU clock freq is independent clock prescaler.
#include <util/delay.h>
#define BAUD_RATE 9600
#define BAUD_PRESCALER ((F_CPU / (BAUD_RATE * 16UL)) - 1)
char String[25];    // global character array to store string.
/*#################### Include necessary libraries above #####################*/



////////////////////////////////////////////////////////////////////////////////
//                   Lab3 Theremin: F Putting it All Together                 //
////////////////////////////////////////////////////////////////////////////////
// /*

// Configure the MCU and initialize the peripherals:
void configure_MCU() {
	UART_init(BAUD_PRESCALER); // Initialize the UART.		
	cli(); // Disable global interrupts.

   	ADMUX = (1 << REFS0); // Set the reference voltage to AVCC.
   	ADCSRA = (1 << ADEN); // Enable the ADC.
	// Prescaler 128:
	ADCSRA = (1 << ADPS2);
	ADCSRA = (1 << ADPS1);
	ADCSRA = (1 << ADPS0); 
  	DIDR0 |= (1 << ADC5D);  //disable digital input buffer on ADC5
	
	DDRB |= (1 << DDB3); // Set PB3 as output.
	DDRB |= (1 << PORTB5); // Set PB5 HIGH.
	DDRD &= ~(1 << DDD7); // Set PD7 as input.
	PORTD |= (1 << PORTD7); // Enable pull-up resistor on PD7.
	
	TCCR2A = (1 << COM2A0)|(1 << WGM20);  // Timer2 in Phase Correct PWM mode.
	TCCR2B = (1 << CS22) | (1 << WGM22);  // Prescaler 64.
	
	// Configure the Ultrasonic Sensor:
	DDRD |= (1 << DDD6); // Set PD6 as trigger pin.
	// Timer0 in Fast PWM mode:
	TCCR0A = (1 << COM0A1);
	TCCR0A |= (1<< COM0A0);
	TCCR0A |= (1 << WGM01);
	TCCR0A |= (1 << WGM00);
	TCCR0B |= (1 << CS01); // Prescaler 8.
	OCR0A = 235; //10uS trigger pulse
	
	// Timer1 in normal mode:
	TCCR1A &= ~(WGM10);                
	TCCR1A &= ~(WGM11);
	TCCR1B &= ~(WGM12);
	
	DDRB &= ~(1 << DDB0); // Set PB0 as input for Input Capture for echo pulse.

	TCCR1B = (1<<ICNC1);  // Enable noise canceler.
	TCCR1B = (1<<ICES1); // Rising edge detection.
	TCCR1B = (1<<CS11);  // Prescaler 8.
	TIMSK1 |= (1<<ICIE1);  // Enable input capture interrupt.             
	TIMSK2 |= (1<<OCIE2B);   // Enable compare match interrupt.       

	sei(); //enable global interrupts
}


// Global variables:
volatile uint16_t pulse_begin = 0;
volatile uint16_t pulse_end = 0;
volatile uint16_t on_time = 0;
uint16_t echo_pulse;
uint16_t distance;
volatile int continuous_Mode = 0;
// Get the distance from the ultrasonic sensor:
uint16_t get_distance() {	
	// Timer 1 with prescaler 8:
	echo_pulse = (float)on_time * 32768 / 65536; // Convert the duty cycle to pulse width.   
	distance = echo_pulse * 0.017; // Convert the pulse width to distance. 
	return distance
}

// Set the duty cycle based on ADC value:
void set_duty_cycle() { 
	if(ADC <= 1020 && ADC >= 921) {
		OCR2B = 0.5*OCR2A;
	} else if(ADC <= 920 && ADC >= 821) {
		OCR2B = 0.45*OCR2A;
	} else if(ADC <= 820 && ADC >= 721) {
		OCR2B = 0.4*OCR2A;
	} else if(ADC <= 720 && ADC >= 621) {
		OCR2B = 0.35*OCR2A;
	} else if(ADC <= 620 && ADC >= 521) {
		OCR2B = 0.3*OCR2A;
	} else if(ADC <= 520 && ADC >= 421) {
		OCR2B = 0.25*OCR2A;
	} else if(ADC <= 420 && ADC >= 321) {
		OCR2B = 0.20*OCR2A;
	} else if(ADC <= 320 && ADC >= 221) {
		OCR2B = 0.15*OCR2A;
	} else if(ADC <= 220 && ADC >= 121) {
		OCR2B = 0.10*OCR2A;
	} else if(ADC < 121) {
		OCR2B = 0.05*OCR2A;
	}
}

// Set the frequency based on the distance:
void discrete_Mode() {
	if(distance >= 2 && distance <= 16) {
		OCR2A = 29;
	} else if(distance >= 17 && distance <= 31) {
		OCR2A = 32;
	} else if(distance >= 32 && distance <= 46) {
		OCR2A = 35;
	} else if(distance >= 47 && distance <= 61) {
		OCR2A = 40;
	} else if(distance >= 62 && distance <= 76) {
		OCR2A = 45;
	} else if(distance >= 77 && distance <= 81) {
		OCR2A = 48;
	} else if(distance >= 82 && distance <= 96) {
		OCR2A = 53;
	} else if(distance >= 97 && distance <= 123) {
		OCR2A = 60;
	}
}

// Main function:
int main() {
	configure_MCU(); // Configure the MCU and initialize the peripherals.
	
	while (1) {
		ADCSRA |= (1 << ADSC); // Start the ADC conversion.
		while (ADCSRA & (1 << ADSC)); // Wait until the conversion is done.
		distance = get_distance(); // Get the distance from the ultrasonic sensor.
		
		if (continuous_Mode) { // If the continuous mode is enabled.
			//min value of OCRA is 29 and max is 60.
			//min value measured is 2 max value measured is 99.
	    	OCR2A = 0.25*distance + 28.48;       	
			set_duty_cycle(); // Set the duty cycle based on ADC value.
		} 
		else { // If the discrete mode is enabled.
			if (distance <= 99) {
				discrete_Mode();
				set_duty_cycle(); // Set the duty cycle based on ADC value.       
			}
	    }
		_delay_ms(100);  
		
		// Toggle the mode if the button is pressed:
		if (!(PIND & (1<<PIND7))) {
			continuous_Mode = !continuous_Mode;  //Toggle the mode
		}
	}
	return 0;
}

// ISR for Timer 1 input capture interrupt:
ISR(TIMER1_CAPT_vect) {
	if((TCCR1B & (1<<ICES1)) == (1<<ICES1)) { // If the rising edge is detected.
		pulse_begin = ICR1;
	}
	else { // If the falling edge is detected.
		pulse_end = ICR1;
	}
	if (pulse_begin != 0 && pulse_end != 0) { // If both pulses have been captured.
		on_time = pulse_end - pulse_begin;
		pulse_begin = 0;
		pulse_end = 0;
	}
	TCCR1B ^= (1<<ICES1); // Toggle the edge detection.
	TIFR1 = (1<<ICF1);  // Clear the input capture flag.
}

// ISR for Timer 2 compare match interrupt:
ISR(TIMER2_COMPB_vect) {
	if((PINB & (1 << PORTB3)) == (1 >> PORTB3)) { // If the pin is LOW.
		PORTB ^= (1 << PORTB5); // Toggle the pin.
	}
	else {
		PORTB &= ~(1 << PORTB5);  // Set the pin LOW.
	}
}
// */
/*-----------------------------------------------------------------------------*/