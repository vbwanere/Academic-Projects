#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include "uart.h"
#include <avr/interrupt.h>
#include <util/delay.h>

// UART Stuff
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
char String[100];

// Variable Initialization
volatile uint32_t edge = 0;
volatile uint32_t width = 0;
volatile uint32_t true_time = 0;
volatile uint16_t n = 0;
volatile uint32_t dist = 0;
volatile uint32_t oc_val = 0;
volatile uint8_t duty_cycle = 0;

// For FINAL PROJECT
// I used the same pins. On a separate Arduino. 

void Initialize() {
	// Disable the global interrupts flag (disable interrupts during init)
	cli();

	UART_init(BAUD_PRESCALER);

	DDRB |= (1<<DDB2); // set PB2 as output (outputting to Trigger)
	DDRB &= ~(1<<DDB0); // set PB0 as input (receive from echo)

	DDRD |= (1<<PORTD5); // Buzzer output  (PD5, ~5)
	
	DDRB &= ~(1<<PINB4); // Button

	// ULTRA SONIC SENSOR

	//Prescale 64
	TCCR1B &= ~(1<<CS12);
	TCCR1B |= (1<<CS11);
	TCCR1B |= (1<<CS10);

	// Normal Mode
	TCCR1A &= ~(1<<WGM10);
	TCCR1A &= ~(1<<WGM11);
	TCCR1B &= ~(1<<WGM12);
	TCCR1B &= ~(1<<WGM13);

	
	TCCR1B |= (1<<ICES1); // Capture a rising edge
	TIMSK1 |= (1<<ICIE1); // Enable input capture interrupt
	TIMSK1 |= (1<<TOIE1); // Overflow interrupt enable
	TIFR1 |= (1 << ICF1); // set interrupt flag to 0 (?)
	
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	// Buzzer Frequency Gen
	
	// Prescale 64
	TCCR0B &= ~(1 << CS12); //0
	TCCR0B |= (1 << CS11); //1
	TCCR0B |= (1 << CS10);  //1

	// Enable PWM Phase Correct mode
	TCCR0B |= (1 << WGM02); //1
	TCCR0A &= ~ (1 << WGM01);  //0
	TCCR0A |= (1 << WGM00);  //1

	// TIMSK0 |= (1<<OCIE0A); // enable output compare DO NOT ACTIVATE
	TCCR0A |= (1<<COM0A0); // toggle on compare
	
	TCCR0A |=  (1 << COM0B1); // Set on OCR0B
	TCCR0A &= ~(1 << COM0B0);
	
	// ADC Setup
	PRR &= ~ (1<<PRADC); //clear power reduction
	
	ADMUX |=  (1<<REFS0);
	ADMUX &= ~ (1<<REFS1);// Select V_ref = AVcc
	
	ADMUX &= ~ (1<<MUX0);
	ADMUX &= ~ (1<<MUX1);
	ADMUX &= ~ (1<<MUX2);
	ADMUX &= ~ (1<<MUX3); // Select Channel 0
	
	ADCSRA |= (1<<ADATE);//Set to auto trigger
	
	ADCSRB &= ~(1<<ADTS0);
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS2); // Free running
	
	DIDR0 |= (1<<ADC0D);
	
	ADCSRA |= (1<<ADEN);
	
	ADCSRA |= (1<<ADSC);

	sei(); // Turn on global interrupts again
}

ISR(TIMER1_CAPT_vect) // Runs when pin PB0 is HIGH
{
	true_time = ICR1 + n*65536;
	width = true_time - edge;
	edge = true_time;
	TCCR1B ^= (1<<ICES1); // switch to opposite edge

	if (TCCR1B & (1<<ICES1))
	
	{ // if we're reading a rising edge
		
		// Convert edge from ticks to ms to cm
		dist = (uint32_t)((width * 64) / (16 * 58)) ; // cm
		oc_val = (uint8_t)(dist*0.086+29.57);
	if (PINB & (1 << PINB4))
	{
		// Now it's toggled at OCR0B instead of OCR0A.
		// Which means the frequency is doubled with original OCR0A value.
		// To keep the frequency same, we need double the OCR0A value below
		if (oc_val < 32 && oc_val > 27)       OCR0A = 60;
		else if (oc_val < 36 && oc_val >=32)  OCR0A = 64;
		else if (oc_val < 40 && oc_val >= 36) OCR0A = 72;
		else if (oc_val < 45 && oc_val >= 40) OCR0A = 80;
		else if (oc_val < 47 && oc_val >= 45) OCR0A = 90;
		else if (oc_val < 53 && oc_val >= 47) OCR0A = 94;
		else if (oc_val < 60 && oc_val >= 53) OCR0A = 106;
		else if (oc_val >= 60)                OCR0A = 120;
		OCR0B = OCR0A*duty_cycle/100;
	}
	
	else if (!(PINB & (1<<PINB4)))
	{
		OCR0A = oc_val*2;
		OCR0B = OCR0A*duty_cycle/100;
	}
	
		
		sprintf(String, "Distance: %lu cm | ", dist);
		UART_putstring(String);

		sprintf(String, "OCR0A: %lu \n", oc_val);
		UART_putstring(String);
	}
	
}

ISR(TIMER1_OVF_vect){
	n++;
}

int main(void) {
	Initialize();
	while(1)
	{
		// Initialize ranging by sending 10 us pulse to TRIG (Pin PB2)
		PORTB &= ~(1<<PINB2);
		_delay_us(100);// Clear first
		
		ADC_trig();
		
		PORTB |= (1<<PINB2);
		_delay_us(10); // 10 us pulse
		PORTB &= ~(1<<PINB2);
	}
	
}

void ADC_trig ()
{
	
	if (ADC < 141 && ADC >= 50 )
	{
		duty_cycle=5;
	}
	if (ADC < 232 && ADC >= 141)
	{
		
		duty_cycle=10;
	}
	if (ADC < 323  && ADC >= 232)
	{
		
		duty_cycle=15;
	}
	if (ADC < 414 && ADC >= 323)
	{
		
		duty_cycle=20;
	}
	if (ADC <505  && ADC >= 414)
	{
		
		duty_cycle=25;
	}
	if (ADC < 596 && ADC >= 505)
	{
		
		duty_cycle=30;
	}
	if (ADC < 687 && ADC >= 596)
	{
		
		duty_cycle=35;
	}
	if (ADC < 778 && ADC >= 687)
	{
		duty_cycle=40;
	}
	if (ADC < 896 && ADC >= 778 )
	{
		
		duty_cycle=45;
	}
	if (ADC < 970 && ADC >=896 )
	{
		
		duty_cycle=50;
	}
	// sprintf(String, "Duty_cycle: %u \n ", duty_cycle);
	// UART_putstring(String);

}