/*
 * Lab 3: Theremin
 * Created: Oct 17, 2023
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
//               B.1 Frequency Generation using Timer Overflow                //
////////////////////////////////////////////////////////////////////////////////
/*
void configure_MCU() {
	cli();
	DDRD |= (1 << PD6);  // Set pin 6 to be an output pin.
    PORTD |= (1 << PD6);  // Set pin 6 to be high. 
	
	// Timer0 in Normal Mode:
	TCCR0A &= ~(WGM00); 
	TCCR0A &= ~(WGM01);
	TCCR0B &= ~(WGM02);
	TCCR0B = (1 << CS02) ; // Prescaler 256
	TIMSK0 = (1 << TOIE0); // Enable Timer0 Overflow Interrupt
	sei();
}

int main() {	
	configure_MCU();  // Configure the MCU.
    while (1) {
		// Do nothing.
		// The square wave is generated in the interrupt service routine.
	}
    return 0;
}

// Interrupt service routine for Timer0 overflow:
ISR(TIMER0_OVF_vect) {
    // Toggle the output pin (PD6) to generate a square wave
    PORTD ^= (1 << PD6);
}
*/
/*-----------------------------------------------------------------------------*/



////////////////////////////////////////////////////////////////////////////////
//               B.2 440 Hz square wave using Timer0 in Normal Mode           //
////////////////////////////////////////////////////////////////////////////////
/*
void configure_MCU() {
	cli();
	DDRD |= (1 << PORTD6);  // Set pin 6 to be an output pin.
	
	// Timer0 in Normal Mode:
	TCCR0A &= ~(WGM00); 
	TCCR0A &= ~(WGM01);
	TCCR0B &= ~(WGM02);
	TCCR0B |= (1 << CS02) ; // Prescaler 256.
	OCR0A = OCR_VALUE; // Set the Output Compare Register to the desired value.
	TIMSK0 = (1 << OCIE0A); // Enable Timer0 Output Compare A Interrupt.
	sei();
}

int main() {	
	configure_MCU();  // Configure the MCU.
    while (1) {
		// Do nothing.
		// The square wave is generated in the interrupt service routine.
	}
    return 0;
}

// Interrupt service routine for Timer0 overflow:
ISR(TIMER0_COMPA_vect) {
	PORTD ^= (1<<PORTD6);
	OCR0A += 70;
	if(OCR0A > 255)
	OCR0A = OCR0A - 255;
}
*/
/*-----------------------------------------------------------------------------*/


////////////////////////////////////////////////////////////////////////////////
//               B.3 440 Hz square wave using Timer0 in CTC Mode              //
////////////////////////////////////////////////////////////////////////////////
/*
void configure_MCU() {
    cli();
    DDRD |= (1 << PORTD6);  // Set pin 6 to be an output pin.

    // Timer0 in CTC Mode:
	TCCR0A |= (1<<COM0A0) | (1<<WGM01);
    TCCR0B = (1 << CS02); // Prescaler 256
    OCR0A = 70; // Set the Output Compare Register to the desired value.
    TIMSK0 = (1 << OCIE0A); // Enable Timer0 Output Compare A Interrupt.
    sei();
}

int main() {
    configure_MCU();  // Configure the MCU.
    while (1) {
        // Do nothing.
        // The PWM wave is generated in the interrupt service routine.
    }
    return 0;
}

// Interrupt service routine for Timer0 Output Compare A:
ISR(TIMER0_COMPA_vect) {
    // Toggle the output pin (PD6) to generate PWM
    PORTD ^= (1 << PORTD6);
}
*/
/*-----------------------------------------------------------------------------*/



////////////////////////////////////////////////////////////////////////////////
//        B.4 440 Hz square wave using Timer0 in Phase Correct PWM Mode       //
////////////////////////////////////////////////////////////////////////////////
/*
void configure_MCU() {
	cli();
	DDRD |= (1 << PORTD6);  // Set pin 6 to be an output pin.
	
	//Timer 0 Phase correct PWM mode:
	TCCR0B |= (1<<CS02);  // Prescaler to 256.	
	TCCR0A |= (1<<WGM00);
	TCCR0B |= (1<<WGM02);	
	TCCR0A |= (1<<COM0A0);
	OCR0A = 35;
	sei();
}

int main() {
	configure_MCU();
	while (1) {
		// Do nothing.
	}
}
*/
/*-----------------------------------------------------------------------------*/


////////////////////////////////////////////////////////////////////////////////
//                C Measuring distance using Ultrasonic Sensor                //
////////////////////////////////////////////////////////////////////////////////
/*
static volatile uint32_t first_reading = 0;
static volatile uint32_t second_reading = 0;
static volatile uint32_t duty_cycle = 0;
char measure[25];

void HCSR04_Init(){ //initialize ultrasonic sensor.
	cli(); //clear prior interrupts
	
	DDRD |= (1<<PORTD6); //set PD6 as output trigger.
	DDRB |= (1<<PORTB3); // set PB3 as output for buzzer.
	// Fast PWM for Timer0
	TCCR0A = (1<<COM0A1)|(1<<COM0A0)|(1<<WGM01)|(1<<WGM00); 
	TCCR0B |= (1<<CS01); //prescaler = 8 for timer 0.

	//10uS trigger pulse
	// 118uS off-time (128uS repetition rate) ON from 235 to 255. 16M/8/
	OCR0A = 235; 
	//Timer 1 running in normal mode
	DDRB &= ~(1<<PORTB0); //PB0 as input (ICP1)
	TCCR1B = (1<<ICNC1)|(1<<ICES1)|(1<<CS11); //timer 1 like C
	TCCR2A = (1<<COM2A0)|(1<<WGM20);  //timer 2 for buzzer, toggle when OCRA
	TCCR2B = (1<<CS22)|(1<<WGM22); //64 prescalar, phasecorrect for buzzer
	//OCR2A = 35;
	
	TIMSK1 |= (1<<ICIE1); //enable timer1 input capture interrupt
	sei();//enable global interrupts
}

uint32_t getDistance(){
	static uint32_t echo_pulse_uS;
	static uint32_t distance_cm;
	//32768uS = 65536 clock ticks for Timer 1 with prescaler = 8
	echo_pulse_uS = (float)duty_cycle * 32768 / 65536;
	distance_cm = echo_pulse_uS * 0.034 / 2;
	
	return distance_cm;
}

ISR(TIMER1_CAPT_vect){
	if ((TCCR1B & (1<<ICES1)) == (1<<ICES1)){
		first_reading = ICR1;
	}
	else{
		second_reading = ICR1;
	}
	
	if (first_reading != 0 && second_reading != 0)
	{
		duty_cycle = second_reading - first_reading;
		first_reading = 0;
		second_reading = 0;
	}
	
	TCCR1B ^= (1<<ICES1); //toggle edge detection bit.
	TIFR1 = (1<<ICF1); //clear Input Capture Flag.
}
int main(){
	UART_init(BAUD_PRESCALER);
	HCSR04_Init();
	while (1){
		uint32_t distance = getDistance();

		//min value of OCRA is 29 and max is 59
		//min value measured is 2 max value measured is 99
		OCR2A = 0.309*distance + 28.38;  
		ultoa(distance, measure, 10);
		strcat(measure, "\n");
		UART_putstring(measure);
		//_delay_ms(2000);
	}
	
}
// */
/*-----------------------------------------------------------------------------*/


////////////////////////////////////////////////////////////////////////////////
//          D.1 Generating Continuous Frequencies based on Distance           //
////////////////////////////////////////////////////////////////////////////////
/*
static volatile uint32_t first_reading = 0;
static volatile uint32_t second_reading = 0;
static volatile uint32_t duty_cycle = 0;
char measure[25];

void HCSR04_Init(){ //initialize ultrasonic sensor.
	cli(); //clear prior interrupts
	
	DDRD |= (1<<PORTD6); //set PD6 as output trigger.
	DDRB |= (1<<PORTB3); // set PB3 as output for buzzer.
	// Fast PWM for Timer0
	TCCR0A = (1<<COM0A1)|(1<<COM0A0)|(1<<WGM01)|(1<<WGM00); 
	TCCR0B |= (1<<CS01); //prescaler = 8 for timer 0.

	//10uS trigger pulse
	// 118uS off-time (128uS repetition rate) ON from 235 to 255. 16M/8/
	OCR0A = 235; 
	//Timer 1 running in normal mode
	DDRB &= ~(1<<PORTB0); //PB0 as input (ICP1) for echo.
	TCCR1B = (1<<ICNC1)|(1<<ICES1)|(1<<CS11); //timer 1 like C
	TCCR2A = (1<<COM2A0)|(1<<WGM20);  //timer 2 for buzzer, toggle when OCRA
	TCCR2B = (1<<CS22)|(1<<WGM22); //64 prescalar, phasecorrect for buzzer
	//OCR2A = 35;
	
	TIMSK1 |= (1<<ICIE1); //enable timer1 input capture interrupt
	sei();//enable global interrupts
}

uint32_t getDistance(){
	static uint32_t echo_pulse_uS;
	static uint32_t distance_cm;
	//32768uS = 65536 clock ticks for Timer 1 with prescaler = 8
	echo_pulse_uS = (float)duty_cycle * 32768 / 65536;
	distance_cm = echo_pulse_uS * 0.034 / 2;
	
	return distance_cm;
}

ISR(TIMER1_CAPT_vect){
	if ((TCCR1B & (1<<ICES1)) == (1<<ICES1)){
		first_reading = ICR1;
	}
	else{
		second_reading = ICR1;
	}
	
	if (first_reading != 0 && second_reading != 0)
	{
		duty_cycle = second_reading - first_reading;
		first_reading = 0;
		second_reading = 0;
	}
	
	TCCR1B ^= (1<<ICES1); //toggle edge detection bit.
	TIFR1 = (1<<ICF1);//clear Input Capture Flag.
}
int main(){
	UART_init(BAUD_PRESCALER);
	HCSR04_Init();
	while (1){
		uint32_t distance = getDistance();

		//min value of OCRA is 29 and max is 59
		//min value measured is 2 max value measured is 99
		OCR2A = 0.26*distance + 28.48;  
		ultoa(distance, measure, 10);
		strcat(measure, "\n");
		UART_putstring(measure);
		_delay_ms(500);
	}
	
}
*/
/*-----------------------------------------------------------------------------*/


////////////////////////////////////////////////////////////////////////////////
//           D.2 Generating Discrete Frequencies based on Distance            //
////////////////////////////////////////////////////////////////////////////////
/*
char String[25];

uint32_t distance;
volatile uint32_t first_pulse = 0;
volatile uint32_t second_pulse = 0;
volatile uint32_t duty_cycle = 0;
volatile int continuousMode = 0;

const int buttonPin = 7;

uint32_t getDist(){
	uint32_t echo_pulse_uS;
	uint32_t distance_cm;
	//32768uS = 65536 clock ticks for Timer 1 with prescaler = 8
	echo_pulse_uS = (float)duty_cycle * 32768 / 65536;
	distance_cm = echo_pulse_uS * 0.034 / 2;
	return distance_cm;
}

ISR(TIMER1_CAPT_vect){
	if ((TCCR1B & (1<<ICES1)) == (1<<ICES1)){
		first_pulse = ICR1;
	}
	else{
		second_pulse = ICR1;
	}
	if (first_pulse != 0 && second_pulse != 0){
		duty_cycle = second_pulse - first_pulse;
		first_pulse = 0;
		second_pulse = 0;
	}
	TCCR1B ^= (1<<ICES1); //toggle edge detection bit
	TIFR1 = (1<<ICF1);//clear Input Capture Flag
}

void discrete()
{
	if(distance >= 2 && distance <= 9 )
	{
		OCR2A = 29;
		//volume_control();/
	}
	if(distance >= 10 && distance <= 19 )
	{
		OCR2A = 30;
		//volume_control();/
	}
	if(distance >= 20 && distance <= 29 )
	{
		OCR2A = 34;
		//volume_control();/
	}
	if(distance >= 30 && distance <= 39 )
	{
		OCR2A = 38;
		//volume_control();/
	}
	if(distance >= 40 && distance <= 49 )
	{
		OCR2A = 44;
		//volume_control();/
	}
	if(distance >= 50 && distance <= 59 )
	{
		OCR2A = 46;
		//volume_control();/
	}
	if(distance >= 60 && distance <= 69 )
	{
		OCR2A = 52;
		//volume_control();/
	}
	if(distance >= 70 && distance <= 99 )
	{
		OCR2A = 59;
		//volume_control();/
	}
}


void initialize(){
	UART_init(BAUD_PRESCALER);
	
	sprintf(String,"The year is %u\n", 2023);
	UART_putstring(String);
	
	cli();

	//Setup as output pin
	DDRB |= (1 << DDB3);
	//Setup as input pin
	DDRD &= ~(1<<DDD7);
	PORTD |= (1<<PORTD7);
	
	//Timer 2
	TCCR2A = (1<<COM2A0)|(1<<WGM20);
	TCCR2B = (1<<CS22) | (1<<WGM22);
	
	
	//Ultrasonic/
	DDRD |= (1<<DDD6); //set PD6 as output
	TCCR0A = (1<<COM0A1)|(1<<COM0A0)|(1<<WGM01)|(1<<WGM00);
	TCCR0B |= (1<<CS01); //prescaler = 8 for timer 0
	OCR0A = 235; //10uS trigger pulse
	
	//Timer 1 running in normal mode
	DDRB &= ~(1<<DDB0); //PB0 as input (ICP1)
	TCCR1B = (1<<ICNC1)|(1<<ICES1)|(1<<CS11); //positive edge detection
	TIMSK1 |= (1<<ICIE1); //enable timer1 input capture interrupt
	
	
	sei(); //enable global interrupts
}

int main() {
	UART_init(BAUD_PRESCALER);
	initialize();
	while (1)
	{
		distance = getDist();
		// sprintf(String,"The distance is %lu\n", distance);
		// UART_putstring(String);
		// sprintf(String,"The distance is %u\n", continuousMode);
		// UART_putstring(String);
		
		if (continuousMode) {
			// min_dist = 2 cm (OCR2A = 29).
			// max_dist = 123 cm (OCR2A = 60).
	    	OCR2A = 0.26*distance + 28.48;

			} else {
			if (distance <= 120) {
				discrete(); // Switch to discre mode.
			}
		}
		_delay_ms(50); // delay for calculating distance.

		// Check the button state and toggle mode if it's pressed
		if (!(PIND & (1<<PIND7))) {	
				continuousMode = !continuousMode;
		}
	}

	return 0;
	
}
*/
/*-----------------------------------------------------------------------------*/


////////////////////////////////////////////////////////////////////////////////
//                   E ADC for reading photoresistor voltage                  //
////////////////////////////////////////////////////////////////////////////////
/*

*/
/*-----------------------------------------------------------------------------*/


////////////////////////////////////////////////////////////////////////////////
//                           F Putting it All Together                        //
////////////////////////////////////////////////////////////////////////////////
// /*
char String[25];

uint32_t distance;
volatile uint32_t first_pulse = 0;
volatile uint32_t second_pulse = 0;
volatile uint32_t duty_cycle = 0;
volatile int continuousMode = 0;


ISR(TIMER2_COMPB_vect)
{
	if((PINB & (1 << PORTB3))== (1>>PORTB3))
	{
		PORTB ^= (1<<PORTB5);
	}
	else
	{
		PORTB &= ~(1 << PORTB5);
	}
}

ISR(TIMER1_CAPT_vect){
	
	if ((TCCR1B & (1<<ICES1)) == (1<<ICES1))
	{
		first_pulse = ICR1;
	}
	else
	{
		second_pulse = ICR1;
	}
	if (first_pulse != 0 && second_pulse != 0)
	{
		duty_cycle = second_pulse - first_pulse;
		first_pulse = 0;
		second_pulse = 0;
	}
	TCCR1B ^= (1<<ICES1);            //toggle edge detection bit
	TIFR1 = (1<<ICF1);               //clear Input Capture Flag
}

void duty_cycle_variation()   //thresholds for volume variation according to ADC values
{
	if(ADC <= 860 && ADC >= 775)
	{
		OCR2B = 0.5*OCR2A;
	}
	else if(ADC <= 774 && ADC >= 689)
	{
		OCR2B = 0.45*OCR2A;
	}
	else if(ADC <= 688 && ADC >= 603)
	{
		OCR2B = 0.4*OCR2A;
	}
	else if(ADC <= 602 && ADC >= 517)
	{
		OCR2B = 0.35*OCR2A;
	}
	else if(ADC <= 516 && ADC >= 431)
	{
		OCR2B = 0.3*OCR2A;
	}
	else if(ADC <= 430 && ADC >= 345)
	{
		OCR2B = 0.25*OCR2A;
	}
	else if(ADC <= 344 && ADC >= 259)
	{
		OCR2B = 0.20*OCR2A;
	}
	else if(ADC <= 258 && ADC >= 173)
	{
		OCR2B = 0.15*OCR2A;
	}
	else if(ADC <= 172 && ADC >= 86)
	{
		OCR2B = 0.10*OCR2A;
	}
	else if(ADC < 86)
	{
		OCR2B = 0.05*OCR2A;
	}
}

void discrete()
{
	if(distance >= 2 && distance <= 9 )
	{
		OCR2A = 29;
		duty_cycle_variation();
	}
	if(distance >= 10 && distance <= 19 )
	{
		OCR2A = 30;
		duty_cycle_variation();
	}
	if(distance >= 20 && distance <= 29 )
	{
		OCR2A = 34;
		duty_cycle_variation();
	}
	if(distance >= 30 && distance <= 39 )
	{
		OCR2A = 38;
		duty_cycle_variation();
	}
	if(distance >= 40 && distance <= 49 )
	{
		OCR2A = 44;
		duty_cycle_variation();
	}
	if(distance >= 50 && distance <= 59 )
	{
		OCR2A = 46;
		duty_cycle_variation();
	}
	if(distance >= 60 && distance <= 69 )
	{
		OCR2A = 52;
		duty_cycle_variation();
	}
	if(distance >= 70 && distance <= 99 )
	{
		OCR2A = 59;
		duty_cycle_variation();
	}
}


uint32_t getDist(){
	
	uint32_t echo_pulse_uS;
	uint32_t distance_cm;

	//32768uS = 65536 clock ticks for Timer 1 with prescaler = 8
	echo_pulse_uS = (float)duty_cycle * 32768 / 65536;   
	distance_cm = echo_pulse_uS * 0.034 / 2; //formula for distance in cm
	return distance_cm;                     //distance in centimeters
}

void initialize(){
	
	UART_init(BAUD_PRESCALER);
		
	cli();

    // Set reference to AVCC and input channel to PA0 (ADC0)
   	ADMUX = (1 << REFS0);
    
	// Enable the ADC and set prescaler to 128 (16MHz/128 = 125kHz)
   	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
  	DIDR0 |= (1<<ADC5D);
		
	//Setup as output pin
	DDRB |= (1 << DDB3);
	DDRB |= (1<<PORTB5);
	
	//Setup as input pin
	DDRD &= ~(1<<DDD7);
	PORTD |= (1<<PORTD7);
	
	//Timer 2
	TCCR2A = (1<<COM2A0)|(1<<WGM20);  //Phase correct PWM for Timer2
	TCCR2B = (1<<CS22) | (1<<WGM22);  //prescalar 64 for timer2
	
	//Ultrasonic/
	DDRD |= (1<<DDD6);                //set PD6 as output trigger
	TCCR0A = (1<<COM0A1)|(1<<COM0A0)|(1<<WGM01)|(1<<WGM00);   //Fast PWM for Timer0
	TCCR0B |= (1<<CS01);              //prescaler = 8 for timer 0
	OCR0A = 235;                      //10uS trigger pulse
	
	TCCR1A &= ~(WGM10);               //Timer 1 running in normal mode 
	TCCR1A &= ~(WGM11);
	TCCR1B &= ~(WGM12);
	
	DDRB &= ~(1<<DDB0);                         //PB0 as input (ICP1)
	//positive edge detection and prescalar 8 for timer 1:
	TCCR1B = (1<<ICNC1)|(1<<ICES1)|(1<<CS11); 
	TIMSK1 |= (1<<ICIE1);              //enable timer1 input capture interrupt
	TIMSK2 |= (1<<OCIE2B);             //Enable output compare interrupt for timer2

	sei(); //enable global interrupts
}

int main() {
	UART_init(BAUD_PRESCALER);
	initialize();
	while (1)
	{
		ADCSRA |= (1 << ADSC);
		while (ADCSRA & (1 << ADSC));    //helps fetch ADC values
		
		distance = getDist();        //run the function to calculate distance
		
		//check if continuos mode or discrete for debugging
 		sprintf(String,"%u\n", continuousMode); 
 		UART_putstring(String);
		
		if (continuousMode) 
		{
			// Calculate the desired frequency based on the distance

			////min value of OCRA is 29 and max is 59
			//min value measured is 2 max value measured is 99
	    	OCR2A = 0.26*distance + 28.38;       	
			duty_cycle_variation();     //duty cycle will vary on the light intensity

		} 
		else 
		{
			if (distance <= 99)
			 {
				discrete();        //run the discrete mode calculations
			 }
	    }

		_delay_ms(100);  
		
		if (!(PIND & (1<<PIND7)))  // Check the button state and toggle mode if it's pressed
		{	
				continuousMode = !continuousMode;  //Toggle the mode
		}
	}
	return 0;
}
// */
/*-----------------------------------------------------------------------------*/