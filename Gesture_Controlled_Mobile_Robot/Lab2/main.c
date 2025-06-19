/*
 * Lab2
 * Created: 10/5/2023
 * Author : vbwanere
 */
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include "uart.h"
// This is necessary for _delay_ms() to work properly. 
#define F_CPU 16000000UL /// CPU clock freq is independent clock prescaler
#include <util/delay.h>
#define BAUD_RATE 9600
#define BAUD_PRESCALER ((F_CPU / (BAUD_RATE * 16UL)) - 1)

/*######################## Include necessary libraries above #######################*/


////////////////////////////////////////////////////////////////////////////////
//                       Part A-1 Lighting LEDs forever                       //
////////////////////////////////////////////////////////////////////////////////
/*
int main(void) {
    // set pins 9, 10, 11, 12 as outputs:
    DDRB |= (1 << PB1);
    DDRB |= (1 << PB2);
    DDRB |= (1 << PB3);
    DDRB |= (1 << PB4); 

    while (1) { 
        // set pins 9, 10, 11, 12 to HIGH:
        PORTB |= (1 << PB1);
        PORTB |= (1 << PB2);
        PORTB |= (1 << PB3);
        PORTB |= (1 << PB4);
    } 
}
*/
/*------------------------------------------------------------------------------*/


////////////////////////////////////////////////////////////////////////////////
//                Part A-2(a) Detecting state of a push button                //
////////////////////////////////////////////////////////////////////////////////
/*
// a function to press the button to turn LEDs on:
void press_button() {
    // check if pin 7 is HIGH:
    if(PIND & (1 << PIND7)) {
        // set pins 9, 10, 11, 12 to HIGH:
        PORTB |= (1 << PORTB1);
        PORTB |= (1 << PORTB2);
        PORTB |= (1 << PORTB3);
        PORTB |= (1 << PORTB4);
    }
    else {
        // set pins 9, 10, 11, 12 to LOW:
        PORTB &= ~(1 << PORTB1);
        PORTB &= ~(1 << PORTB2);
        PORTB &= ~(1 << PORTB3);
        PORTB &= ~(1 << PORTB4);
    }
}

int main(void) {
    // set pins 9, 10, 11, 12 as outputs:
    DDRB |= (1 << PORTB1);
    DDRB |= (1 << PORTB2);
    DDRB |= (1 << PORTB3);
    DDRB |= (1 << PORTB4);

    DDRD &= ~(1 << PORTD7);    // set pin 7 as input.
    PORTD |= (1 << PORTD7);    // enable pull-up resistor on pin 7.
    
    while (1) {
        press_button();
    } 
}
*/
/*------------------------------------------------------------------------------*/


////////////////////////////////////////////////////////////////////////////////
//          Part A-2(b) Detecting state of a push button in sequence          //
////////////////////////////////////////////////////////////////////////////////
/*
// configure the hardware:

void configure_hardware() {
    // Set pins 9, 10, 11, 12 as output pins:
    DDRB |= (1 << PORTB1);
    DDRB |= (1 << PORTB2);
    DDRB |= (1 << PORTB3);
    DDRB |= (1 << PORTB4);

    DDRD &= ~(1 << PORTD7);    // set pin 7 as input.
    PORTD |= (1 << PORTD7);    // enable pull-up resistor on pin 7.
}

// a function to turn LEDs on in sequence:
uint8_t currentState = 0;    // current state of the switch.
uint8_t lastState = 0;    // last state of the switch.
uint8_t ledState = 0;    // current state of the LEDs.

void switch_LED() {
    // Read the current state of the switch:
    currentState = PIND & (1 << PIND7);

    // Check if the button state has changed since the last loop iteration
    if (currentState && !lastState) {
        // increment the ledState
        ledState = (ledState + 1) % 4;
        // delay for 50ms to avoid multiple presses
        _delay_ms(50);
    }
    lastState = currentState;    // update lastState.

    // Clear the LEDs:
    PORTB &= ~(1 << PORTB1);
    PORTB &= ~(1 << PORTB2);
    PORTB &= ~(1 << PORTB3);
    PORTB &= ~(1 << PORTB4);

    // Turn the appropriate LED on:
    switch (ledState) {
        case 0:
            PORTB |= (1 << PORTB1);
            break;
        case 1:
            PORTB |= (1 << PORTB2);
            break;
        case 2:
            PORTB |= (1 << PORTB3);
            break;
        case 3:
            PORTB |= (1 << PORTB4);
            break;
    }
    _delay_ms(10);
}

// main function:
int main(void) {
    configure_hardware();
    while (1) {
        switch_LED();
    }
    return 0;
}
*/
/*------------------------------------------------------------------------------*/


////////////////////////////////////////////////////////////////////////////////
//           Part B. Detecting state of a low asserting push button           //
////////////////////////////////////////////////////////////////////////////////
/*
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
*/
/*------------------------------------------------------------------------------*/


////////////////////////////////////////////////////////////////////////////////
//           Part C. Using Input Capture to detect the switch state           //
////////////////////////////////////////////////////////////////////////////////
/*

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
*/
/*------------------------------------------------------------------------------*/



////////////////////////////////////////////////////////////////////////////////
//        Part D. Printing Morse Code Characters using Input Capture          //
////////////////////////////////////////////////////////////////////////////////
/*

#define DOT 1875    // Minimum duration for a dot.
#define DASH 12500    // Minimum duration for a dash.
#define SPACE 25000    // Maximum duration for a space.

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
*/
/*------------------------------------------------------------------------------*/



////////////////////////////////////////////////////////////////////////////////
//        Part E. Decoding Morse Code Characters Generated from Part D        //
////////////////////////////////////////////////////////////////////////////////
/*

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
*/
/*------------------------------------------------------------------------------*/


/********************************************************************************/
/*                         Part E using State Machine                           */
/********************************************************************************/
// /*
#define MAX_BUFFER 4  // Maximum buffer size for Morse code sequence

// Define the possible states in the Morse processing state machine
enum MorseStates {WAIT_FOR_PRESS, CHECK_DURATION, DECODE};

// Structure to hold the Morse data and its associated states
typedef struct {
	unsigned int start_time;    // Time when button press starts.
	unsigned int end_time;      // Time when button press ends.
	char sequence[MAX_BUFFER];  // Buffer to hold Morse code sequence.
	volatile int index;         // Current index in the Morse sequence buffer.
	volatile char decodedChar;  // Character decoded from the Morse sequence.
	enum MorseStates state;     // Current state of the Morse processing.
} MorseData;

MorseData morseData;  // Instance of MorseData structure

const char morseTable[36][6] = {
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

// Function prototypes
void initialize(void);
void processMorse(void);
void clearBuffer(void);
void decodeMorse(void);

int main(void) {
	initialize();  // Initialize system configurations

	// Main loop to keep processing Morse input
	while (1) {
		processMorse();
	}

	return 0;
}

// Function to initialize the system configurations and variables
void initialize(void) {
	cli();  // Disable global interrupts.
	UART_init(BAUD_PRESCALER);  // Initialize UART with the computed baud rate prescaler.
	DDRB |= (1 << PB1) | (1 << PB2);  // Configure PB1 and PB2 as outputs.
	DDRB &= ~(1 << PB0);  // Configure PB0 as input.
	CLKPR |= (1 << CLKPCE);  // Allow changes to Clock Prescale Register.
	TCCR1B |= (1 << CS12) | (1 << CS10) | (1 << ICNC1) | (1 << ICES1);  // Timer/Counter1 configuration.
	TIMSK1 |= (1 << ICIE1);  // Enable Timer/Counter1 capture event interrupt.
	sei();  // Enable global interrupts.
	
	morseData.state = WAIT_FOR_PRESS;  // Set initial state to waiting for button press
	morseData.index = 0;  // Reset buffer index
	clearBuffer();  // Clear the Morse sequence buffer
}

// Interrupt service routine for Timer/Counter1 capture event
ISR(TIMER1_CAPT_vect) {
	// Check edge detection type
	if (TCCR1B & (1 << ICES1)) {
		morseData.start_time = ICR1;  // Capture start time of button press
		} else {
		morseData.end_time = ICR1;  // Capture end time of button press
		morseData.state = CHECK_DURATION;  // Set state to check the duration of the press
	}
	TCCR1B ^= (1 << ICES1);  // Toggle edge detection type
}

// Main function to process Morse input based on its current state
void processMorse(void) {
	switch (morseData.state) {
		case WAIT_FOR_PRESS:
		// Check if a significant time has elapsed to move to decode state
		if ((TCNT1 - morseData.end_time) > 45000) {
			morseData.state = DECODE;
		}
		break;
		case CHECK_DURATION:
		// Check if the press duration corresponds to a dot or dash
		// Update the Morse sequence buffer and the output LED accordingly
		if ((morseData.end_time - morseData.start_time) >= 468 && (morseData.end_time - morseData.start_time) <= 3000) {
			UART_putstring(". ");
			morseData.sequence[morseData.index++] = '.';
			PORTB |= (1 << PB1);
			_delay_ms(50);
			PORTB &= ~(1 << PB1);
		} else if ((morseData.end_time - morseData.start_time) >= 3000 && (morseData.end_time - morseData.start_time) <= 6250) {
			UART_putstring("- ");
			morseData.sequence[morseData.index++] = '-';
			PORTB |= (1 << PB2);
			_delay_ms(50);
			PORTB &= ~(1 << PB2);
		}
		morseData.state = WAIT_FOR_PRESS;  // Reset state to waiting for button press
		break;
		case DECODE:
		decodeMorse();  // Decode the Morse sequence to a character
		clearBuffer();  // Clear the Morse sequence buffer for the next input
		morseData.state = WAIT_FOR_PRESS;  // Reset state to waiting for button press
		break;
	}
}

// Function to decode the Morse sequence to its corresponding character
void decodeMorse(void) {
	for (int i = 0; i < 36; i++) {
		if (strcmp(morseData.sequence, morseTable[i]) == 0) {
			morseData.decodedChar = (i < 26) ? ('A' + i) : ('0' + (i - 26));
			UART_send(morseData.decodedChar);  // Send the decoded character over UART
			return;
		}
	}
}

// Function to clear the Morse sequence buffer
void clearBuffer(void) {
	for (int i = 0; i < MAX_BUFFER; i++) {
		morseData.sequence[i] = 0;  // Clear each element in the buffer
	}
	morseData.index = 0;  // Reset buffer index
}
// */
/*------------------------------------------------------------------------------*/