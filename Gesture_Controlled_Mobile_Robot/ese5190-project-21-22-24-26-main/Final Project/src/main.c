// PWM Skeleton code lol - 11/26
// Erica Santos 
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Include other libraries 
#include "uart.h"
#include "ST7735.h"
#include "LCD_GFX.h"
#include "ADXL345.h"
#include "DHT11.h"    
#include "MQ2.h"
#include "AHS_I2C.h"
#include "AHS_AHT20.h"

// UART
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
char String[100];

#define smokeA0 0
char smoke_buffer[10];
char *smoke_word = "Smoke: ";
int analogSensor = 0;

// Temp Sensor
AHS_AHT20_T TempSensor;
char temp_buffer[16];
char *temp_word = "Temp: ";

int GAS_PIN = 0;
int TEMP_PIN = 0;
int ULTRASONC_PIN = 0;
int M1A = PIND2; // Left
int M2A = PIND4; // Right
int MB = PIND7;

int ACCEL_FORWARD = 1;
int ACCEL_BACKWARD = 1;
int ACCEL_LEFT = 1;
int ACCEL_RIGHT = 1;

// put function declarations here:
void forward();
void backward();
void right();
void left();
void stop();

void accel_init(){
  // Initialize and power up the accelerometer.
  adxl345_setDataFormat(ADXL_DATA_FORMAT_FULL_RES | ADXL_DATA_FORMAT_RANGE_02); // full resolution, +/-16g
  adxl345_setBWRate(ADXL_BW_RATE_0012);  // data rate 12.5Hz
  adxl345_setPowerControl(ADXL_POWER_CTL_MEASURE);
}

void initialize() {
  cli();
  UART_init(BAUD_PRESCALER);
  i2c_init(400000UL); // Initialize the I2C bus

  // ACCELEROMETER ======================================================
  // accel_init();
  // MOTORS ======================================================
  // Voltage: 5.3V
  DDRD |= (1<<M1A); // define output
  DDRD |= (1<<M2A); // define output
  DDRD |= (1<<MB); // define output
  // GAS SENSOR ======================================================
  // TEMP SENSOR  ======================================================
  aht20_init(&TempSensor);
  aht20_update(&TempSensor); // these two will update my object
  
  // ULTRASONIC ======================================================
  /*
  DDRB |= (1<<DDB4); // set PB4 (12) as output (outputting to Trigger)
  DDRB &= ~(1<<DDB0); // set PB0 (8) as input (receive from echo)
  
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

  */
  // LCD SCREEN ======================================================
  lcd_init();
  // PWM GENERATION ======================================================

  // Define PWM pin output 
  // DDRD |= (1<<PD6); 
  DDRD |= (1<<PD5);
  // Prescale clock 0 with prescaler of 256
  TCCR0B |= (1 << CS12); //1
  TCCR0B &= ~(1 << CS11); //0
  TCCR0B &= ~(1 << CS10);  //0
  // Enable PWM Phase Correct mode
  TCCR0B &= ~(1 << WGM02); //0 (if 1 it clears on OCR0A)
  TCCR0A &= ~ (1 << WGM01); //0
  TCCR0A |= (1 << WGM00);  //1
  // toggle on compare match (with OCR0A) ; sets output pin to OC0A
  TCCR0A |= (1<<COM0A0);  // 1
  // Clear Oc0B on compare match ; sets output pin to OC0B
  TCCR0A |= (1<<COM0B1);  // 1
  // Set OCR0A to a value to create a PWM
  // OCR0A = 71; // Sets frequency (220hz, 11.5Vpp)
  OCR0B = 30;  // slow motors
  // 122Hz, 11.5Vpp, 7Vmax
  TIFR1 |= (1<<ICF1); // Set interrupt flag to 0

  sei();

}


void ultra_pulse(){
  // Initialize ranging by sending 10 us pulse to TRIG (Pin PB2)
  // Need to call it before I want to read something 
    PORTB &= ~(1<<PINB4);
    _delay_us(100);// Clear first
    
    PORTB |= (1<<PINB4);
    _delay_us(10); // 10 us pulse
    PORTB &= ~(1<<PINB4);
}

void print(message){
  sprintf(String, message);
  UART_putstring(String);
}

int main() {
  initialize();
  print("Hello World (init) \n");
  LCD_setScreen(WHITE);
  // LCD_drawCircle(30,40,20,BLUE);
  // LCD_drawString(60,70,"30",BLACK,WHITE);

  // Read analog sensor value from ADC
  analogSensor = analogReadADC(smokeA0);

  // Convert the analog sensor value to a string before transmitting
  itoa(analogSensor, smoke_buffer, 10);
  // Transmit the analog sensor value over UART
  UART_putstring("Smoke Concentration: ");
  UART_putstring(smoke_buffer);
  strcat(smoke_word,smoke_buffer);
  UART_putstring(" ppm");
  UART_putstring("\r\n");

  LCD_drawString(10,10,smoke_word,BLACK,WHITE);

  print("TEMP: ");
  dtostrf(TempSensor.temperature_fahrenheit,6,3,temp_buffer);
  print(temp_buffer);
  strcat(temp_word,temp_buffer);
  LCD_drawString(10,110,temp_word,BLACK,WHITE);
  print("\n");


  while(1){
    // ultra_pulse();



    // Read accelerometer pins 
    /*
    float accelX, accelY, accelZ;
    uint8_t accelBuf[6];
    adxl345_getAccelData(accelBuf);
    accelX = (int16_t)accelBuf[1] << 8;
    accelX += (int16_t)accelBuf[0];
    accelY = (int16_t)accelBuf[3] << 8;
    accelY += (int16_t)accelBuf[2];
    
    if(accelX > ACCEL_FORWARD){ //forward
      forward();
    } else if (accelX < ACCEL_BACKWARD) { // backward (assuming negative) 
      backward();
    } else if (accelY > ACCEL_LEFT){ // left
      left();
    } else if (accelY < ACCEL_RIGHT) { // right  (assuming negative) 
      right();
    } else { // neutral/stop
      stop();
    }
    */
  
  }

}

// put function definitions here:
void turn_on(PIN){
  PORTD |= (1<<PIN);
}

void turn_off(PIN){
  PORTD &= ~(1<<PIN);
}

// Motor Control: Changes the pin that PWM signal is output to 
void forward() { // Activate M1A and M2A
  turn_off(MB); //MB OFF

  turn_on(M1A); //M1A ON
  turn_on(M2A); //M2A ON
}

void backward(){ // Activate M1B and M2B
  turn_off(M1A);
  turn_off(M2A);

  turn_on(MB);
  // M1A and M2A don't matter
}

void left(){ // Only turn on M2A
  turn_off(MB);
  turn_off(M1A);
  
  turn_on(M2A);
}

void right(){ // Only turn M1A
  turn_off(MB);
  turn_off(M2A);
  
  turn_on(M1A);
}

void stop(){ // Disconnect all power
  turn_off(MB);
  turn_off(M1A);
  turn_off(M2A);
}


