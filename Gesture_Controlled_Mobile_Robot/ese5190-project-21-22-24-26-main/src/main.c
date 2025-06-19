/*---------------------------------------------------------------------------*/
/* Getting the ADXL345 accelerometer readings and printing on serial moniotr.*/
/*---------------------------------------------------------------------------*/

//------------------------------- Libraries ---------------------------------//
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include "I2C.h"
#include "ADXL345.h"
#include "uart.h"
#include "DHT11.h"    
#include "MQ2.h"
#include "ST7735.h"  // LCD library.
#include "LCD_GFX.h" // LCD graphics library.
#include "AHS_I2C.h"
#include "AHS_AHT20.h"

//-------------------------------- Macros -----------------------------------//
#define F_CPU 16000000UL // CPU clock freq is independent clock prescaler.
#define BAUD_RATE 9600
#define BAUD_PRESCALER ((F_CPU / (BAUD_RATE * 16UL)) - 1)

/*---------------------------------------------------------------------------*/
/*                              Code for ADXL345                             */
/*---------------------------------------------------------------------------*/
// /*
// string formatting for the itoa() function:
#define HEX_FORMAT 16
#define DEC_FORMAT 10
#define OCT_FORMAT 8

//------------------------------- Variables ----------------------------------//
// temporary storage for raw data read from a sensor, two uint8_ts per axis:
uint8_t accelBuf[6];
// int16_t accelX, accelY, accelZ;
// float accelBuf[6];
float accelX, accelY, accelZ;
int M1A = PIND2; // Left
int M2A = PIND4; // Right
int MB = PIND7;
int x1, y1;
int x_currernt = 5;
int y_current = 60;

#define smokeA0 0
char smoke_buffer[10];
char *smoke_word = "Smoke: ";
int analogSensor = 0;
int analogSensor2 = 1;

// Temp Sensor
AHS_AHT20_T TempSensor;
char temp_buffer[16];
char *temp_word = "Temp: ";
int i = 1;


char tempStr[8]; // temp storage for a formatted string from itoa().
char String[100];

//------------------------------ Subroutines ---------------------------------//
void configure_Hardware() {
  UART_init(BAUD_PRESCALER); // Initialize the UART.

  i2c_init(400000UL); // Initialize the I2C bus.

  // Initialize and power up the accelerometer, full resolution, +/-16g:
  adxl345_setDataFormat(ADXL_DATA_FORMAT_FULL_RES | ADXL_DATA_FORMAT_RANGE_02); 
  adxl345_setBWRate(ADXL_BW_RATE_0012);  // data rate 12.5Hz
  adxl345_setPowerControl(ADXL_POWER_CTL_MEASURE);
  lcd_init(); // Initialize the screen. 
  LCD_setScreen(WHITE);
  // Motor Control: Set up the pins for the motor control.
  DDRD |= (1<<M1A); // define output
  DDRD |= (1<<M2A); // define output
  DDRD |= (1<<MB); // define output
}

void floatToString(float value, char* buffer, int decimalPlaces) {
    int intPart = (int)value;
    int fracPart = (int)((value - intPart) * 100);  // Multiply by 100 to get two decimal places
    if (fracPart < 0) {
        fracPart = -fracPart;  // Make sure the fractional part is positive
    }
    sprintf(buffer, "%d.%02d", intPart, fracPart);
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
  turn_on(M1A); //M1A ON
  turn_on(M2A); //M2A ON
  turn_off(MB); //MB OFF
  }

void backward(){ // Activate M1B and M2B
  turn_on(MB);
  // M1A and M2A don't matter
  turn_off(M1A);
  turn_off(M2A);
}

void left(){ // Only turn on M2A
  turn_on(M2A);
  turn_off(MB);
  turn_off(M1A);
}

void right(){ // Only turn M1A
  turn_on(M1A);
  turn_off(MB);
  turn_off(M2A);
}

void stop(){ // Disconnect all power
  turn_off(MB);
  turn_off(M1A);
  turn_off(M2A);
}

void print(message){
  sprintf(String, message);
  UART_putstring(String);
}

//--------------------------------- Main -------------------------------------//
int main(void) {
  configure_Hardware();
  // Timer2A_1024();  // Set up Timer2A with 1024 prescaler.

  while(1) { // read the sensor data:
    adxl345_getAccelData(accelBuf);
    accelX = (float)(((int16_t)accelBuf[1] << 8) + (int16_t)accelBuf[0]) / 28.7;

    // UART_putstring("X: ");
    // floatToString(accelX, tempStr, 2);  // 2 decimal places
    // UART_putstring(tempStr);
    // UART_putstring("\r\n");
    _delay_ms(100);

    accelY = (float)(((int16_t)accelBuf[3] << 8) + (int16_t)accelBuf[2]) / 28.7;
    // UART_putstring("Y: ");
    // floatToString(accelY, tempStr, 2);  // 2 decimal places
    // UART_putstring(tempStr);
    // UART_putstring("\r\n");
    _delay_ms(100);

    accelZ = (float)(((int16_t)accelBuf[5] << 8) + (int16_t)accelBuf[4]) / 28.7;
    // UART_putstring("Z: ");
    // floatToString(accelZ, tempStr, 2);  // 2 decimal places
    // UART_putstring(tempStr);
    // UART_putstring("\r\n");
    // UART_putstring("----------------------------------\r\n");
    // UART_putstring("\r\n");
    _delay_ms(100);

    // TCNT2 = 0;  // Reset the counter.
    if (accelX < -3 && accelY < 2 && accelY > -2) { // Forward.
      UART_putstring("Forward\r\n");
      forward();
      x1 = x_currernt + 5;  // 25 pixels per second.
      y1 = y_current + 2;
      LCD_drawBlock(x_currernt, y_current, x1, y1, BLACK);
      x_currernt = x1;
      y_current = y1;
    } else if (accelX > 3 && accelY < 2 && accelY > -2) { // Backward.
      UART_putstring("Backward\r\n");
      backward();
      x1 = x_currernt - 5;  // 25 pixels per second.
      y1 = y_current - 2;
      LCD_drawBlock(x_currernt, y_current, x1, y1, RED);
      x_currernt = x1;
      y_current = y1;
    } else if (accelY > 3 && accelX < 2 && accelX > -2) { // Right.
      UART_putstring("Right\r\n");
      right();
      x1 = x_currernt + 2;  // 25 pixels per second.
      y1 = y_current + 5;
      LCD_drawBlock(x_currernt, y_current, x1, y1, BLACK);
      x_currernt = x1;
      y_current = y1;
    } else if (accelY < -3 && accelX < 2 && accelX > -2) { // Left.
      UART_putstring("Left\r\n");
      left();
      x1 = x_currernt + 5;  // 25 pixels per second.
      y1 = y_current + 2;
      LCD_drawBlock(x_currernt, y_current, x1, y1, BLACK);
      x_currernt = x1;
      y_current = y1;
    } else { // Stop.
      stop();
      UART_putstring("Stop\r\n");
    }



    analogSensor = analogReadADC(smokeA0);

    if (analogSensor2 != analogSensor){
      smoke_buffer[0] = '\0';
      // Convert the analog sensor value to a string before transmitting
      itoa(analogSensor, smoke_buffer, 10);
      // Transmit the analog sensor value over UART
      UART_putstring("Smoke Concentration: ");
      UART_putstring(smoke_buffer);
      // strcat(smoke_buffer, smoke_buffer);
      strcat(smoke_word, smoke_buffer);
      UART_putstring(" ppm");
      UART_putstring("\r\n");

      LCD_drawString(10,10,smoke_word,BLACK,WHITE);
      smoke_word = "Smoke: ";
      
    }
    analogSensor2 = analogSensor;

  
    if(i == 1){
      // TEMP SENSOR  ======================================================
      aht20_init(&TempSensor);
      aht20_update(&TempSensor); // these two will update my object
      print("TEMP: ");
      temp_buffer[0] = '\0';
      temp_word = "Temp: ";
      dtostrf(TempSensor.temperature_fahrenheit,6,3,temp_buffer);
      print(temp_buffer);
      strcat(temp_word,temp_buffer);
      LCD_drawString(10,110,temp_word,BLACK,WHITE);
      print("\n");
      i--;
    }
    
  }
}
// */
/**********************************************************************************/


/*---------------------------------------------------------------------------*/
/*                              Code for DHT11                               */
/*---------------------------------------------------------------------------*/
/*
int main(void) {
  UART_init(BAUD_PRESCALER); // Initialize the UART.
  uint8_t x [5];
  
  while (1) {
    dht11_init();
    dht11_find_response();
    dht11_receivedht(x);

    UART_putstring("\n\r");
    UART_putstring("Humidity:\n\r");
    decimal0(x[0]);
    UART_putstring(".");
    decimal0(x[1]);
    UART_putstring("%\n\r");

    UART_putstring("Temperature:\n\r");
    decimal0(x[2]);
    UART_putstring(".");
    decimal0(x[3]);
    UART_putstring("C\n\r");

    _delay_ms(1000);

  }
}
*/
/***************************************************************************/


/*---------------------------------------------------------------------------*/
/*                               Code for MQ-2                               */
/*---------------------------------------------------------------------------*/
/*
char buffer[10];
#define smokeA0 0 // A0 corresponds to ADC channel 0

int main(void) {
  initADC();  // Initialize ADC.
  UART_init(BAUD_PRESCALER); // Initialize the UART.
  
  while (1) {
    // Read analog sensor value from ADC
    int analogSensor = analogReadADC(smokeA0);

    // Convert the analog sensor value to a string before transmitting
    itoa(analogSensor, buffer, 10);
    // Transmit the analog sensor value over UART
    UART_putstring("Smoke Concentration: ");
    UART_putstring(buffer);
    UART_putstring(" ppm");
    UART_putstring("\r\n");
    
    _delay_ms(1000);
  }
  return 0;
}
*/
/***************************************************************************/

/*---------------------------------------------------------------------------*/
/*                          DHT11 and MQ-2 together                          */
/*---------------------------------------------------------------------------*/
/*
char buffer[10];
#define smokeA0 0 // A0 corresponds to ADC channel 0

int main(void) {
    UART_init(BAUD_PRESCALER); // Initialize the UART.
    uint8_t x[5];
  
    while (1) {
        // DHT11 functionality
        dht11_init();
        dht11_find_response();
        dht11_receivedht(x);

        UART_putstring("\n\r");
        UART_putstring("Humidity:\n\r");
        decimal0(x[0]);
        UART_putstring(".");
        decimal0(x[1]);
        UART_putstring("%\n\r");

        UART_putstring("Temperature:\n\r");
        decimal0(x[2]);
        UART_putstring(".");
        decimal0(x[3]);
        UART_putstring("C\n\r");

        // Smoke sensor functionality
        initADC();  // Initialize ADC.
        // Read analog sensor value from ADC
        int analogSensor = analogReadADC(smokeA0);

        // Convert the analog sensor value to a string before transmitting
        itoa(analogSensor, buffer, 10);
        // Transmit the analog sensor value over UART
        UART_putstring("Smoke Concentration: ");
        UART_putstring(buffer);
        UART_putstring(" ppm");
        UART_putstring("\r\n");

        _delay_ms(1000);
    }

    return 0;
}
*/
/***************************************************************************/

