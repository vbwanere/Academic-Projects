#ifndef MQ2_H
#define MQ2_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include "uart.h"
#include <util/delay.h>

// #define A0 0 // A0 on AtMega328p corresponds to ADC channel 0

// Function prototypes
void initADC();
int analogReadADC(uint8_t channel);
#endif // MQ2_H
