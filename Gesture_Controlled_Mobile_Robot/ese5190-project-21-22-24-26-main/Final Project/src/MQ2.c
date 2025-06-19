#include "DHT11.h"
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include "uart.h"
#include <util/delay.h>

void initADC() {
    // Set voltage reference to AVcc (default)
    ADMUX |= (1 << REFS0);

    // Set ADC channel to 0 (corresponding to A0)
    ADMUX &= ~(1 << MUX0);
    ADMUX &= ~(1 << MUX1);
    ADMUX &= ~(1 << MUX2);
    ADMUX &= ~(1 << MUX3);
    // Enable ADC
    ADCSRA |= (1 << ADEN);
}

int analogReadADC(uint8_t channel) {
    // Set ADC channel
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    // Start ADC conversion
    ADCSRA |= (1 << ADSC);
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));
    // Read ADC result
    return ADC;
}

