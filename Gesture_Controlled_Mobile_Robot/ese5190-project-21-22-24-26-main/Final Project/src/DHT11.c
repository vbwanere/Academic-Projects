#include "DHT11.h"
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include "uart.h"
#include <util/delay.h>

/*---------------------------------------------------------------------------*/
/*                              Code for DHT11                               */
/* Works on Single Wire Communication Protocol developed by AOSONG Inc.      */
/*---------------------------------------------------------------------------*/
uint8_t data[5];

void decimal0(uint8_t x) {
    uint8_t y = x / 100;
    UART_send(y + '0');
    x = x - y * 100;
    y = x / 10;
    UART_send(y + '0');
    x = x - y * 10;
    UART_send(x + '0');
}

void dht11_init(void) {
    TCCR1B |= (1 << CS12) | (1 << CS10);  // 1024 prescaler for Timer1B.
    TCNT1 = 0;                            // Reset Timer1B.

    TCCR0B |= (1 << CS01);  // 8 prescaler for Timer0B.
    TCNT0 = 0;              // Reset Timer0B.

    DDRB |= (1 << DDB1);      // Set PB1 as output.
    PORTB &= ~(1 << PORTB1);  // Set PB1 low.
    TCNT1 = 0;

    while (TCNT1 <= 281);    // Wait 281 us.
    PORTB |= (1 << PORTB1);  // Set PB1 high.
    TCNT0 = 0;               // Reset Timer0B.

    while (TCNT0 <= 80);   // Wait 80 us.
    DDRB &= ~(1 << DDB1);  // Set PB1 as input.
}

int dht11_find_response() {
    if (!(PINB & (1 << PINB1))) { // If PB1 is low.
        TCNT0 = 0;
        while (TCNT0 <= 170);
    } else {
        UART_putstring("error: PIN is not low\r\n");
        // return 1;
    }

    if (PINB & (1 << PINB1)) { // If PB1 is high.
        TCNT0 = 0;
        while (TCNT0 <= 170);
    } else {
        UART_putstring("error: PIN is not high\r\n");
        // return 2;
    }
}

int dht11_receivedht(uint8_t *x) {
    uint8_t check; 
    volatile uint8_t cnt = 0;  

    for (int z = 0; z < 5; z++) { // 5 bytes of data.
        for (int j = 7; j >= 0; j--) { // 8 bits in each byte.
            TCNT0 = 0;

            while (!(PINB & (1 << PINB1))) { // If PB1 is low.
                if (TCNT0 >= 120) {
                    decimal0(TCNT0);
                    // return 12;
                }
            }
            TCNT0 = 0;

            while (PINB & (1 << PINB1)) { // If PB1 is high.
                if (TCNT0 <= 160) {
                    cnt = TCNT0;
                } else {
                    decimal0(TCNT0);
                    // return 1;
                }
            }

            if (cnt >= 40 && cnt <= 70) { // If PB1 is high for 40-70 us.
                data[z] &= ~(1 << j);  // Clear bit.
            } else if (cnt >= 120 && cnt <= 160) { // If PB1 is high for 120-160 us.
                data[z] |= (1 << j);  // Set bit.
            } else {  // If PB1 is high for 80 us.
                UART_putstring("error");
                decimal0(cnt); // Print the value of cnt.
                // return 2;
            }
        }
    }

    check = (data[0] + data[1] + data[2] + data[3]) & 0xFF; // checksum
    if (check != data[4]) {
        UART_putstring("checksum error");
        // return 3;
    }

    for (int d = 0; d < 5; d++) {
        x[d] = data[d];
    }
    // return 4;
}
/********************************************************************************/