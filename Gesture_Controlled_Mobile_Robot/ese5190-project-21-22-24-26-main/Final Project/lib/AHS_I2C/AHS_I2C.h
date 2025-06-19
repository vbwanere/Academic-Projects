#ifndef AHS_I2C_LIB
#define AHS_I2C_LIB

#include "avr/io.h"
#include "avr/interrupt.h"

#define I2C_READ 1
#define I2C_WRITE 0

void ahs_i2c_init();

void ahs_i2c_start();

void ahs_i2c_stop();

void ahs_i2c_write(uint8_t data);

void ahs_i2c_setAddress(uint8_t addr, uint8_t read_write);

uint8_t ahs_i2c_read();
#endif