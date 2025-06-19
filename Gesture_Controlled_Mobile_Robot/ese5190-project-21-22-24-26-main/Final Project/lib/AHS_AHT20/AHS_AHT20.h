#ifndef AHS_AHT20_H
#define AHS_AHT20_H

#include <avr/io.h>
#include "AHS_I2C.h"
#include <avr/delay.h>

#define AHT20_SLAVE_ADDRESS 0x38
#define TRIGGER_MEASUREMENT_CMD 0xAC
#define TRIGGER_MEASUREMENT_PARAM_1 0x33
#define TRIGGER_MEASUREMENT_PARAM_2 0x00

typedef struct AHS_AHT20
{
  uint16_t humidity_data_raw;
  uint16_t temperature_data_raw;
  uint8_t state;
  double relative_humidity;
  double temperature_celsius;
  double temperature_fahrenheit;
  uint8_t CRC;
} AHS_AHT20_T;

void aht20_init(AHS_AHT20_T *sensor);
void aht20_update(AHS_AHT20_T *sensor);
double temperature_transform_c(uint16_t sensor_data);
double temperature_transform_f(uint16_t sensor_data);
double relative_humidity_transform(uint16_t sensor_data);

#endif