#include "AHS_AHT20.h"

/**************************************************************************
 * Function: aht20_init()
 *
 * Purpose: Initializes the aht20 sensor and sends necessary commands for
 * sensor to begin measuring data
 *
 * NOTE: According to thee datasheet, wait at least 80ms before sending
 * "Trigger Measurement" command
 **************************************************************************/
void aht20_init(AHS_AHT20_T *sensor)
{
    /* Snesor reading process start */
    ahs_i2c_start();
    ahs_i2c_setAddress(AHT20_SLAVE_ADDRESS, I2C_WRITE);
    ahs_i2c_write(TRIGGER_MEASUREMENT_CMD);
    ahs_i2c_write(TRIGGER_MEASUREMENT_PARAM_1);
    ahs_i2c_write(TRIGGER_MEASUREMENT_PARAM_2);
    ahs_i2c_stop();
    _delay_ms(80);

    /* Initialize default sensor values */
    sensor->humidity_data_raw = 0;
    sensor->temperature_data_raw = 0;
    sensor->state = 0;
    sensor->relative_humidity = 0;
    sensor->temperature_celsius = 0;
    sensor->temperature_fahrenheit = 0;
    sensor->CRC = 0;
}

/**************************************************************************
 * Function: aht20_update()
 *
 * Purpose: Updates all parameters in the sensor suite. Both temperature
 * and humidity are refreshed with current sensor values
 *
 * NOTE: Assume MSB when reading data from AHT20
 **************************************************************************/
void aht20_update(AHS_AHT20_T *sensor)
{
    ahs_i2c_start();
    ahs_i2c_setAddress(0x38, 1);

    /* Read byte 1 (State) */
    sensor->state = ahs_i2c_read();

    /* Read byte 2 (Humidity data [16:8]) */
    sensor->humidity_data_raw = (ahs_i2c_read());
    sensor->humidity_data_raw <<= 8; /* Allocate space for next byte */

    /* Read byte 3 (Humidity data [7:0]) */
    sensor->humidity_data_raw |= (ahs_i2c_read());

    /* Read byte 4 (Temperature [16:8]) */
    sensor->temperature_data_raw = (ahs_i2c_read() & 0x0F);
    sensor->temperature_data_raw <<= 8; /* Shift by 8 to allocate space for next byte */

    /* Read byte 5 (Temperature [7:0])*/
    sensor->temperature_data_raw |= (ahs_i2c_read());
    sensor->temperature_data_raw = sensor->temperature_data_raw & 0xFFFF;

    /*********************************************************************
     * Read byte 6
     *
     * NOTE: Due to hardware limitations, we are discarding this byte.
     * However, we still must read from the i2c bus to complete the data
     * measurement cycle
     *********************************************************************/
    ahs_i2c_read();

    /* Read byte 7 (CRC) */
    sensor->CRC = ahs_i2c_read();

    /* Send stop signal to end mesaurement cycle */
    ahs_i2c_stop();

    /*********************************************************************
     * Convert Temperature and Humidity data to real-world representation
     *  - Temperature (°Celsius)
     *  - Temperature (°Farenheit)
     *  - Relative humidity (%)
     *********************************************************************/

    /* Temperature Celsisus Data Transform */
    sensor->temperature_celsius = temperature_transform_c(sensor->temperature_data_raw);

    /* Temperature Farenheit Data Transform */
    sensor->temperature_fahrenheit   = temperature_transform_f(sensor->temperature_data_raw);

    /* Temperature Celsisus Data Transform */
    sensor->relative_humidity = relative_humidity_transform(sensor->humidity_data_raw);
}

/**************************************************************************
 * Function: temperature_transform_c
 *
 * Purpose: Converts raw sensor data to a floating point number
 * representing the current teperature in celsius
 **************************************************************************/
double temperature_transform_c(uint16_t sensor_data)
{
    return ((sensor_data / pow(2, 12)) * 200.0 - 50);
}

/**************************************************************************
 * Function: temperature_transform_f
 *
 * Purpose: Converts raw sensor data to a floating point number
 * representing the current teperature in farenheit
 **************************************************************************/
double temperature_transform_f(uint16_t sensor_data)
{
    return (temperature_transform_c(sensor_data) * 1.8 + 32);
}

/**************************************************************************
 * Function: temperature_transform_f
 *
 * Purpose: Converts raw sensor data to a floating point number
 * representing the current relative humidity
 **************************************************************************/
double relative_humidity_transform(uint16_t sensor_data)
{
    return ((1.0 * sensor_data) / pow(2, 12))*10;
}