/*
 * Driver functions for Analog Devices ADXL345 triple-axis accelerometer.
 *
 * The ADXL345 register descriptions below are taken from the data sheet.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 3
 * or the GNU Lesser General Public License version 3, both as
 * published by the Free Software Foundation.
 */
#include "I2C.h"
#include "ADXL345.h"

/*******************************************************************************
 * Device slave address
 *
 * The SDO/Alt Adrs pin is pulled low on the breakout board, therefore the 
 * alternate address of 0x53 is used.  Left shift 1 bit: 0xa6
 */
#define ADXL345SlaveAdrs 0x53

/*******************************************************************************
 * Data Rate and Power Control
 *
 * D7 | D6 | D5 |     D4    | D3 | D2 | D1 | D0
 * 0  | 0  | 0  | LOW_POWER |       Rate
 *
 * LOW_POWER Bit
 *
 * A setting of 0 in the LOW_POWER bit selects normal operation, and a setting 
 * of 1 selects reduced power operation, which has somewhat higher noise (see 
 * the Power Modes section for details).
 *
 * Rate Bits
 *
 * These bits select the device bandwidth and output data rate (see Table 6 and 
 * Table 7 for details). The default value is 0x0A, which translates to a 100 Hz
 * output data rate. An output data rate should be selected that is appropriate 
 * for the communication protocol and frequency selected. Selecting too high of 
 * an output data rate with a low communication speed results in samples being 
 * discarded.
 *
 * Rate (Hz)| code
 *  3200    | 1111
 *  1600    | 1110
 *   800    | 1101
 *   400    | 1100
 *   200    | 1011
 *   100    | 1010
 *    50    | 1001
 *    25    | 1000
 *    12.5  | 0111
 *     6.25 | 0110
 */
#define ADXL_BW_RATE 0x2c

/*******************************************************************************
 * Power Control
 *
 * D7 | D6 |  D5  |     D4     |   D3    |  D2   | D1 | D0
 *  0 |  0 | Link | AUTO_SLEEP | Measure | Sleep | Wakeup
 *
 * Link Bit
 *
 * A setting of 1 in the link bit with both the activity and inactivity 
 * functions enabled delays the start of the activity function until inactivity 
 * is detected. After activity is detected, inactivity detection begins, 
 * preventing the detection of activity. This bit serially links the activity 
 * and inactivity functions. When this bit is set to 0, the inactivity and 
 * activity functions are concurrent. Additional information can be found in the
 * Link Mode section.
 *
 * When clearing the link bit, it is recommended that the part be placed into 
 * standby mode and then set back to measurement mode with a subsequent write. 
 * This is done to ensure that the device is properly biased if sleep mode is 
 * manually disabled; otherwise, the first few samples of data after the link 
 * bit is cleared may have additional noise, especially if the device was asleep 
 * when the bit was cleared.
 *
 * AUTO_SLEEP Bit
 *
 * If the link bit is set, a setting of 1 in the AUTO_SLEEP bit sets the ADXL345
 * to switch to sleep mode when inactivity is detected (that is, when 
 * acceleration has been below the THRESH_INACT value for at least the time 
 * indicated by TIME_INACT). A setting of 0 disables automatic switching 
 * to sleep mode. See the description of the sleep bit in this section for more 
 * information.
 *
 * When clearing the AUTO_SLEEP bit, it is recommended that the part be placed 
 * into standby mode and then set back to measurement mode with a subsequent 
 * write. This is done to ensure that the device is properly biased if sleep 
 * mode is manually disabled; otherwise, the first few samples of data after the
 * AUTO_SLEEP bit is cleared may have additional noise, especially if the device
 * was asleep when the bit was cleared.
 *
 * Measure Bit
 *
 * A setting of 0 in the measure bit places the part into standby mode, and a 
 * setting of 1 places the part into measurement mode. The ADXL345 powers up in 
 * standby mode with minimum power consumption.
 *
 * Sleep Bit
 *
 * A setting of 0 in the sleep bit puts the part into the normal mode of 
 * operation, and a setting of 1 places the part into sleep mode. Sleep mode 
 * suppresses DATA_READY, stops transmission of data to FIFO, and switches the 
 * sampling rate to one specified by the wakeup bits. In sleep mode, only the 
 * activity function can be used.
 *
 * When clearing the sleep bit, it is recommended that the part be placed into 
 * standby mode and then set back to measurement mode with a subsequent write. 
 * This is done to ensure that the device is properly biased if sleep mode is 
 * manually disabled; otherwise, the first few samples of data after the sleep 
 * bit is cleared may have additional noise, especially if the device was asleep
 * when the bit was cleared.
 *
 * Wakeup Bits
 *
 * These bits control the frequency of readings in sleep mode as described in 
 * the table below:
 *
 * Setting
 * D1 | D0 | Frequency (Hz)
 *  0    0 |     8
 *  0    1 |     4
 *  1    0 |     2
 *  1    1 |     1
 */
#define ADXL_POWER_CTL 0x2d


/*******************************************************************************
 * Data Format
 *
 *     D7     |  D6 |     D5     | D4 |    D3    |   D2    | D1 | D0
 * SELF_TEST  | SPI | INT_INVERT |  0 | FULL_RES | Justify |  Range
 *
 * The DATA_FORMAT register controls the presentation of data to Register 0x32 
 * through Register 0x37. All data, except that for the ±16 g range, must be 
 * clipped to avoid rollover.
 *
 * SELF_TEST Bit
 *
 * A setting of 1 in the SELF_TEST bit applies a self-test force to the sensor, 
 * causing a shift in the output data. A value of 0 disables the self-test 
 * force.
 *
 * SPI Bit
 *
 * A value of 1 in the SPI bit sets the device to 3-wire SPI mode, and a value 
 * of 0 sets the device to 4-wire SPI mode.
 *
 * INT_INVERT Bit
 *
 * A value of 0 in the INT_INVERT bit sets the interrupts to active high, and a 
 * value of 1 sets the interrupts to active low.
 *
 * FULL_RES Bit
 *
 * When this bit is set to a value of 1, the device is in full resolution mode, 
 * where the output resolution increases with the g range set by the range bits 
 * to maintain a 4 mg/LSB scale factor. When the FULL_RES bit is set to 0, the 
 * device is in 10-bit mode, and the range bits determine the maximum g range 
 * and scale factor.
 *
 * Justify Bit
 *
 * A setting of 1 in the justify bit selects left (MSB) justified mode, and a 
 * setting of 0 selects right justified mode with sign extension.
 *
 * Range Bits
 *
 * These bits set the g range as described in the table below:
 *
 * Setting
 * D1 | D0 | g Range
 * 0  | 0  | ±2 g
 * 0  | 1  | ±4 g
 * 1  | 0  | ±8 g
 * 1  | 1  | ±16 g
 */
#define ADXL_DATA_FORMAT 0x31

/*******************************************************************************
 * Output Registers
 *
 * These six bytes (Register 0x32 to Register 0x37) are eight bits each and hold
 * the output data for each axis. Register 0x32 and Register 0x33 hold the 
 * output data for the x-axis, Register 0x34 and Register 0x35 hold the output 
 * data for the y-axis, and Register 0x36 and Register 0x37 hold the output data
 * for the z-axis. The output data is twos complement, with DATAx0 as the least
 * significant byte and DATAx1 as the most significant byte, where x represent 
 * X, Y, or Z. The DATA_FORMAT register (Address 0x31) controls the format of 
 * the data. It is recommended that a multiple-byte read of all registers be 
 * performed to prevent a change in data between reads of sequential registers.
 */
#define ADXL_ACCEL_DATA 0x32

/*
 * adxl345_setPowerControl()
 *
 * Setup the power saving features of the accelerometer.  See the description
 * of Power Control above.
 */
void adxl345_setPowerControl(uint8_t data) {
  
  i2c_write(ADXL345SlaveAdrs, 1, ADXL_POWER_CTL, &data);

}/* end adxl345_setPowerControl() */

/*
 * adxl345_setDataFormat()
 *
 * Set the data format of the accelerometer.  See the Data Format description
 * above.
 */
void adxl345_setDataFormat(uint8_t data) {
  i2c_write(ADXL345SlaveAdrs, 1, ADXL_DATA_FORMAT, &data);
}/* end adxl345_setDataFormat() */

/*
 * adxl345_setBWRate()
 *
 * Set the ADXL345 bandwidth and data rate.  See the Data Rate and Power Control
 * description above.
 */
void adxl345_setBWRate(uint8_t data) {
  i2c_write(ADXL345SlaveAdrs, 1, ADXL_BW_RATE, &data);
}/* end  adxl345_setBWRate() */

/*
 * adxl345_getAccelData()
 *
 * Read the acceleration data from the X, Y, and Z axis into the array pointed
 * to by buf.  The calling routine must ensure that whatever buf points to is
 * big enough to hold 6 bytes.
 *
 * See the description of the ADXL_DATAX, ADXL_DATAY, or ADXL_DATAZ registers.
 */
void adxl345_getAccelData(uint8_t buf[]) {
  i2c_read(ADXL345SlaveAdrs, 6, ADXL_ACCEL_DATA, buf);
}/* end adxl345_getAccelData() */