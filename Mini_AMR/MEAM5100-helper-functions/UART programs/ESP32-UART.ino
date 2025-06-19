/*
 * ESP32-UART
 *  test routine for SERIAL2 between two ESP32's
 *  tie gnd and 32 and 33 on one board to 33 and 32 on the other
 *  also example of sending bytes
 *  (doesn't work with ESP32-S2)
 *  ...
 */
//#include "driver/uart.h"
#include <HardwareSerial.h>
#define RXD2 18
#define TXD2 19

HardwareSerial Serial2(1);

// Sending/receiving binary numbers
void tx_uint16(uint16_t i) { 
  Serial2.write((uint8_t)(i>>8)); //transmit msbyte first
  Serial2.write((uint8_t) (i & 0xff));
}

uint16_t rx_uint16() {
  uint16_t i;
  i = ((uint16_t)Serial2.read()<<8) + Serial2.read();
  return i;
}


void setup() {
  Serial.begin(115200);//  Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.print("start");
}

void loop() { 
  static uint16_t i;
  while (Serial2.available()) { // read any incoming and write it to the monitor
    Serial.print((char) Serial2.read());
  }

  Serial2.printf("ESP32 write %d ",i++);
  delay(1000);
}
