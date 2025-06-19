/* Name: Lab4.ino
 * MEAM5100 Lab4
 * Author: Vaibhav Wanere (vbwanere)
 * Instructions: Please read the file contained in doc directory.
   Uncomment the code to run the different parts of the lab.
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
*/
#include <WiFi.h>
#include <WiFiUdp.h>
#include "html510.h" // HTML library for UDP communication.
//########################## Include necessary libraries above #########################//


//////////////////////////////////////////////////////////////////////
//   4.1.1 Reading the State of a Switch to Turn a LED ON and OFF   //
//////////////////////////////////////////////////////////////////////
/*
#define BUTTON_PIN 18
#define LED_PIN 19

void setup() {
  pinMode(BUTTON_PIN, INPUT);  // Set the button pin as an input
  pinMode(LED_PIN, OUTPUT);  // Set the LED pin as an output.
}

void loop() {
  int button_state = digitalRead(BUTTON_PIN);  // Read the state of the button.
  if (button_state == HIGH) {
    digitalWrite(LED_PIN, LOW);   // Turn the LED OFF.
  }
  else {
    digitalWrite(LED_PIN, HIGH);    // Turn the LED ON.
  }
}
*/
/*--------------------------------------------------------------------*/



//////////////////////////////////////////////////////////////////////
//          4.1.2 ADC to vary PWM to switch LED ON and OFF          //
//////////////////////////////////////////////////////////////////////
/*
#define LEDC_CHANNEL       0 // use first channel of 6 (on C3)  
#define LEDC_RESOLUTION_BITS 14 // 2^14 = 16383
#define LEDC_RESOLUTION  ((1 << LEDC_RESOLUTION_BITS) - 1) //16383
#define LEDC_FREQ_HZ     20 // greater than 39000/2^14 = 2.3 Hz
#define LED_PIN          4
#define POT_PIN          1  // Connect the potentiometer to GPIO 4.

// can set syntax to be like analogWrite() with input[ 0 : valueMax ]:         
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 4095) {            
  uint32_t duty =  LEDC_RESOLUTION * min(value, valueMax) / valueMax;   
  ledcWrite(channel, duty);  // write duty to LEDC. 
}

// Setup timer and attach timer to a led pin:
void setup() {
  Serial.begin(115200); // initialize serial communication.        
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL);
}

void loop() {
  uint32_t pot_value = analogRead(POT_PIN);  // Read the potentiometer value.
  ledcAnalogWrite(LEDC_CHANNEL, pot_value);  // Set the LED brightness.
  Serial.println(pot_value);
  delay(50);
}
*/
/*--------------------------------------------------------------------*/



// My IP address: 192.168.1.192 (Student 1).
//////////////////////////////////////////////////////////////////////
//             4.1.3a Controlling ESP32 via Wi-Fi-AP Mode           //
//////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------//
//                           Code for SENDER                        //
//------------------------------------------------------------------//
/*
#define POT_PIN 1  // Connect the potentiometer to GPIO 4.
#define LED_PIN 4

WiFiUDP UDPTestServer; // UDP object from WiFiUDP class.
IPAddress TargetIP(192, 168, 1, 122); // Student 2 IP address.
const char* ssid = "vbwanere_server";

void setup() {
  Serial.begin(115200); // initialize serial communication.
  // Setting up the Wi-Fi on my ESP32:
  WiFi.mode(WIFI_AP);
  IPAddress myIP(192, 168, 1, 192); // Student 1 IP address.
  WiFi.softAPConfig(myIP, myIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, "");
  UDPTestServer.begin(2808); // any UDP port# up to 65535, but higher is safer > 1023
  Serial.print("AP IP address: ");
  Serial.print(WiFi.softAPIP());

  pinMode(POT_PIN, INPUT); // Set the potentiometer pin as an input.
  pinMode(LED_PIN, OUTPUT); // Set the LED pin as an output.
}

// Subroutine to send UDP packets:
const int sdataSize = 2;
byte sendData[sdataSize]; // two byte buffer to hold data to send.
void send_data(short int duty_cycle) {
  sendData[0] = duty_cycle & 0xff; // do bitwise AND to get LSB.
  sendData[1] = duty_cycle >> 8; // do bitwise right shift to get MSB.

  UDPTestServer.beginPacket(TargetIP, 2808);
  UDPTestServer.write(sendData, 2);
  UDPTestServer.endPacket();

  Serial.print("Sent Duty Cycle: ");
  Serial.println(100*duty_cycle / 4095.0);
  digitalWrite(LED_PIN, HIGH);
  delay(350);
  digitalWrite(LED_PIN, LOW);
  delay(350);
}

void loop() {
  uint32_t pot_value = analogRead(POT_PIN);  // Read the potentiometer value.
  send_data(pot_value); // Send the potentiometer value to the server.
}
*/
//------------------------------------------------------------------//
//                           Code for RECEIVER                      //
//------------------------------------------------------------------//
/*
#define LEDC_CHANNEL 0 // use first channel of 6 (on C3).  
#define LEDC_RESOLUTION_BITS 14 // 2^14 = 16383.
#define LEDC_RESOLUTION  ((1<<LEDC_RESOLUTION_BITS)-1) //16383.
#define LEDC_FREQ_HZ 4 // greater than 39000/(2^14 - 1) = 2.38 Hz.
#define LED_PIN 4

WiFiUDP UDPTestServer; // UDP object from WiFiUDP class.
const char* ssid = "vbwanere_server";

void setup() {
  Serial.begin(115200); // initialize serial communication.

  // Setting up the Wi-Fi on my ESP32 in STA mode:
  IPAddress myIP(192, 168, 1, 122);
  WiFi.begin(ssid);
  WiFi.config(myIP, myIP, IPAddress(255, 255, 255, 0));
  UDPTestServer.begin(2808); // any UDP port# up to 65535, but higher is safer > 1023

  // Setting up pins for LED and Potentiometer:
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL);                
}

// can set syntax to be like analogWrite() with input[ 0 : valueMax ]:         
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 4095) {            
  uint32_t duty =  LEDC_RESOLUTION * min(value, valueMax) / valueMax;   
  ledcWrite(channel, duty);  // write duty to LEDC
} 

// Receive UDP packets from the server:
const int rdataSize = 2; // number of bytes to receive.
byte receiveData[rdataSize]; // buffer to hold data received.
void receive_data() {
  int duty_cycle, parsed = UDPTestServer.parsePacket();
  delay(700); // wait for packet to get parsed.
  if (parsed) { // if packet is received.
    // read the packet into receiveData buffer and store it in int i:
    UDPTestServer.read(receiveData, rdataSize);
    duty_cycle = receiveData[0] + (receiveData[1] << 8);
    Serial.print("Duty Cycle = ");
    Serial.println(100*duty_cycle / 4095.0);
    ledcAnalogWrite(LEDC_CHANNEL, duty_cycle);
  }
  else {
    Serial.println("No packet received.");
  }
}

void loop() {
  receive_data();
}
*/
/*--------------------------------------------------------------------*/



//////////////////////////////////////////////////////////////////////
//   4.1.3b Controlling ESP32 through a Webpage using AP and DHCP.  //
//////////////////////////////////////////////////////////////////////
/*
#include "LED_Control_webpage.h"

#define LEDC_CHANNEL 0 // use first channel of 6 (on C3).
#define LEDC_RESOLUTION_BITS 14
#define LEDC_RESOLUTION  ((1 << LEDC_RESOLUTION_BITS)-1)
#define LEDC_FREQ_HZ 30 // greater than 39000/2^14 = 2.3 Hz
#define LED_PIN 4

const char* ssid = "vbwanere_webpage";
const char* password = "vaibhav890";

HTML510Server h(80);

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.print(WiFi.softAPIP());

  h.begin();
  h.attachHandler("/freq?val=", handleFreq); 
  h.attachHandler("/duty?val=", handleDuty); 
  h.attachHandler("/", handleRoot);

  pinMode(LED_PIN, OUTPUT);
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL);
}

// can set syntax to be like analogWrite() with input[ 0 : valueMax ]:
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 4095) {
  uint32_t duty = LEDC_RESOLUTION * min(value, valueMax) / valueMax;
  ledcWrite(channel, duty);
}

// Subroutine to set the frequency of the PWM:
void handleFreq() {
  int freq = h.getVal();
  ledcSetup(LEDC_CHANNEL, freq, LEDC_RESOLUTION_BITS);
  Serial.println(freq);
}

// Subroutine to set the duty cycle of the PWM:
void handleDuty() {
  int duty = h.getVal();
  ledcAnalogWrite(LEDC_CHANNEL, duty);
}

void handleRoot() {
  h.sendhtml(body);
}

void loop() {
  h.serve();
  delay(50);
}
*/
/*--------------------------------------------------------------------*/



//////////////////////////////////////////////////////////////////////
//        4.1.4 Driving DC Motor through a Webpage using WiFi       //
//////////////////////////////////////////////////////////////////////
// /*
#include "Motor_Control_webpage.h" // HTML code for the webpage.

#define LEDC_CHANNEL 0 // use first channel of 6 (on C3).
#define LEDC_RESOLUTION_BITS 14
#define LEDC_RESOLUTION  ((1 << LEDC_RESOLUTION_BITS)-1)
#define LEDC_FREQ_HZ 1500 // greater than 39000/2^14 = 2.3 Hz
#define MOTOR_PIN 1 // PWM pin for motor.
#define DIR1_PIN 0 // Direction pin 1 for motor.
#define DIR2_PIN 5 // Direction pin 2 for motor.
#define ENC1_PIN 19 // Encoder pin 1 for motor.
const char* ssid = "vbwanere_webpage";
const char* password = "vaibhav890";

volatile unsigned long pulseCount = 0;

HTML510Server h(80);

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.print(WiFi.softAPIP());

  h.begin();
  h.attachHandler("/duty?val=", handleDuty);
  h.attachHandler("/changeDirection?val=", handleDIR); 
  h.attachHandler("/", handleRoot);

  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(ENC1_PIN, INPUT);  // Set the encoder pin as an input.
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(MOTOR_PIN, LEDC_CHANNEL);

  pinMode(ENC1_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1_PIN), pulseCounter, RISING);
}

// can set syntax to be like analogWrite() with input[ 0 : valueMax ]:
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 100) {
  uint32_t duty = LEDC_RESOLUTION * min(value, valueMax) / valueMax;
  ledcWrite(channel, duty);
}

// Subroutine to set the Motor speed:
void handleDuty() {
  int des_speed = h.getVal();

  ledcAnalogWrite(LEDC_CHANNEL, des_speed);
}

// Subroutine to set the Motor direction:
void handleDIR() {
  int DIR = h.getVal();
  if (DIR == 1) {
    digitalWrite(DIR1_PIN, HIGH);
    digitalWrite(DIR2_PIN, LOW);
    Serial.println("Forward");
    Serial.println(DIR);
  }
  else {
    digitalWrite(DIR2_PIN, HIGH);
    digitalWrite(DIR1_PIN, LOW);
    Serial.println("Reverse");
    Serial.println(DIR);
  }
}

void handleRoot() {
  h.sendhtml(body);
}


void loop() {
  static unsigned long lastTime = millis();
  if (millis() - lastTime >= 1000) {
    noInterrupts(); // Disable interrupts to read pulseCount safely
    unsigned long measured_speed = pulseCount;
    pulseCount = 0; // Reset counter after copying
    interrupts(); // Re-enable interrupts

    Serial.print("measured speed:");
    Serial.print(measured_speed); // Number of pulses in one second is the frequency in Hz
    lastTime += 1000; // Prepare for the next interval
  }
  h.serve();
}

// Subroutine to count the number of pulses:
void pulseCounter() {
  pulseCount++;
}
// */
/*--------------------------------------------------------------------*/
