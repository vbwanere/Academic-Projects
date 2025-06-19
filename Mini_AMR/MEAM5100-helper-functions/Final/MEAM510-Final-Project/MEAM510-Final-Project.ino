/* Name: MEAM510-Final-Project.ino
 * MEAM5100 Final Project
 * Team: Aditya Andy and Vaibhav
 * Instructions: Please read the file contained in doc directory.
   Uncomment the code to run the different parts of the lab.
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
*/
#include <WiFi.h>
#include <WiFiUdp.h>
#include "html510.h" // HTML library for UDP communication.
#include "vive510.h" // Vive library for HTC Vive communication.

#define BAUD_RATE 115200
/********************** Include necessary libraries above *************************/

/*------------------------------------------------------------------------*/
/*                             0.1 Blink RGB.                              */
/*------------------------------------------------------------------------*/
/*
#define RGB_BUILTIN 2

void setup() {
  // No need to initialize the RGB LED.
}
void loop() {
#ifdef RGB_BUILTIN
  neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0); // Red.
  delay(100);
  neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0); // Green.
  delay(100);
  neopixelWrite(RGB_BUILTIN,0,0,RGB_BRIGHTNESS); // Blue.
  delay(100);
#endif
}
*/
/*********************************************************************************/


/*------------------------------------------------------------------------*/
/*                      0.2 Blink RGB ESP32-WROOM.                        */
/*------------------------------------------------------------------------*/
/*
#define LED_BUILTIN 10

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(2000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(2000);                      // wait for a second
}
*/
/*********************************************************************************/


/*------------------------------------------------------------------------*/
/*             1. x-y co-ordinate detection using HTC Vive.               */
/*------------------------------------------------------------------------*/
/*
#define RGB_LED 18 // for ESP32S2 Devkit pin 18, for M5 stamp=2
#define SIGNAL_PIN1 36 // pin receiving signal from Vive circuit
#define UDP_PORT 2510 // For GTA 2022C game 
#define STUDENT_IP 204 // Andy's IP number
#define team_Number 31 
#define FREQ 1 // in Hz

const char* ssid     = "TP-Link_E0C8";
const char* password = "52665134";

Vive510 vive1(SIGNAL_PIN1);

WiFiUDP UDPTestServer;
IPAddress ipTarget(192, 168, 1, 255); // 255 => broadcast

void UdpSend(int x, int y) {
  char udpBuffer[20];
  sprintf(udpBuffer, "%02d:%4d,%4d", team_Number, x, y);                                                
  UDPTestServer.beginPacket(ipTarget, UDP_PORT);
  UDPTestServer.println(udpBuffer);
  UDPTestServer.endPacket();
  Serial.println(udpBuffer);
}
               
void setup() {
  int i = 0;
  Serial.begin(115200);

  WiFi.mode(WIFI_AP_STA);
  WiFi.config(IPAddress(192, 168, 1, STUDENT_IP), IPAddress(192, 168, 1, 1),\
              IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);

  Serial.printf("team  #%d ", team_Number); 
  Serial.print("Connecting to ");  Serial.println(ssid);
  while(WiFi.status()!=WL_CONNECTED && i++ < 20){
    delay(500);   Serial.print(".");
  }
  if (i < 19) {
    Serial.println("WiFi connected as "); Serial.print(WiFi.localIP());
  } else {
    Serial.printf("Could not connect err: %d ",i); 
  }
  UDPTestServer.begin(UDP_PORT);
  vive1.begin();
  Serial.println("Vive trackers started");
}
                                 
void loop() { 
  static long int ms = millis();
  static uint16_t x,y;

  if (millis()-ms>1000/FREQ) {
    ms = millis();
    if (WiFi.status()==WL_CONNECTED)
        neopixelWrite(RGB_LED,255,255,255);  // full white.
    UdpSend(x,y);
  }
  
  if (vive1.status() == VIVE_RECEIVING) {
    x = vive1.xCoord();
    y = vive1.yCoord();
    Serial.printf("X %d, Y %d\n", vive1.xCoord(), vive1.yCoord());
    neopixelWrite(RGB_LED, 0, x/200, y/200);  // blue to greenish.
  }
  else {
    x=0;
    y=0; 
    switch (vive1.sync(5)) {
      break;
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak).
        neopixelWrite(RGB_LED,64,32,0);  // yellowish.
      break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected.     
        neopixelWrite(RGB_LED,128,0,0);  // red.
    }
  }
  delay(20);
}
*/
/********************************************************************************/


/*------------------------------------------------------------------------*/
/*                     2. Generating a PWM using LEDC.                    */
/*------------------------------------------------------------------------*/
/*
#define LEDC_CHANNEL 0 // use first channel of 6 (on C3).
#define LEDC_RESOLUTION_BITS 14
#define LEDC_RESOLUTION  ((1 << LEDC_RESOLUTION_BITS)-1)
#define LEDC_FREQ_HZ 550 // greater than 39000/2^14 = 2.3 Hz
#define MOTOR_PIN 1 // PWM pin for motor.


void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(MOTOR_PIN, LEDC_CHANNEL);
}

// can set syntax to be like analogWrite() with input[0 : valueMax]:
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 100) {
  uint32_t duty = LEDC_RESOLUTION * min(value, valueMax) / valueMax;
  ledcWrite(channel, duty);
}

void loop() {
  ledcAnalogWrite(LEDC_CHANNEL, 50);
}
*/
/*****************************************************************************/



/*------------------------------------------------------------------------*/
/*                   3. Frequency Detection for Beacon.                   */
/*------------------------------------------------------------------------*/
/*
#define INPUT_PIN 0  // Pin to which the signal is connected
#define LEDC_CHANNEL 0 // use first channel of 6 (on C3).
#define LEDC_RESOLUTION_BITS 14
#define LEDC_RESOLUTION  ((1 << LEDC_RESOLUTION_BITS)-1)
#define MOTOR_PIN 1 // PWM pin for motor.

#define LEDC_FREQ_HZ 550 // greater than 39000/2^14 = 2.3 Hz
#define DUTY 50 // Duty cycle for PWM.

void setup() {
  Serial.begin(9600);
  pinMode(INPUT_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(MOTOR_PIN, LEDC_CHANNEL);
}

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 100) {
  uint32_t duty = LEDC_RESOLUTION * min(value, valueMax) / valueMax;
  ledcWrite(channel, duty);
}

void detectFrequency() {
  unsigned long OnTime = pulseIn(INPUT_PIN, HIGH);  // Measure on the period of the incoming signal
  Serial.print("On time: ");
  Serial.println(OnTime);
  if (OnTime > 400 && OnTime < 3000) {
    Serial.println("Detected 550 Hz");
  } else if (OnTime > 8000) {
    Serial.println("Detected 23 Hz");
  } else {
    Serial.println("Detected nothing");
  }
  // delay(100); // Wait a bit before next measurement.
}

void loop() {
  ledcAnalogWrite(LEDC_CHANNEL, DUTY); // Generate PWM.
  detectFrequency(); // Detect frequency.
  delay(1000);
}
*/
/*****************************************************************************/


/*------------------------------------------------------------------------*/
/*                 4. IR-Retroreflective Proximity Sensor                 */
/*------------------------------------------------------------------------*/
/*
// Define the pins where the sensors are connected:
#define SENSOR_PIN_1 32
#define SENSOR_PIN_2 35
#define SENSOR_PIN_3 34

// Structure to hold the status of all sensors:
struct SensorStatus {
  bool front;
  bool right;
  bool left;
};

void setup() {
  Serial.begin(115200); // Start the serial communication at 115200 baud rate.
  // Set the sensor pins as input:
  pinMode(SENSOR_PIN_1, INPUT);
  pinMode(SENSOR_PIN_2, INPUT);
  pinMode(SENSOR_PIN_3, INPUT);
}

void loop() {
  // Read the sensor values:
  SensorStatus sensor_status;
  sensor_status.front = digitalRead(SENSOR_PIN_1) == HIGH;
  sensor_status.right = digitalRead(SENSOR_PIN_2) == HIGH;
  sensor_status.left = digitalRead(SENSOR_PIN_3) == HIGH;

  // Use the sensor_status variable in an if-else conditional:
  if (sensor_status.front && sensor_status.right && sensor_status.left) {
    Serial.println("Nothing detected");
  } else if (!sensor_status.front && sensor_status.right && sensor_status.left) {
    Serial.println("Front blocked");
  } else if (sensor_status.front && !sensor_status.right && sensor_status.left) {
    Serial.println("Right blocked");
  } else if (sensor_status.front && sensor_status.right && !sensor_status.left) {
    Serial.println("Left blocked");
  } else if (!sensor_status.front && !sensor_status.right && sensor_status.left) {
    Serial.println("Front & Right blocked, take left");
  } else if (!sensor_status.front && sensor_status.right && !sensor_status.left) {
    Serial.println("Front & Left blocked, take right");
  } else if (sensor_status.front && !sensor_status.right && !sensor_status.left) {
    Serial.println("Right & Left blocked, go straight");
  } else {
    Serial.println("All blocked");
  }
  delay(500); // Wait for 500 milliseconds before reading the sensors again
}
*/
/*****************************************************************************/


/*------------------------------------------------------------------------*/
/*                           5. Wall Following                            */
/*------------------------------------------------------------------------*/
// //********************** Definitions ***********************
// //-------------------- Define Motor Pin --------------------------
// #define LEDC_CHANNEL_LEFT 1 // use the first channel for the left motor.
// #define LEDC_CHANNEL_RIGHT 0 // use the second channel for the right motor.
// #define LEDC_RESOLUTION_BITS 14
// #define LEDC_RESOLUTION  ((1 << LEDC_RESOLUTION_BITS) - 1)
// #define LEDC_FREQ_HZ 1000 // greater than 39000/2^14 = 2.3 Hz

// // Motor Right:
// #define RIGHTMOTOR_PIN 18 // PWM pin for motor.
// #define RIGHTMOTORD1_PIN 19 // Direction pin 1 for motor.
// #define RIGHTMOTORD2_PIN 21 // Direction pin 2 for motor.

// // Motor Left:
// #define LEFTMOTOR_PIN 17 // PWM pin for motor.
// #define LEFTMOTORD1_PIN 16 // Direction pin 1 for motor.
// #define LEFTMOTORD2_PIN 14 // Direction pin 2 for motor.

// #define DUTY 100 // Duty cycle in % for PWM.

// //--------------------- Define IR Pin -------------------------
// // Define the pins where the sensors are connected:
// #define SENSOR_PIN_1 35 // Front IR
// #define SENSOR_PIN_2 10 // Right IR
// #define SENSOR_PIN_3 36 // Left IR

// // Structure to hold the status of all sensors:
// struct SensorStatus {
//   bool front;
//   bool right;
//   bool left;
// };

// //********************* Setup ************************
// void setup() {
//   //--------------------- IR Setup -------------------------
//   Serial.begin(115200); // Start the serial communication at 115200 baud rate.

//   // Set the sensor pins as input
//   pinMode(SENSOR_PIN_1, INPUT);
//   pinMode(SENSOR_PIN_2, INPUT);
//   pinMode(SENSOR_PIN_3, INPUT);

//   //--------------------- Motor Setup -------------------------
//   // Left
//   pinMode(LEFTMOTOR_PIN, OUTPUT);
//   pinMode(LEFTMOTORD1_PIN, OUTPUT);
//   pinMode(LEFTMOTORD2_PIN, OUTPUT);

//   ledcSetup(LEDC_CHANNEL_LEFT, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
//   ledcAttachPin(LEFTMOTOR_PIN, LEDC_CHANNEL_LEFT);

//   //Right
//   pinMode(RIGHTMOTOR_PIN, OUTPUT);
//   pinMode(RIGHTMOTORD1_PIN, OUTPUT);
//   pinMode(RIGHTMOTORD2_PIN, OUTPUT);

//   ledcSetup(LEDC_CHANNEL_RIGHT, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
//   ledcAttachPin(RIGHTMOTOR_PIN, LEDC_CHANNEL_RIGHT);
// }

// //******************** Motor Driving Functions *************************
// //--------------------- Motor Driving Functions -------------------------
// // can set syntax to be like analogWrite() with input[ 0 : valueMax ]:
// void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 100) {
//   uint32_t duty = LEDC_RESOLUTION * min(value, valueMax) / valueMax;
//   ledcWrite(channel, duty);
// }

// void drive_forward() {
//   ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed for both
//   ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.78 * 0.5); // 0.78 correction to go straight

//   digitalWrite(LEFTMOTORD1_PIN, LOW);
//   digitalWrite(LEFTMOTORD2_PIN, HIGH);

//   digitalWrite(RIGHTMOTORD1_PIN, LOW);
//   digitalWrite(RIGHTMOTORD2_PIN, HIGH);
// }

// void drive_backward() {
//   ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed for both
//   ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.78 * 0.5); // 0.78 correction to go straight

//   digitalWrite(LEFTMOTORD1_PIN, HIGH);
//   digitalWrite(LEFTMOTORD2_PIN, LOW);

//   digitalWrite(RIGHTMOTORD1_PIN, HIGH);
//   digitalWrite(RIGHTMOTORD2_PIN, LOW);
// }

// void drive_left() {
//   ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5);
//   ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.78 * 0.5); // 0.78 correction to go straight.

//   digitalWrite(LEFTMOTORD1_PIN, HIGH);
//   digitalWrite(LEFTMOTORD2_PIN, LOW);

//   digitalWrite(RIGHTMOTORD1_PIN, LOW);
//   digitalWrite(RIGHTMOTORD2_PIN, HIGH);
// }

// void drive_right() {
//   ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed.
//   ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.78 * 0.5);

//   digitalWrite(LEFTMOTORD1_PIN, LOW);
//   digitalWrite(LEFTMOTORD2_PIN, HIGH);

//   digitalWrite(RIGHTMOTORD1_PIN, HIGH);
//   digitalWrite(RIGHTMOTORD2_PIN, LOW);
// }
// // Variable to store the time when right sensor first gets blocked:
// unsigned long rightBlockedStartTime = 0; // 


// //*********************************** Loop ************************************
// void loop() {
//   //--------------------- IR Loop -------------------------
//   // Read the sensor values
//   SensorStatus sensor_status;
//   sensor_status.front = digitalRead(SENSOR_PIN_1) == HIGH;
//   sensor_status.right = digitalRead(SENSOR_PIN_2) == HIGH;
//   sensor_status.left = digitalRead(SENSOR_PIN_3) == HIGH;

//   // For debugging the sensor readings:
//   /*
//   if (sensor_status.front && sensor_status.right && sensor_status.left) {
//     Serial.println("Nothing detected");
//     }
//     if (!sensor_status.front && sensor_status.right && sensor_status.left) {
//     Serial.println("Front blocked");
//     }
//     if (sensor_status.front && !sensor_status.right && sensor_status.left) {
//     Serial.println("Right blocked");
//     }
//     if (sensor_status.front && sensor_status.right && !sensor_status.left) {
//     Serial.println("Left blocked");
//     }
//     if (!sensor_status.front && !sensor_status.right && sensor_status.left) {
//     Serial.println("Front&Right blocked, take left");
//     }
//     if (!sensor_status.front && sensor_status.right && !sensor_status.left) {
//     Serial.println("Front&Left blocked, take right");
//     }
//     if (sensor_status.front && !sensor_status.right && !sensor_status.left) {
//     Serial.println("Right&Left blocked, go straight");
//     }

//   delay(100); // Wait for 250 milliseconds before reading the sensors again
//   */

//   //--------------------- Motor Driving Loop -------------------------
//   //TODO
//   /*
//   drive straight ahead at the beginning, keep going staright unless the following:

//     if front & right is blocked, 
//     stop and turn to the left, for am set amount of time (time it takes to rotate the vehicle
//     90 degrees) after rotation, if front is not blocked, continue straight forward, if front
//     still blocked, turn to the left for 200 ms keep checking if front is blocked, if not
//     blocked continue forward. If front blocked, turn if only front is blocked, keep turning
//     to the left until only right is blocked, when only right is blocked, go straight, if only
//     right blocked, just go straight.

//   */

//   // Check if right is continuously blocked for 8 seconds:
//   if (!sensor_status.right) {
//     if (rightBlockedStartTime == 0) { // If not already tracking time
//       rightBlockedStartTime = millis(); // Start timing
//     } else if (millis() - rightBlockedStartTime >= 5000) { // Check if 5 seconds have elapsed.
//       drive_left(); // Turn left
//       delay(600); // Turn for 0.6 seconds
//       rightBlockedStartTime = 0; // Reset the timer
//     }
//   } else {
//     rightBlockedStartTime = 0; // Reset if right is not blocked.
//   }

//   // Drive straight ahead at the beginning
//   drive_forward();

//   if (!sensor_status.front && !sensor_status.right) { // Front and right are blocked.
//     // stop_motors(); // Stop the motors.
//     drive_left(); // Start turning to the left.
//     // delay(1000); // Turn for a set amount of time to rotate 90 degrees.
    
//     // After rotation
//     while (true) {
//       if (digitalRead(SENSOR_PIN_1) == HIGH) { // If front is not blocked.
//         drive_forward(); // Continue straight forward.
//         break; // Exit the while loop.
//       } else { // If front is still blocked.
//         drive_left(); // Turn to the left for 200 ms.
//         delay(200); // Delay for 200 ms.
//       }
//     }
//   } else if (!sensor_status.front) { // Only front is blocked.
//     while (true) {
//       drive_left(); // Keep turning to the left.
//       if (digitalRead(SENSOR_PIN_1) == HIGH) { // When front is no longer blocked.
//         drive_forward(); // Go straight.
//         break; // Exit the while loop.
//       }
//     }
//   } else if (!sensor_status.right) {// Only right is blocked.
//       drive_forward(); // Just go straight.
//   }
//   delay(100); // Short delay to prevent excessive loop cycling.
// }
// /*****************************************************************************/ 

/*------------------------------------------------------------------------*/
/*                         6. Beacon Detection                            */
/*------------------------------------------------------------------------*/
// //*********************** Definitions ***********************
// //-------------------- Define Motor Pin --------------------------
// #define LEDC_CHANNEL_LEFT 1 // use the first channel for the left motor.
// #define LEDC_CHANNEL_RIGHT 0 // use the second channel for the right motor.
// #define LEDC_RESOLUTION_BITS 14
// #define LEDC_RESOLUTION  ((1 << LEDC_RESOLUTION_BITS) - 1)
// #define LEDC_FREQ_HZ 1000 // greater than 39000/2^14 = 2.3 Hz

// // Motor Right:
// #define RIGHTMOTOR_PIN 33 // PWM pin for motor.
// #define RIGHTMOTORD1_PIN 19 // Direction pin 1 for motor.
// #define RIGHTMOTORD2_PIN 21 // Direction pin 2 for motor.

// // Motor Left:
// #define LEFTMOTOR_PIN 17 // PWM pin for motor.
// #define LEFTMOTORD1_PIN 16 // Direction pin 1 for motor.
// #define LEFTMOTORD2_PIN 14 // Direction pin 2 for motor.

// #define DUTY 100 // Duty cycle in % for PWM.

// //--------------------- Define IR Pin -------------------------
// // Define the pins where the sensors are connected:
// #define SENSOR_PIN_1 35 // Front IR
// #define SENSOR_PIN_2 10 // Right IR
// #define SENSOR_PIN_3 36 // Left IR

// //--------------------- Define LED Pin -------------------------
// #define LED_PIN 18 // Replace with the correct pin number for your board

// //--------------------- Define Beacon Pin -------------------------
// #define BEACON_PIN 3  // Pin to which the signal is connected

// // Structure to hold the status of all sensors:
// struct SensorStatus {
//   bool front;
//   bool right;
//   bool left;
// };

// unsigned long rightBlockedStartTime = 0; // time when right sensor first gets blocked.
// uint16_t BeaconFreq = 0; // Frequency of the beacon.

// //******* Setup ********
// void setup() {
//   //--------------------- IR-Retro Reflective Setup -------------------------
//   Serial.begin(115200); // Start the serial communication at 115200 baud rate.

//   // Set the sensor pins as input
//   pinMode(SENSOR_PIN_1, INPUT);
//   pinMode(SENSOR_PIN_2, INPUT);
//   pinMode(SENSOR_PIN_3, INPUT);

//   //--------------------- Motor Setup -------------------------
//   // Left
//   pinMode(LEFTMOTOR_PIN, OUTPUT);
//   pinMode(LEFTMOTORD1_PIN, OUTPUT);
//   pinMode(LEFTMOTORD2_PIN, OUTPUT);

//   ledcSetup(LEDC_CHANNEL_LEFT, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
//   ledcAttachPin(LEFTMOTOR_PIN, LEDC_CHANNEL_LEFT);

//   //Right
//   pinMode(RIGHTMOTOR_PIN, OUTPUT);
//   pinMode(RIGHTMOTORD1_PIN, OUTPUT);
//   pinMode(RIGHTMOTORD2_PIN, OUTPUT);

//   ledcSetup(LEDC_CHANNEL_RIGHT, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
//   ledcAttachPin(RIGHTMOTOR_PIN, LEDC_CHANNEL_RIGHT);

//   //--------------------- Beacon Setup -------------------------
//   pinMode(BEACON_PIN, INPUT);

//   //--------------------- LED Setup -------------------------
//   pinMode(LED_PIN, OUTPUT); // Initialize the LED pin as an output
// }

// //--------------------- Motor Driving Functions -------------------------
// // can set syntax to be like analogWrite() with input[ 0 : valueMax ]:
// void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 100) {
//   uint32_t duty = LEDC_RESOLUTION * min(value, valueMax) / valueMax;
//   ledcWrite(channel, duty);
// }

// void drive_forward() {
//   ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed for both
//   ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.78 * 0.5); // 0.78 correction to go straight

//   digitalWrite(LEFTMOTORD1_PIN, LOW);
//   digitalWrite(LEFTMOTORD2_PIN, HIGH);

//   digitalWrite(RIGHTMOTORD1_PIN, LOW);
//   digitalWrite(RIGHTMOTORD2_PIN, HIGH);
// }

// void drive_backward() {
//   ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed for both
//   ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.78 * 0.5); // 0.78 correction to go straight

//   digitalWrite(LEFTMOTORD1_PIN, HIGH);
//   digitalWrite(LEFTMOTORD2_PIN, LOW);

//   digitalWrite(RIGHTMOTORD1_PIN, HIGH);
//   digitalWrite(RIGHTMOTORD2_PIN, LOW);
// }

// void drive_left() {
//   ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.6);
//   ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.78 * 0.6); // 0.78 correction to go straight.

//   digitalWrite(LEFTMOTORD1_PIN, HIGH);
//   digitalWrite(LEFTMOTORD2_PIN, LOW);

//   digitalWrite(RIGHTMOTORD1_PIN, LOW);
//   digitalWrite(RIGHTMOTORD2_PIN, HIGH);
// }

// void drive_right() {
//   ledcAnalogWrite(LEDC_CHANNEL_LEFT, DUTY * 0.5); // 0.5 to reduce speed.
//   ledcAnalogWrite(LEDC_CHANNEL_RIGHT, DUTY * 0.78 * 0.5);

//   digitalWrite(LEFTMOTORD1_PIN, LOW);
//   digitalWrite(LEFTMOTORD2_PIN, HIGH);

//   digitalWrite(RIGHTMOTORD1_PIN, HIGH);
//   digitalWrite(RIGHTMOTORD2_PIN, LOW);
// }

// void stop() {
//   ledcAnalogWrite(LEDC_CHANNEL_LEFT, 0);
//   ledcAnalogWrite(LEDC_CHANNEL_RIGHT, 0);

//   digitalWrite(LEFTMOTORD1_PIN, LOW);
//   digitalWrite(LEFTMOTORD2_PIN, HIGH);

//   digitalWrite(RIGHTMOTORD1_PIN, HIGH);
//   digitalWrite(RIGHTMOTORD2_PIN, LOW);
// }

// // Subroutine to detect the frequency of the beacon:
// uint16_t detectFrequency() {
//   unsigned long OnTime = pulseIn(BEACON_PIN, HIGH);  // Measure on the period of the incoming signal
//   if (OnTime > 400 && OnTime < 3000) {
//     Serial.print("550Hz detected\n");
//     return 550;
//   } else if (OnTime > 8000) {
//     Serial.print("23Hz detected\n");
//     return 23;
//   } else {
//     Serial.print("Nothing detected\n");
//     return 0;
//   }
// }

// // Subroutine to head to the beacon:
// void headToBeacon() {
//   if (BeaconFreq == 550 || BeaconFreq == 23) { 
//     drive_forward();
//     Serial.print("Driving forward\n");
//     delay(1000);
//     // return; // Exit the function after waiting for 6 seconds
//   } else {
//     digitalWrite(LED_PIN, HIGH); // Turn on the LED
//     drive_left();
//     delay(100);
//     stop();
//     Serial.print("Driving left\n");
//     delay(1000);
//   }
//   BeaconFreq = detectFrequency();
// }

// //************ Loop *************
// void loop() {
//   //--------------------- Beacon Loop -------------------------
//   BeaconFreq = detectFrequency();
//   while (true) {
//     headToBeacon();
//   }
//   //--------------------- IR Loop -------------------------
//   // Read the sensor values
//   // SensorStatus sensor_status;
//   // sensor_status.front = digitalRead(SENSOR_PIN_1) == HIGH;
//   // sensor_status.right = digitalRead(SENSOR_PIN_2) == HIGH;
//   // sensor_status.left = digitalRead(SENSOR_PIN_3) == HIGH;

//   // // For debugging the sensor readings:
//   // /*
//   // if (sensor_status.front && sensor_status.right && sensor_status.left) {
//   //   Serial.println("Nothing detected");
//   //   }
//   //   if (!sensor_status.front && sensor_status.right && sensor_status.left) {
//   //   Serial.println("Front blocked");
//   //   }
//   //   if (sensor_status.front && !sensor_status.right && sensor_status.left) {
//   //   Serial.println("Right blocked");
//   //   }
//   //   if (sensor_status.front && sensor_status.right && !sensor_status.left) {
//   //   Serial.println("Left blocked");
//   //   }
//   //   if (!sensor_status.front && !sensor_status.right && sensor_status.left) {
//   //   Serial.println("Front&Right blocked, take left");
//   //   }
//   //   if (!sensor_status.front && sensor_status.right && !sensor_status.left) {
//   //   Serial.println("Front&Left blocked, take right");
//   //   }
//   //   if (sensor_status.front && !sensor_status.right && !sensor_status.left) {
//   //   Serial.println("Right&Left blocked, go straight");
//   //   }

//   // delay(100); // Wait for 250 milliseconds before reading the sensors again
//   // */

//   // //--------------------- Motor Driving Loop -------------------------
//   // //TODO

//   // /*
//   // drive straight ahead at the beginning, keep going staright unless the following:

//   //   if front & right is blocked, 
//   //   stop and turn to the left, for am set amount of time (time it takes to rotate the vehicle
//   //   90 degrees) after rotation, if front is not blocked, continue straight forward, if front
//   //   still blocked, turn to the left for 200 ms keep checking if front is blocked, if not
//   //   blocked continue forward. If front blocked, turn if only front is blocked, keep turning
//   //   to the left until only right is blocked, when only right is blocked, go straight, if only
//   //   right blocked, just go straight.

//   // */

//   // // Check if right is continuously blocked for 8 seconds:
//   // if (!sensor_status.right) {
//   //   if (rightBlockedStartTime == 0) { // If not already tracking time.
//   //     rightBlockedStartTime = millis(); // Start timing.
//   //   } else if (millis() - rightBlockedStartTime >= 5000) { // Check if 5 seconds have elapsed.
//   //     drive_left(); // Turn left
//   //     delay(600); // Turn for 0.6 seconds
//   //     rightBlockedStartTime = 0; // Reset the timer
//   //   }
//   // } else {
//   //   rightBlockedStartTime = 0; // Reset if right is not blocked.
//   // }  

//   // // Drive straight ahead at the beginning
//   // drive_forward();

//   // if (!sensor_status.front && !sensor_status.right) { // Front and right are blocked.
//   //   // stop_motors(); // Stop the motors.
//   //   drive_left(); // Start turning to the left.
//   //   // delay(1000); // Turn for a set amount of time to rotate 90 degrees.
    
//   //   // After rotation
//   //   while (true) {
//   //     if (digitalRead(SENSOR_PIN_1) == HIGH) { // If front is not blocked.
//   //       drive_forward(); // Continue straight forward.
//   //       break; // Exit the while loop.
//   //     } else { // If front is still blocked.
//   //       drive_left(); // Turn to the left for 200 ms.
//   //       delay(200); // Delay for 200 ms.
//   //     }
//   //   }
//   // } else if (!sensor_status.front) { // Only front is blocked.
//   //   while (true) {
//   //     drive_left(); // Keep turning to the left.
//   //     if (digitalRead(SENSOR_PIN_1) == HIGH) { // When front is no longer blocked.
//   //       drive_forward(); // Go straight.
//   //       break; // Exit the while loop.
//   //     }
//   //   }
//   // } else if (!sensor_status.right) {// Only right is blocked.
//   //     drive_forward(); // Just go straight.
//  // }
//  // delay(100); // Short delay to prevent excessive loop cycling.
// }
// /*****************************************************************************/


/*------------------------------------------------------------------------*/
/*                               7. HTC Vive                              */
/*------------------------------------------------------------------------*/

// #define RGBLED 18 // for ESP32S2 Devkit pin 18, for M5 stamp=2.
// #define SIGNALPIN1 4 // pin receiving signal from Vive circuit.
// #define UDPPORT 2510 // For GTA 2022C game. 
// #define STUDENTIP 204 // Andy's IP number.
// #define teamNumber 31 
// #define FREQ 1 // in Hz.

// const char* ssid     = "TP-Link_E0C8";
// const char* password = "52665134";

// Vive510 vive1(SIGNALPIN1);

// WiFiUDP UDPTestServer;
// IPAddress ipTarget(192, 168, 1, 255); // 255 => broadcast.

// void UdpSend(int x, int y) {
//   char udpBuffer[20];
//   sprintf(udpBuffer, "%02d:%4d,%4d", teamNumber, x, y);                                                
//   UDPTestServer.beginPacket(ipTarget, UDPPORT);
//   UDPTestServer.println(udpBuffer);
//   UDPTestServer.endPacket();
//   Serial.println(udpBuffer);
// }
               
// void setup() {
//   int i = 0;
//   Serial.begin(115200);

//   WiFi.mode(WIFI_AP_STA);
//   WiFi.config(IPAddress(192, 168, 1, STUDENTIP), IPAddress(192, 168, 1, 1),\
//               IPAddress(255, 255, 255, 0));
//   WiFi.begin(ssid, password);

//   Serial.printf("team  #%d ", teamNumber); 
//   Serial.print("Connecting to ");
//   Serial.println(ssid);
//   while(WiFi.status()!=WL_CONNECTED && i++ < 20){
//     delay(500);   Serial.print(".");
//   }
//   if (i < 19) {
//     Serial.println("WiFi connected as "); Serial.print(WiFi.localIP());
//   } else {
//     Serial.printf("Could not connect err: %d ",i); 
//   }
//   UDPTestServer.begin(UDPPORT);
//   vive1.begin();
//   Serial.println("Vive trackers started");
// }
                                 
// void loop() {  
//   static long int ms = millis();
//   static uint16_t x, y;

//   if (millis()-ms > 1000/FREQ) {
//     ms = millis();
//     if (WiFi.status()==WL_CONNECTED)
//         neopixelWrite(RGBLED,255,255,255);  // full white
//     UdpSend(x, y);
//   }
  
//   if (vive1.status() == VIVE_RECEIVING) {
//     x = vive1.xCoord();
//     y = vive1.yCoord();
//     Serial.printf("X %d, Y %d\n", vive1.xCoord(), vive1.yCoord());
//     neopixelWrite(RGBLED, 0, x/200, y/200);  // blue to greenish
//   }
//   else {
//     x=0;
//     y=0; 
//     switch (vive1.sync(5)) {
//       break;
//       case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
//         neopixelWrite(RGBLED, 64, 32, 0);  // yellowish
//       break;
//       default:
//       case VIVE_NO_SIGNAL: // nothing detected     
//         neopixelWrite(RGBLED, 128, 0, 0);  // red
//     }
//   } 
//   delay(20);
// }
/***********************************************************************/



/*--------------------------------------------------------------------------*/
/*                      8. Getting Police Car Location                      */
/*----------------------------------------------------------------------- --*/
#define UDPPORT 2510 // port for game obj transmission
WiFiUDP UDPServer;
IPAddress myIPaddress(192, 168, 1, 204); // Andy's IP address.

// uncomment the router SSID and Password that is being used:
// const char* ssid     = "TP-Link_05AF";
// const char* password = "47543454";

const char* ssid     = "TP-Link_E0C8";
const char* password = "52665134";

//const char* ssid     = "TP-Link_FD24"; 
//const char* password = "65512111";

void handleUDPServer() {
   const int UDP_PACKET_SIZE = 14; // can be up to 65535          
   uint8_t packetBuffer[UDP_PACKET_SIZE];

   int cb = UDPServer.parsePacket(); // if there is no message cb=0
   if (cb) {
      int x, y;
      packetBuffer[13]=0; // null terminate string
      UDPServer.read(packetBuffer, UDP_PACKET_SIZE);
      x = atoi((char *)packetBuffer+3); // ##,####,#### 2nd indexed char
      y = atoi((char *)packetBuffer+8); // ##,####,#### 7th indexed char
      Serial.print("From Team: ");
      Serial.println((char *)packetBuffer);
      Serial.println(x);
      Serial.println(y);
   }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  WiFi.config(myIPaddress,        // Device IP address.
      IPAddress(192, 168, 1, 1),   // gateway (not important for 5100).
      IPAddress(255, 255, 255, 0)); // net mask. 
  
  UDPServer.begin(UDPPORT);  // 2510 for game arbitrary UDP port# need to use same one.   
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("WiFi connected to %s\n", ssid);
  Serial.print("Using static IP ");
  Serial.print(myIPaddress); 
  Serial.print("and UDP port ");
  Serial.println(UDPPORT);
}

void loop() {
  handleUDPServer();
  delay(10);
}
/*****************************************************************************/

