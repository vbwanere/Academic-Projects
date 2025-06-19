// My IP address: 192.168.1.192 (Student 1).
//////////////////////////////////////////////////////////////////////
//       4.1.3a Controlling ESP32 via Wi-Fi-AP Mode : RECEIVER      //
//////////////////////////////////////////////////////////////////////
/*
#include <WiFi.h>
#include <WiFiUdp.h>

#define LEDC_CHANNEL 0 // use first channel of 6 (on C3).  
#define LEDC_RESOLUTION_BITS 14 // 2^14 = 16383.
#define LEDC_RESOLUTION  ((1<<LEDC_RESOLUTION_BITS)-1) //16383.
#define LEDC_FREQ_HZ 20 // greater than 39000/(2^14 - 1) = 2.38 Hz.
#define LED_PIN 4

WiFiUDP UDPTestServer; // UDP object from WiFiUDP class.

const char* ssid = "vbwanere_server";

void setup() {
  Serial.begin(115200); // initialize serial communication.

  // Setting up the Wi-Fi on my ESP32:
  WiFi.mode(WIFI_AP);
  IPAddress myIP(192, 168, 1, 182);
  WiFi.softAPConfig(myIP, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, "");
  UDPTestServer.begin(2808); // any UDP port# up to 65535, but higher is safer > 1023

  Serial.print("IP address: ");
  Serial.print(WiFi.softAPIP());

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
  Serial.println(parsed);
  if (parsed) { // if packet is received.
    // read the packet into receiveData buffer and store it in int i:
    UDPTestServer.read(receiveData, rdataSize);
    duty_cycle = receiveData[0] + (receiveData[1] << 8);
    Serial.print("Setting duty to: ");
    Serial.println(100*duty_cycle / 16383.0);
    ledcAnalogWrite(LEDC_CHANNEL, duty_cycle);
  }
  else {
    Serial.println("Not Received!");
  }
}

void loop() {
  delay(1000);
  receive_data();
  delay(1000);
}
*/
/*--------------------------------------------------------------------*/