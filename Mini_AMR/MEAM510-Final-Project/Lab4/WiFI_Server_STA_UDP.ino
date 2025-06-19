/*
// SENDER:
#include <WiFi.h>
#include <WiFiUdp.h>

#define LEDC_CHANNEL 0 // use first channel of 6 (on C3).  
#define LEDC_RESOLUTION_BITS 14 // 2^14 = 16383.
#define LEDC_RESOLUTION  ((1<<LEDC_RESOLUTION_BITS)-1) //16383.
#define LEDC_FREQ_HZ 20 // greater than 39000/(2^14 - 1) = 2.38 Hz.
#define LED_PIN 5
#define POT_PIN 18  // Connect the potentiometer to GPIO 4.

const char* ssid = "TP-Link_E0C8"; 
const char* password = "52665134";

char udpBuffer[100]; // expect messages upto 100 characters
const int potentiometer = 0;  
WiFiUDP UDPTestServer;
IPAddress TargetIP(192, 168, 1, 129); 
IPAddress myIP (192, 168, 1, 145);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.config(myIP, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);
  UDPTestServer.begin(1235); // any UDP port# up to 65535 // but higher is safer > 1023 while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
  Serial.println("WiFi connected");
  pinMode(potentiometer, INPUT);
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL);
  Serial.begin(115200);
}

void fncUdpSend() {
  // send what ever you want upto buffer size
  // udpBuffer[0] = i & 0xff;
  // udpBuffer[0] = i >> 8;
  // udpBuffer[0] = 0;
  // udp.beginPacket(TargetIP, 2808);
  //   udp.printf("%s", udpBuffer)
  // udp.endPacket();

  Serial.println(udpBuffer);
  // send to UDPport 2808
  UDPTestServer.beginPacket(TargetIP, 1235);
    UDPTestServer.printf("%s",udpBuffer);
  UDPTestServer.endPacket();
}

void loop() {
  // put your main code here, to run repeatedly:
  // send udp packet every 4 seconds
  int potentiometer_value = analogRead(potentiometer);
  int duty_cycle = map(potentiometer_value,0,4095,0,255); // map 0-4095 <-> 0 – 255 
  Serial.println (duty_cycle);
  snprintf(udpBuffer, sizeof(udpBuffer), "%d", duty_cycle);
  fncUdpSend();
}
*/
/*--------------------------------------------------------------------*/