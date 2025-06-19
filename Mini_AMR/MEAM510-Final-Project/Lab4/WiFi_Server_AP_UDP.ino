// My IP address: 192.168.1.192 (Student 1).
//////////////////////////////////////////////////////////////////////
//         4.1.3a Controlling ESP32 via Wi-Fi-AP Mode: SENDER       //
//////////////////////////////////////////////////////////////////////
/*
#include <WiFi.h>
#include <WiFiUdp.h>


#define POT_PIN 18  // Connect the potentiometer to GPIO 4.

WiFiUDP UDPTestServer; // UDP object from WiFiUDP class.
IPAddress TargetIP(192, 168, 1, 182); // Student 2 IP address.
const char* ssid = "vbwanere";


void setup() {
  Serial.begin(115200); // initialize serial communication.
  // Setting up the Wi-Fi on my ESP32:
  WiFi.mode(WIFI_AP);
  IPAddress myIP(192, 168, 1, 192); // Student 1 IP address.
  WiFi.softAPConfig(myIP, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, "");
  UDPTestServer.begin(2808); // any UDP port# up to 65535, but higher is safer > 1023
  Serial.print("AP IP address: ");
  Serial.print(WiFi.softAPIP());
}

// Subroutine to send UDP packets:
const int sdataSize = 2;
byte sendData[sdataSize]; // two byte buffer to hold data to send.
void send_data(short int i) {
  sendData[0] = i & 0xff; // do bitwise AND to get LSB.
  sendData[1] = i >> 8; // do bitwise right shift to get MSB.

  UDPTestServer.beginPacket(TargetIP, 2808);
  UDPTestServer.write(sendData, 2);
  UDPTestServer.endPacket();

  Serial.print("Sent: ");
  Serial.println(i);
}

void loop() {
  delay(1000);
  uint32_t pot_value = analogRead(POT_PIN);  // Read the potentiometer value.
  send_data(pot_value); // Send the potentiometer value to the server.
  delay(1000);
}
*/
/*--------------------------------------------------------------------*/
