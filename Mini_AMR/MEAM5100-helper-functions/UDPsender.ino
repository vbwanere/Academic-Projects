/*
 * sends text udp packet to TargetIP
 *  using station mode (needs router) and static IP
 */
#include <WiFi.h>
#include <WiFiUdp.h>

// uncomment the router SSID and Password that is being used 

const char* ssid     = "TP-Link_FD24";
const char* password = "65512111";

//const char* ssid     = "TP-Link_05AF";
//const char* password = "47543454";

//const char* ssid     = "TP-Link_E0C8";
//const char* password = "r0botics";
//const char* password = "52665134";

WiFiUDP UDPServer;

IPAddress target(192, 168, 1, 58); // change to IP you are sending to
IPAddress myIP(192, 168, 1, 57);     // change to your IP

char udpBuffer[100];

void fncUdpSend()
{
  // send what ever you want upto buffer size                      
  UDPServer.beginPacket(target, 2808);  // send to UDPport 2808
    UDPServer.printf("%s",udpBuffer);
  UDPServer.endPacket();
  Serial.println(udpBuffer);
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);

  WiFi.config( myIP,    // device IP address
      IPAddress(192, 168, 1, 1), // gateway (not used)
      IPAddress(255, 255, 255, 0)); // netmask
  
  UDPServer.begin(2808);  // 2808 arbitrary UDP port#    
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);    Serial.print(".");
  }
  Serial.printf("WiFi connected to %s", ssid);
  Serial.print("Sending messages from "); Serial.print(myIP); 
  Serial.print(" to "); Serial.println(target);  
}

void loop() {
  strcpy(udpBuffer, "hello testing message");
  fncUdpSend();
  delay(1000);
}
