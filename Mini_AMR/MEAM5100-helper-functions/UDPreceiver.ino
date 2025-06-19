/*
 * receives udp text packets and print packets to monitor
 *  using station mode (needs router) and static IP
 */
#include <WiFi.h>
#include <WiFiUdp.h>

// uncomment the router SSID and Password that is being used 

const char* ssid     = "TP-Link_FD24"; // AP mode
const char* password = "65512111";

//const char* ssid     = "TP-Link_05AF";
//const char* password = "47543454";

//const char* ssid     = "TP-Link_E0C8";
//const char* password = "52665134";

WiFiUDP UDPServer;

IPAddress myIPaddress(192, 168, 1, 58); // change to your IP

void handleUDPServer() {
  char packetBuffer[100];
  int cb = UDPServer.parsePacket();
  if (cb) {
    int len = UDPServer.read(packetBuffer, 100);
    packetBuffer[len]=0;
    Serial.printf ("%s\n",packetBuffer);
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);

  WiFi.config( myIPaddress,        // Device IP address
      IPAddress(192, 168, 1, 1),   // gateway (not important for 5100)
      IPAddress(255, 255, 255, 0)); // net mask 
  
  UDPServer.begin(2808);  // 2808 arbitrary UDP port# need to use same one   
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.print("Using static IP "); Serial.println(myIPaddress);
}

void loop() {
  handleUDPServer();
  delay(10);
}
