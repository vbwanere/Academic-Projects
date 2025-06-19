/*
 * WebLED 
 * simple demonstration of TCP turn rgbLED red and green
 * uses AP and DHCP
 * 
 */
#include <WiFi.h> 
#include "html510.h" // HTML library for UDP communication.

// HTML code for the webpage:
const char body[] PROGMEM = R"===(
 <!DOCTYPE html>  
 <html><body>        
 <h1> Change LED Colour <br>
 <a href="/R"> RED</a>.<br>
 <a href="/G"> GREEN</a>.<br>
 </h1>
 </body></html>  
)===";


const char* ssid = "webLED";  // name of the network on ESP32.
const char* password = "vaibhav890"; 

HTML510Server h(80); // create a server at port 80 (from HTML510.h).

void setup() {
  Serial.begin(115200);                              
  WiFi.softAP(ssid, "vaibhav890"); // start the access point.
  Serial.print("AP IP address: HTML//");  
  Serial.print(WiFi.softAPIP());

  h.begin();
  h.attachHandler("/R", handleR);
  h.attachHandler("/G", handleG);
  h.attachHandler("/ ", handleRoot);
}

void handleR() {
  neopixelWrite(2,64,0,0); // Red
  h.sendhtml(body);
}
void handleG() {
  neopixelWrite(2,0,64,0); // Green
  h.sendhtml(body);
}
void handleRoot() {
  h.sendhtml(body);
}


void loop() {
  h.serve();
  delay(10);
}
