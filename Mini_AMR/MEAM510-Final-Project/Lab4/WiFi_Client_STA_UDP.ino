/*
// RECEIVER:
#define LEDC_CHANNEL 1 // use first of 6 channels
#define LEDC_RESOLUTION_BITS 13
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQ_HZ 15
#define LED_PIN 19

const char *ssid = "TP-Link_E0C8";
const char *password = "52665134";
WiFiUDP UDPTestServer;
IPAddress myIP(192, 168, 1, 145);

const int UDP_PACKET_SIZE = 100; // allow packets up to 100 bytes
byte packetBuffer[UDP_PACKET_SIZE]; // can be up to 65535

int received_potentiometer_value = 0; // Store the received value

void setup() {
  // Needs router and static IP
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.config(myIP, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);
  UDPTestServer.begin(1235); // any UDP port# up to 65535 // but higher is safer > 1023
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL);
}

void handleUDPServer() {
  int cb = UDPTestServer.parsePacket();
  if (cb) {
    int len = UDPTestServer.read(packetBuffer, UDP_PACKET_SIZE - 1);
    packetBuffer[len] = 0;
    Serial.printf("%s\n", packetBuffer);
    received_potentiometer_value = atoi((char *)packetBuffer);
    Serial.println(received_potentiometer_value);    
    ledcWrite(LEDC_CHANNEL, received_potentiometer_value);
  }
}

void loop() {
  handleUDPServer();
  delay(1);
}
*/
/*----------------------------------------------------------------------*/