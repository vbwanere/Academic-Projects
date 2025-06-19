/*
 * int_TimerCapture
 * Simple example of recording time when button is pressed using interrupt
 */
volatile uint32_t buttonPressTime;

void IRAM_ATTR handleButtonInterrupt() {
   buttonPressTime = millis();  
}

void setup(){
  Serial.begin(115200);
  pinMode(3, INPUT_PULLUP);  // GPIO 3 has button on M4 Stamp board   
  attachInterrupt(digitalPinToInterrupt(3), handleButtonInterrupt, FALLING);
  Serial.println("press button");
}

void loop() {
  static int oldbuttonPressTime;
  if (oldbuttonPressTime != buttonPressTime) {
    Serial.printf("Button pressed at %d\n",buttonPressTime);
    oldbuttonPressTime = buttonPressTime;
  }
}
