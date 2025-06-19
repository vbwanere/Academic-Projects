/*
 * int_TimerButtonpress
 * Simple example of recording time when button is pressed using interrupt
 * display time that button is down for - does not do debouncing...
 */
volatile uint32_t buttonDownTime,buttonUpTime;

void IRAM_ATTR handleButtonInterrupt() {
  if (digitalRead(3))   buttonUpTime =  millis();  
  else     buttonDownTime =  millis();
}

void setup(){
  Serial.begin(115200);
  pinMode(3, INPUT_PULLUP);  // GPIO 3 has button on M4 Stamp board   
  attachInterrupt(digitalPinToInterrupt(3), handleButtonInterrupt, CHANGE);
  Serial.println("press button");
}

void loop() {
  static int oldbuttonUpTime;
  if (oldbuttonUpTime != buttonUpTime) {
    Serial.printf("Button pressed for %d ms\n",buttonUpTime-buttonDownTime);
    oldbuttonUpTime = buttonUpTime;
  }
}
