/*
 Repeat timer example
 This example shows how to use hardware timer in ESP32. The timer calls onTimer
 function every second. The timer can be stopped with button attached to PIN 0
 (IO0).
 */

// Stop button is attached to onboard button PIN 3
#define BTN_STOP_ALARM    3

hw_timer_t * timer = NULL;

volatile uint32_t isrCounter = 0;

void IRAM_ATTR onTimer(){
  isrCounter++;
}

void setup() {
  Serial.begin(115200);

  // Set BTN_STOP_ALARM to input mode
  pinMode(BTN_STOP_ALARM, INPUT_PULLUP);

  Serial.println("timers start setup");
  // Use 1st timer of 2 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer. 3rd param currently must be false
  timerAttachInterrupt(timer, &onTimer, false);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (3rd parameter)
  timerAlarmWrite(timer, 1000000, true);

  // Start an alarm
  timerAlarmEnable(timer);
  Serial.println("timers setup");
}

void loop() {
  // If Timer has fired
  static int oldCounter;
  if (isrCounter != oldCounter){
    Serial.print("onTimer no. ");
    Serial.println(isrCounter);
    oldCounter = isrCounter;
  }
  // If button is pressed
  if (digitalRead(BTN_STOP_ALARM) == LOW) {
    // If timer is still running
    if (timer) {
      // Stop and free timer
      timerEnd(timer);
      timer = NULL;
      Serial.println("timerstopped");
    }
  }
  delay(1);
}
