/* 
 *  int_Timer
 *  interrupt based timer as an example output compare. Here doing PWM
 *  (note LEDc maybe more convenient)
 */
 hw_timer_t* timer = NULL;
 uint32_t duty = 0; 
 uint32_t per = 10000; // period in uS

void IRAM_ATTR onTimer(){  
  if (digitalRead(4)) { // use GPIO 4 (add external LED)
    digitalWrite(4,LOW);   
    timerAlarmWrite(timer, duty*per/100,true);
  }
  else  {
    digitalWrite(4, HIGH);
    timerAlarmWrite(timer, (100-duty)*per/100,true);
  }
}

void setup() {
  pinMode(4, OUTPUT);                        
  timer = timerBegin(0, 80, true);             // Use timer 0 [0:3],  Set prescaler = 80.   
  timerAttachInterrupt(timer, &onTimer, true); // Attach onTimer() to our timer counting up

// Set alarm to call onTimer after 1 second (value in microseconds).   
  timerAlarmWrite(timer, per, true);      // Repeat the alarm (third parameter)  
  timerAlarmEnable(timer);        // Start an alarm 
}

void loop() {
  duty = 30;  // some code to set duty cycle
  delay(10);
}
