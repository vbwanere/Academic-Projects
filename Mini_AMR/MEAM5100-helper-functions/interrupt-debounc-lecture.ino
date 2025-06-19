/*
 * Debounced button task
 * modified from https://www.switchdoc.com/2018/04/esp32-tutorial-debouncing-a-button-press-using-interrupts/
 */

#define BUTTONPIN 3
#define DEBOUNCETIME 20 // in ms

volatile int numberOfButtonInterrupts = 0;
volatile bool lastState;
volatile uint32_t debounceTimeout = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Interrupt Service Routine - Keep it short!
void IRAM_ATTR handleButtonInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
   numberOfButtonInterrupts++;
   lastState = digitalRead(BUTTONPIN);
   debounceTimeout = xTaskGetTickCount(); // faster version of millis()
  portEXIT_CRITICAL_ISR(&mux);
}

void setup(){
  Serial.begin(115200);
  pinMode(BUTTONPIN, INPUT_PULLUP);  
  attachInterrupt(digitalPinToInterrupt(BUTTONPIN), handleButtonInterrupt, FALLING|RISING);
}
void loop() {
  static uint32_t saveDebounceTimeout;
  static bool saveLastState;
  static int save;

  portENTER_CRITICAL_ISR(&mux);
   save  = numberOfButtonInterrupts;
   saveDebounceTimeout = debounceTimeout;
   saveLastState  = lastState;
  portEXIT_CRITICAL_ISR(&mux);
  bool currentState = digitalRead(BUTTONPIN);

  if ((save != 0) //interrupt has triggered                                                                   
    && (currentState == saveLastState) // pin state same as last interrupt                                  
    && (millis() - saveDebounceTimeout > DEBOUNCETIME )) 
  { //  low for at least DEBOUNCETIME,            
    if (currentState == LOW)  Serial.printf("Button is pressed\n");
    else   Serial.printf("Button is RELEASED\n");
    Serial.printf("Triggered %d times\n", save);
    portENTER_CRITICAL_ISR(&mux); // can't change it unless, atomic - Critical section                       
     numberOfButtonInterrupts = 0; // acknowledge keypress and reset interrupt counter                       
    portEXIT_CRITICAL_ISR(&mux);
    delay(1);
  }
  delay(1);
}
