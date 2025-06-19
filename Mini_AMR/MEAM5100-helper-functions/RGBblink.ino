/*
 * RGBblink
 * demo blinking using neopixel library
 * 
 */
void setup() {
  // No need to initialize the RGB LED
}

// the loop function runs over and over again forever
void loop() {
  neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0); // Red
  delay(1000);
  neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0); // Green
  delay(1000);
  neopixelWrite(RGB_BUILTIN,0,0,RGB_BRIGHTNESS); // Blue
  delay(1000);
  neopixelWrite(RGB_BUILTIN,0,0,0); // Off / black
  delay(1000);
}
