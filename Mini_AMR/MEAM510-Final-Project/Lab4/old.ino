// #include "body.h"
// #include "html510.h"
// #include <WiFi.h>

// HTML510Server h(80);

// const char* ssid = "siddharthjeyanth";
// const char* password = "hello12345";

// IPAddress Ip(192,168,1,178);

// #define PWM1_PIN 1
// #define PWM2_PIN 4
// #define DIR1_PIN 5
// #define DIRo1_PIN 0
// #define DIR2_PIN 10
// #define DIRo2_PIN 6
// #define CHANNEL_PWM1 4
// #define CHANNEL_PWM2 5
// #define FREQ 100
// #define RESOLUTION 14
// #define ENC1_PIN 18
// #define ENC2_PIN 19

// //Let motor 1 be right motor
// //Let motor 2 be left motor

// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(115200);
//   WiFi.mode(WIFI_AP); //setting wifi mode to Access Point mode
//   WiFi.softAP(ssid, password); //Access Point creates its own WiFi network with the ssid and password given as input

//   WiFi.softAPConfig(Ip, IPAddress(192, 168, 1, 2), IPAddress(255, 255, 255, 0));

//   //setting all pins for Motor 1 as OUTPUT
//   pinMode(PWM1_PIN,OUTPUT); //setting PWM1_PIN mode as output for controlling the speed of the motor 1 using PWM
//   pinMode(DIR1_PIN,OUTPUT); //setting DIR1_PIN to output mode for sending logic to the red cable end of motor 1
//   pinMode(DIRo1_PIN,OUTPUT); //setting DIRo1_PIN to output mode for sending logic to the black cable end of motor 1

//   //setting all pins for Motor 2 as OUTPUT
//   pinMode(PWM2_PIN,OUTPUT); //setting PWM2_PIN mode to output for controlling the speed of the motor 2 using PWM
//   pinMode(DIR2_PIN,OUTPUT); //setting DIR2_PIN to output mode for sending logic to the orange cable end of motor 2
//   pinMode(DIRo2_PIN,OUTPUT); //setting DIRo2_PIN to output mode for sending logic to the brown cable end of motor 2

//   pinMode(ENC1_PIN,INPUT);
//   pinMode(ENC2_PIN,INPUT);

//   //setting up the PWM1 channel and to output a frequency
//   ledcSetup(CHANNEL_PWM1,FREQ,RESOLUTION); //setting up a Channel for PWM1 that outputs a square wave of FREQ frequency with a RESOLUTION bit resolution
//   ledcAttachPin(PWM1_PIN,CHANNEL_PWM1); //setting up PWM1_PIN to output this frequency on CHANNEL_PWM1
  
//   //setting up the PWM2 channel and to output a frequency
//   ledcSetup(CHANNEL_PWM2,FREQ,RESOLUTION); //setting up a Channel for PWM2 that outputs a square wave of FREQ frequency with a RESOLUTION bit resolution
//   ledcAttachPin(PWM2_PIN,CHANNEL_PWM2); //setting up PWM2_PIN to output this frequency on CHANNEL_PWM2

//   h.begin(); //start the server and begin listening for incoming client connections
//   h.attachHandler("/ ",handleRoot);
//   //h.attachHandler("/slider1?val=",handleSlider1);
//   h.attachHandler("/slider2?val=",handleSlider2);
//   h.attachHandler("/hitF ", handleHitF);  
//   h.attachHandler("/hitR ", handleHitR);
//   h.attachHandler("/hitL ", handleHitL);
//   h.attachHandler("/hitB ", handleHitB);
//   h.attachHandler("/hitS ", handleHitS);
// }
// void handleRoot(){
//   h.sendhtml(body);
// }
// /*
// void handleSlider1(){
//   int dint = h.getVal(); //getting the value of direction (0 or 1) from the webpage
//   if(dint == 0)
//   {
//     //setting pin 5 at High and pin 4 at Low to drive backwards
//     digitalWrite(5,HIGH);
//     digitalWrite(4,LOW);
//     String s = "Direction: Backwards"; //String to indicate to the user that the direction is backwards
//     h.sendplain(s); //sends the String to the webpage
//   }
//   else if(dint == 1)
//   {
//     //setting pin 5 at Low and pin 4 at High to drive forward
//     digitalWrite(5,LOW);
//     digitalWrite(4,HIGH);
//     String s = "Direction: Forward"; //String to indicate to the user that the direction is Forward
//     h.sendplain(s); //sends the String to the webpage
//   }
// }
// */
// void handleSlider2(){
//   String s = "Speed: ";
//   int sint = h.getVal(); //getting the value of speed (0 to 100) from the webpage
//   String speedstring = String(sint); //converts the integer value of the speed to a string
//   s = s + speedstring; //concatenates 2 strings to say Speed: 
  
//   //setting the PWM1 duty cycle to control speed of motor 1
//   ledcWrite(CHANNEL_PWM1,((1<<RESOLUTION)-1)*sint/100); 
  
//   //setting the PWM2 duty cycle to control speed of motor 2
//   ledcWrite(CHANNEL_PWM2,((1<<RESOLUTION)-1)*sint/100);

//   h.sendplain(s); //sends the value of the duty cycle to the webpage
// }
// void handleHitF()
// {
//   static int toggle;
//   if(++toggle%2)
//   {
//     Serial.println("Moving forward");

//     //setting the logic of Motor 1
//     digitalWrite(DIR1_PIN,HIGH);
//     digitalWrite(DIRo1_PIN,LOW);

//     //setting the logic of Motor 2
//     digitalWrite(DIR2_PIN,LOW);
//     digitalWrite(DIRo2_PIN,HIGH);

//     h.sendplain(""); //acknowledge

//     for(int i=1; i<=50; i++)
//     {
//       Serial.print("State of encoder 1: ");
//       Serial.println(digitalRead(ENC1_PIN));
//     }
//   }
//   else
//   {
//     Serial.println("Stop forward motion");

//     //setting the logic of Motor 1
//     digitalWrite(DIR1_PIN,LOW);
//     digitalWrite(DIRo1_PIN,LOW);

//     //setting the logic of Motor 2
//     digitalWrite(DIR2_PIN,LOW);
//     digitalWrite(DIRo2_PIN,LOW);

//     h.sendplain(""); //acknowledge
//   }
// }
// void handleHitR()
// {
//   static int toggle;
//   if(++toggle%2)
//   {
//     Serial.println("Moving right");

//     //setting the logic of Motor 1
//     digitalWrite(DIR1_PIN,LOW);
//     digitalWrite(DIRo1_PIN,LOW);

//     //setting the logic of Motor 2
//     digitalWrite(DIR2_PIN,LOW);
//     digitalWrite(DIRo2_PIN,HIGH);

//     h.sendplain(""); //acknowledge

//     for(int i=1; i<=50; i++)
//     {
//       Serial.print("State of encoder 2: ");
//       Serial.println(digitalRead(ENC2_PIN));
//     }
//   }
//   else
//   {
//     Serial.println("Stop right motion");

//     //setting the logic of Motor 1
//     digitalWrite(DIR1_PIN,LOW);
//     digitalWrite(DIRo1_PIN,LOW);

//     //setting the logic of Motor 2
//     digitalWrite(DIR2_PIN,LOW);
//     digitalWrite(DIRo2_PIN,LOW);

//     h.sendplain(""); //acknowledge
//   }
// }
// void handleHitL()
// {
//   static int toggle;
//   if(++toggle%2)
//   {
//     Serial.println("Moving left");

//     //setting the logic of Motor 1
//     digitalWrite(DIR1_PIN,HIGH);
//     digitalWrite(DIRo1_PIN,LOW);

//     //setting the logic of Motor 2
//     digitalWrite(DIR2_PIN,LOW);
//     digitalWrite(DIRo2_PIN,LOW);

//     h.sendplain(""); //acknowledge
//   }
//   else
//   {
//     Serial.println("Stop left motion");

//     //setting the logic of Motor 1
//     digitalWrite(DIR1_PIN,LOW);
//     digitalWrite(DIRo1_PIN,LOW);

//     //setting the logic of Motor 2
//     digitalWrite(DIR2_PIN,LOW);
//     digitalWrite(DIRo2_PIN,LOW);

//     h.sendplain(""); //acknowledge
//   }
// }
// void handleHitB()
// {
//   static int toggle;
//   if(++toggle%2)
//   {
//     Serial.println("Moving backwards");
    
//     //setting the logic of Motor 1
//     digitalWrite(DIR1_PIN,LOW);
//     digitalWrite(DIRo1_PIN,HIGH);

//     //setting the logic of Motor 2
//     digitalWrite(DIR2_PIN,HIGH);
//     digitalWrite(DIRo2_PIN,LOW);

//     h.sendplain(""); //acknowledge
//   }
//   else
//   {
//     Serial.println("Stop backward motion");

//     //setting the logic of Motor 1
//     digitalWrite(DIR1_PIN,LOW);
//     digitalWrite(DIRo1_PIN,LOW);

//     //setting the logic of Motor 2
//     digitalWrite(DIR2_PIN,LOW);
//     digitalWrite(DIRo2_PIN,LOW);

//     h.sendplain(""); //acknowledge
//   }
// }
// void handleHitS()
// {
//   static int toggle;
//   if(++toggle%2)
//   {
//     Serial.println("Moving stop");

//     //setting the logic of Motor 1
//     digitalWrite(DIR1_PIN,LOW);
//     digitalWrite(DIRo1_PIN,LOW);

//     //setting the logic of Motor 2
//     digitalWrite(DIR2_PIN,LOW);
//     digitalWrite(DIRo2_PIN,LOW);

//     h.sendplain(""); //acknowledge
//   }
//   else
//   {
//     Serial.println("Stop stop motion");

//     //setting the logic of Motor 1
//     digitalWrite(DIR1_PIN,LOW);
//     digitalWrite(DIRo1_PIN,LOW);

//     //setting the logic of Motor 2
//     digitalWrite(DIR2_PIN,LOW);
//     digitalWrite(DIRo2_PIN,LOW);

//     h.sendplain(""); //acknowledge
//   }
// }
// void loop() {
//   // put your main code here, to run repeatedly:
//   h.serve();
//   delay(10);
// }