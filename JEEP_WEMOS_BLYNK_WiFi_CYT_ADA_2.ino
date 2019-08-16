// JEEP_Wemos_Blynk_WiFi_Cytron_Motor_Controller_Adafruit_Motor_Controller


//#define BLYNK_DEBUG // Optional, this enables lots of prints
#define BLYNK_PRINT Serial

//****************STEERING SERVO*************************/
//Servo on GPIO 13 = Wemos Pin D7  

#include <Servo.h>
Servo steerServo;
int ServoMin = 60;
int ServoMax = 140;

//****************END STEERING SERVO-PCA8695-I2C*************************/

//****************Battery Voltage Alarm******************************/

float bitVoltage; //ADC converted voltage on batPin A0

float batteryVoltage; //computed battery voltage

int batRemPercent;

const int batPin =  A0; //GPIO - ADC0 variable that holds analog pin 0
unsigned long previousMillis = 0; //for timer reading A0
unsigned long interval = 5000; // millis between read A0

//****************END Battery Voltage Alarm******************************/

//**************** BLYNK - ESP8266 - WiFi-iPhone Hotspot ******************/
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).

char auth[] = "01fc44af30694b00af215465a120df48";//ESP8266

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "JLM_iPhone";
char pass[] = "x7ypavwb9g86b";

//define colors for Blynk interface
#define BLYNK_GREEN     "#23C48E"
#define BLYNK_BLUE      "#04C0F8"
#define BLYNK_YELLOW    "#ED9D00"
#define BLYNK_RED       "#D3435C"
#define BLYNK_DARK_BLUE "#5F7CD8"

//**************** END BLYNK - WiFi *************************/

//********************MOTOR***********************************/

#define DIR 5 //GPIO5 = Wemos D1- Cytron yellow wire
#define PWM 4 //GPIO4 = Wemos D2- Cytron white wire

int motorSpeed = 0; // Variable for motor rotation speed set by Blynk Slider
const int slideState = 0; //Variable for motor rotation speed
uint8_t switchState = 0; //Variable for Blynk segmented switch (V0)

//********************END MOTOR*******************************/

//********************WINCH Adafruit DRV8871 Motor Driver**********************/

#define winchDIR1 0 //GPIO 0 = Wemos D3 Winch Motor + (IN1)
#define winchDIR2 2 //GPIO 2 = Wemos D4 Winch Motor - (IN2)

int winchSpeed = 0; // Variable for motor rotation speed set by Blynk Slider
uint8_t winchStop = 0; // Variable for motor speed in stop condition
const int winchSlideState = 0; //for restting the slider to 0 on change
uint8_t winchState = 0; //Variable for Winch Segmented switch position

//***********************END WINCH Adafruit DRV8871 Motor Driver****************/

void setup(){
  
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass); //START BLYNK
   
  pinMode(PWM,OUTPUT); //Set Wemos D2 PIN to OUTPUT (Cytron White Wire)
   
  pinMode(DIR,OUTPUT); //Set Wemos D1 PIN to OUTPUT (Cytron Yellow Wire)
   
  pinMode(winchDIR1,OUTPUT); //Set Wemos D3 PIN to OUTPUT
  pinMode(winchDIR2,OUTPUT); //Set Wemos D4 PIN to OUTPUT
     
  pinMode (A0, INPUT); //ADC 0 Set voltage read pin A0 to input
 
  steerServo.attach(13);//Wemos Pin D7 = GPIO 13
  
//Inialize Blynk Motor Segmented Switch to Stop, Red and Slider to 0
  Blynk.setProperty(V0, "color", BLYNK_RED);
  Blynk.virtualWrite(V0, 2); 
  Blynk.syncVirtual(V0);
  Blynk.virtualWrite(V1, 0);
  Blynk.syncVirtual(V1);
  
//Inialize Blynk Winch Segmented Switch to Stop, Red and Slider to 0
  Blynk.setProperty(V4, "color", BLYNK_RED);
  Blynk.virtualWrite(V4, 2); 
  Blynk.syncVirtual(V4);
  Blynk.setProperty(V3, "color", BLYNK_GREEN);
  Blynk.virtualWrite(V3, 0);
  Blynk.syncVirtual(V3);
  
}
void loop(){
  
  Blynk.run();
  
  computeBatteryVoltage();
  
  }

//**************** V2 BLYNK Read JOYSTICK position and adjust steering Servo Position***********/

BLYNK_WRITE(V2) {
  int joystick_Pos;
  int x_position = param[0].asInt();   //uses joystick x position for steering left/right
  joystick_Pos = map(x_position, 0, 1023, ServoMin, ServoMax); //map joystick Blynk values to servo values
  steerServo.write(joystick_Pos);
  Serial.print("x_Position=  "); Serial.print(x_position); Serial.print("  joystick_Pos= "); 
  Serial.println(joystick_Pos); 
}

//********** END V2 BLYNK Read JOYSTICK position and adjust steering Servo Position***********/

//********** V1 Read Blynk SLIDER SWITCH and Convert to MOTOR SPEED Variables**********/

BLYNK_WRITE(V1) { 
  motorSpeed = param.asInt(); // assigning incoming value from pin V1 to a variable for Motor Speed (0-255)
                              // this value is used in the stateMachine function
   Serial.print("V1 Slider value is: "); Serial.println(motorSpeed);
    if (switchState == 1){
        analogWrite(PWM, motorSpeed); // Send PWM signal to motor
        digitalWrite(DIR, HIGH);
    }   
    else if (switchState == 2){
        analogWrite(PWM, 0); // Send PWM signal to motor
        digitalWrite(DIR, LOW);  
    }
    else if (switchState == 3){
        analogWrite(PWM,motorSpeed); // Send PWM signal to motor
        digitalWrite(DIR, LOW);
    }                
}

//********** END V6 Read Blynk SLIDER SWITCH and Convert to MOTOR SPEED Variables**********/

//************* V0 BLYNK THREE SEGMENT SWITCH Vehicle Direction *************/

BLYNK_WRITE(V0) {
  switchState = (param.asInt()); // Get Switch state value 1 = Forward, 2 = Neutral, 3= Reverse switch (switchState){  //From BLYNK_WRITE(V3) switchState = 1, 2 or 3 on Blynk App
   Serial.print ("Switchstate= "); Serial.println (switchState);
   
   Blynk.virtualWrite(V1, 0); 
    switch (switchState) {
    case 1:  // FORWARD       
        analogWrite(PWM, 0);// Send PWM signal to motor
        Blynk.setProperty(V0, "color", BLYNK_GREEN);
        Blynk.virtualWrite(V1, 0);
        break;
      
    case 2:  // NEUTRAL      
        analogWrite(PWM, 0);  // Set motor speed to 0
        Blynk.setProperty(V0, "color", BLYNK_RED); 
        Blynk.virtualWrite(V1, 0);
        break;
      
    case 3:  // REVERSE
        analogWrite(PWM, 0); // Send PWM signal to motor
        Blynk.setProperty(V0, "color", BLYNK_GREEN);
        Blynk.virtualWrite(V1, 0);
        break;
        }
    }

//*************END V7 BLYNK THREE SEGMENT SWITCH Vehicle Direction*************/

//********** V8 Read Blynk WINCH SLIDER SWITCH and Convert to Winch WIND Speed Variables*********/

BLYNK_WRITE(V3) { 
  winchSpeed = param.asInt(); // assigning incoming value from pin V3 to a variable for Motor Speed (0-1023)
                              // this value is used in the stateMachine function
  if (winchState == 1){
        analogWrite(winchDIR1, 0);
        analogWrite(winchDIR2, winchSpeed); // Send PWM signal to motor    
        }   
    else if (winchState == 2){
        analogWrite(winchDIR1, 0);
        analogWrite(winchDIR2, 0);
        //analogWrite(winchDIR1, winchStop);  // Set motor speed to 0  
        }
    else if (winchState == 3){
        analogWrite(winchDIR1, winchSpeed);
        analogWrite(winchDIR2, 0); // Send PWM signal to motor
        } 
  Serial.print("V3 winch Slider value is: ");
  Serial.println(winchSpeed);
    }
//********** END V8 Read Blynk WINCH SLIDER SWITCH and Convert to Winch WIND Speed Variables*********

//**************V9 Blynk Read SEGMENTED SWITCH for WINCH*******************

BLYNK_WRITE(V4) {
  
  winchState = (param.asInt()); // Get Switch state value 1 = Forward, 2 = Neutral, 3= Reverse
  Serial.print ("Winchstate= "); Serial.println (winchState);
  Blynk.virtualWrite(V3, 0);
  switch (winchState){  //From BLYNK_WRITE(V5) winchState = 1, 2 or 3 on Blynk App
    case 1:  // Unwind
        Blynk.virtualWrite(V3, 0);
        analogWrite(winchDIR1, 0);
        analogWrite(winchDIR2, 0); // Send PWM signal to motor   
        Blynk.setProperty(V4, "color", BLYNK_GREEN);    
        break;
      
    case 2:  // Stop
        Blynk.virtualWrite(V3, 0);
        analogWrite(winchDIR1, 0);
        analogWrite(winchDIR2, 0);
        //analogWrite(winchDIR1, winchStop);  // Set motor speed to 0 
         Blynk.setProperty(V4, "color", BLYNK_RED);  
        break;
      
    case 3:  // RE-Wind
        Blynk.virtualWrite(V3, 0);
        analogWrite(winchDIR2, 0);
        analogWrite(winchDIR1, 0); // Send PWM signal to motor
        Blynk.setProperty(V4, "color", BLYNK_GREEN);
        break;     
      }
    }

//*************Battery Voltage*************
  
  // The max voltage to ESP8266 A0 is 1.0V. The WEMOS has a voltage divider built-in that has 
  // R1=220 and R2=100. For 3.3 V on A0 the divider yields 1.023V. A fully charged LiPo yields 8.4V.
  // Adding a 300K+100K+100K+20K to R1 with 8.3V yields 0.99V at A0. (I use 8.3 in my calculation to get 
  // better agreement with my voltmeter and batteries) The BLYNK interface displays will change color as
  // the voltage drops. When RED it's time to recharge.
  
//********************Send Battery Voltage to BLYNK interface*********
BLYNK_READ(V5) {  //BLYNK Virtual Pin V5 - BitVoltage Meter
  if(bitVoltage > 920) {
   Blynk.setProperty(V5, "color", BLYNK_GREEN);
   Blynk.virtualWrite(V5, bitVoltage); 
  }
  else if(bitVoltage <= 920 && bitVoltage > 820 ) {
   Blynk.setProperty(V5, "color", BLYNK_YELLOW);
   Blynk.virtualWrite(V5, bitVoltage); 
  }
   else if(bitVoltage < 820 ) {
   Blynk.setProperty(V5, "color", BLYNK_RED);
   Blynk.virtualWrite(V5, bitVoltage);
  }
}

BLYNK_READ(V6) {  //BLYNK Virtual Pin V6 - Battery Meter
  if(batteryVoltage > 7.5) {
   Blynk.setProperty(V6, "color", BLYNK_GREEN);
   Blynk.virtualWrite(V6, batteryVoltage);
  }
  else if(batteryVoltage <= 7.5 && batteryVoltage >= 6.7) {
   Blynk.setProperty(V6, "color", BLYNK_YELLOW);
   Blynk.virtualWrite(V6, batteryVoltage);
  }
  else if(batteryVoltage <6.7 ) {
   Blynk.setProperty(V6, "color", BLYNK_RED);
   Blynk.virtualWrite(V6, batteryVoltage);
  }
}

BLYNK_READ(V7) {  //BLYNK Virtual Pin V7 - Battery Meter
  if(bitVoltage > 920 ) {
   Blynk.setProperty(V7, "color", BLYNK_GREEN);
   Blynk.virtualWrite(V7, batRemPercent);
  }
  else if(bitVoltage <= 920 && bitVoltage >= 820) {
   Blynk.setProperty(V7, "color", BLYNK_YELLOW);
   Blynk.virtualWrite(V7, batRemPercent);
  }
  else if(bitVoltage < 820) {
   Blynk.setProperty(V7, "color", BLYNK_RED);
   Blynk.virtualWrite(V7, batRemPercent);
 }
}
//*********************END Send Battery Voltage to BLYNK interface*********


//*************************compute Battery Voltage function***********************
  void computeBatteryVoltage(){
    
  unsigned long currentMillis = millis(); //set the current time
  if (currentMillis - previousMillis > interval) { //check to see if the interval has been met
  previousMillis = currentMillis; //reset the time
  
  bitVoltage = analogRead(A0); // read the ADC voltage on Pin A0
  batteryVoltage = 8.3 * bitVoltage / 1023; //Convert the ADC voltage to actual voltage
  batRemPercent = map(bitVoltage, 820, 1023, 0, 100); //Map ADC voltage to percent
  Serial.print ("bitVoltage= "); Serial.print (bitVoltage); Serial.print("\t"); Serial.print("batteryVoltage= "); 
  Serial.println (batteryVoltage);
  }
  }
//************************compute Battery Voltage function***********************


/********************* THANKS TO: ***********************
  -Steering Servo Tutorial inspiring the servo code - 
  https://dronebotworkshop.com/servo-motors-with-arduino/

  -Cytron MD13S Motor Controller -
  https://docs.google.com/document/d/1icu1GVDxZhUn3ADSUc3JknNcmUMdPcsnJ4MhxOPRo-I/view

  Battery Voltage Monitor - 
  https://arduinodiy.wordpress.com/2016/12/25/monitoring-lipo-battery-voltage-with-wemos-d1-minibattery-shield-and-thingspeak/
 
************************BLYNK****************************
  
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.
 *************************************************************/
