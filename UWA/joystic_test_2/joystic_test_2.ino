#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

#include "hidjoystickrptparser.h"

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

#define LOW_SPEED 70
#define HIGH_SPEED 255
#define constant_turn_PWM 200

int leftspeed=100;
int rightspeed=100;

void setup() {

  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);

  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);

  pinMode(8,OUTPUT);
  digitalWrite(8,HIGH);
  
        Serial.begin(115200);
#if !defined(__MIPSEL__)
        while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
        Serial.println("Start");

        if (Usb.Init() == -1)
                Serial.println("OSC did not start.");

        delay(200);

        if (!Hid.SetReportParser(0, &Joy))
                ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);
}

void loop() {
  
        Usb.Task();
}

void JoystickEvents::OnHatSwitch(uint8_t hat) {
        
}

void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt) {
        
        movefunc(evt);

}

void movefuncOnHat(uint8_t hat){
      Serial.print("Hat Switch moderate with motor details: ");
        
        switch (hat){
            case 0: moveForward(leftspeed,rightspeed);break;
            case 4: motorReverse(leftspeed,rightspeed);break;
            case 6: turnLeft();break;
            case 2: turnRight();break;
            case 8: motorStop();break;
        }  
}

void movefunc(const GamePadEventData *evt){
        

        rightspeed = evt->Z2 -1;
        leftspeed = 255 - evt->Z2; 

        Serial.print(" X1 X2 butttons");
        Serial.println(evt->Y - (evt->Z1*2));
        Serial.print(" \tTurn func left ");
        Serial.print (leftspeed);  
        Serial.print(" \tTurn func Right ");
        Serial.println(rightspeed);      

        switch (evt->Y-(evt->Z1 *2)){
            case 128: moveForward(leftspeed,rightspeed);break;
            case -382: motorReverse(leftspeed,rightspeed);break;
            case -254: turnRight();break;
            case 1: turnLeft();break;
            case -126: motorStop();break;
        }  

        
}

void JoystickEvents::OnButtonUp(uint8_t but_id) {
       if(but_id == 8){
        digitalWrite(8,HIGH);
        Serial.println("Startter OFF"); 
       }
}

void JoystickEvents::OnButtonDn(uint8_t but_id) {
       if(but_id == 8){
        digitalWrite(8,LOW);
        Serial.println("Startter ON"); 
       }
}

void motorStop(){
  digitalWrite(5,LOW);
  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);
  digitalWrite(2,HIGH);
  digitalWrite(3,LOW);
  digitalWrite(4,HIGH);
}

void moveForward(int pwml,int pwmr){
  leftmotorforward(pwml);
  rightmotorforward(pwmr);
}

void rightmotorforward(int pwm){
  digitalWrite(5,LOW);
  digitalWrite(7,HIGH);
  pwm< LOW_SPEED ? analogWrite(6,LOW_SPEED): pwm > HIGH_SPEED ? analogWrite(6,HIGH_SPEED): analogWrite(6,pwm);
 
}
//
void leftmotorforward(int pwm){
  digitalWrite(2,LOW);
  digitalWrite(4,HIGH);
  pwm< LOW_SPEED ? analogWrite(3,LOW_SPEED): pwm > HIGH_SPEED ? analogWrite(3,HIGH_SPEED): analogWrite(3,pwm);
  
}
//
void rightmotorReverse(int pwm){
  digitalWrite(5,HIGH);
  digitalWrite(7,LOW);
  pwm< LOW_SPEED ? analogWrite(6,LOW_SPEED): pwm > HIGH_SPEED ? analogWrite(6,HIGH_SPEED): analogWrite(6,pwm);
 
}
//
void leftmotorReverse(int pwm){
  digitalWrite(2,HIGH);
  digitalWrite(4,LOW);
  pwm< LOW_SPEED ? analogWrite(3,LOW_SPEED): pwm > HIGH_SPEED ? analogWrite(3,HIGH_SPEED): analogWrite(3,pwm);
  
}

void motorReverse(int pwml,int pwmr){
    leftmotorReverse(pwml);
    rightmotorReverse(pwmr);
}

void turnLeft(){
  rightmotorforward(constant_turn_PWM);
  leftmotorReverse(constant_turn_PWM);
}

void turnRight(){
  leftmotorforward(constant_turn_PWM);
  rightmotorReverse(constant_turn_PWM);
}
