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

#define LF_PWM 4
#define LR_PWM 5
#define RF_PWM 6
#define RR_PWM 7
#define MP 10
#define MM 13


void setup() {

// pin out
pinMode(LF_PWM,OUTPUT);
pinMode(LR_PWM,OUTPUT);
pinMode(RF_PWM,OUTPUT);
pinMode(RR_PWM,OUTPUT);
pinMode(MP,OUTPUT);
pinMode(MM,OUTPUT);
breakUp();
digitalWrite(MP,LOW);
digitalWrite(MM,LOW);
  
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


void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt) {

        int thumbY= evt->Rz;  // right hand side joy eke uda pahala yana eka => uda=0 yata=255 mada 128
        int thumbX = evt->Z2; // right hand side joy eke left right eka left 00 right 255 mada 127
        int directoryY = evt->Z1;// left cross eke uda pahal uda 00 mada 127 pahala 255
        int directoryX =evt->Y; // left cross eke left right left=0 right 255 mada 128
  
        
          if(directoryY==0){
            // move Forward 
            moveForward(255,255);
            Serial.print("F"); 
          }else if(directoryY==255){
            // move rwverse  
            moveBack(255,255);
            Serial.print("B");
          }else if(directoryX==0){
            // move left 
            moveLeft(255,255); 
            Serial.print("L");
          }else if(directoryX==255){
            // move Right  
            moveRight(255,255);
            Serial.print("R");
          }else{
            // break 
            breakUp(); 
            Serial.print("BR");
          }
        
        Serial.println();
}

void JoystickEvents::OnButtonUp(uint8_t but_id) {
        Serial.print("Up wdown: ");
        Serial.println(but_id, DEC);
        wdown();
}

void JoystickEvents::OnButtonDn(uint8_t but_id) {
        Serial.print("Down wup: ");
        Serial.println(but_id, DEC);
        wup();
}

void breakUp(){
    digitalWrite(LF_PWM,HIGH);
    digitalWrite(LR_PWM,HIGH);
    digitalWrite(RF_PWM,HIGH);
    digitalWrite(RR_PWM,HIGH);
    delay(100);
    digitalWrite(LF_PWM,LOW);
    digitalWrite(LR_PWM,LOW);
    digitalWrite(RF_PWM,LOW);
    digitalWrite(RR_PWM,LOW);
}


void rightMotorsForward(int pwm){
    digitalWrite(RF_PWM,LOW);
    digitalWrite(RR_PWM,HIGH);
}

void rightMotorsReverse(int pwm){
    digitalWrite(RF_PWM,HIGH);
    digitalWrite(RR_PWM,LOW);
}

void leftMotorsForward(int pwm){
    digitalWrite(LF_PWM,LOW);
    digitalWrite(LR_PWM,HIGH);
}

void leftMotorsReverse(int pwm){
    digitalWrite(LF_PWM,HIGH);
    digitalWrite(LR_PWM,LOW);
}

void moveForward(int pwml,int pwmr){
  breakUp();
  rightMotorsForward(pwmr);
  leftMotorsForward(pwml);
}

void moveBack(int pwml,int pwmr){
  breakUp();
  rightMotorsReverse(pwmr);
  leftMotorsReverse(pwml);
}

void moveLeft(int pwml,int pwmr){
    breakUp();
    leftMotorsForward(pwml);
    rightMotorsReverse(pwmr);
}

void moveRight(int pwml,int pwmr){
  breakUp();
  rightMotorsForward(pwmr);
  leftMotorsReverse(pwml);
}

void wstop(){
  digitalWrite(MP,LOW);
  digitalWrite(MM,LOW);   
}

void wup(){
  
  digitalWrite(MM,HIGH);
  delay(1000);
  digitalWrite(MP,LOW);
  digitalWrite(MM,LOW);
}

void wdown(){
  digitalWrite(MP,HIGH);
  delay(1000);
  digitalWrite(MP,LOW);
  digitalWrite(MM,LOW);
}
