/*
 * LCD RS pin to digital pin 1234
 * LCD Enable pin to digital pin 1136
 * LCD D4 pin to digital pin 538
 * LCD D5 pin to digital pin 440
 * LCD D6 pin to digital pin 342
 * LCD D7 pin to digital pin 244
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 */

#include <LiquidCrystal.h>

int lEncoderCount;
int rEncoderCount;

#define ENCORDER_UNIT 0.315

LiquidCrystal lcd(34, 36, 38, 40, 42, 44);

void setup() {
  // motor Pins
  for(int i=0;i<6;i++){
    pinMode(2+i,OUTPUT);
  }

  attachInterrupt(5, lEncoderIncrement, CHANGE);
  attachInterrupt(4, rEncoderIncrement, CHANGE);

   Serial.begin(9600);
   delay(100);

   lcd.begin(16, 2);
   lcd.print("TIF 13");
}

void loop() {
  
 moveForward(255,255);
}

void moveForward(int lpwm, int rpwm){
  
  lMotorForward(lpwm);
  rMotorForward(rpwm);
  
}

void lMotorForward(int pwm){
  digitalWrite(4,HIGH);
  digitalWrite(3,LOW);
  analogWrite(2, pwm);
}

void rMotorForward(int pwm){
  digitalWrite(6,HIGH);
  digitalWrite(5,LOW);
  analogWrite(7, pwm);
}

void motorStop(){
    digitalWrite(3,LOW);
    digitalWrite(4,LOW);
    analogWrite(2, 0);
    digitalWrite(6,LOW);
    digitalWrite(5,LOW);
    analogWrite(7, 0);
}

void lEncoderIncrement(){
  lEncoderCount++;
}

void rEncoderIncrement(){
  rEncoderCount++;
}

int getLEncoderCount(){
  return lEncoderCount;
}

int getREncoderCount(){
  return  rEncoderCount;
}

int getLowerIrRead(){
  return analogRead(A0);  
}

int getHigherIrRead(){
  return analogRead(A1);  
}

double getDistance(int encorder){
  return (encorder * ENCORDER_UNIT);
}

void printDouble(double d){
  char ch[15];
  dtostrf(d,7, 3, ch);
  lcd.print(ch);  
}

void doPID(){

  int rEn=getREncoderCount();
  int len=getLEncoderCount();

  int err= rEn-len;

  lEncoderCount+=err;
  rEncoderCount-=err;

  int K = 10;
  int bassSpeed=190;

  int lSpeed=bassSpeed + (err*K);
  int rSpeed=bassSpeed - (err*K);

  Serial.print(getREncoderCount());
  Serial.print(",");
  Serial.println(getLEncoderCount());
  moveForward(lSpeed,rSpeed );
  delay(500);

}
