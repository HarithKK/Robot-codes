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
#include<math.h>

int lEncoderCount;
int rEncoderCount;

int lowerStart=0,lowerFinish=0,higherStart=0,higherFinish=0;

double lenA=0,lenH=0,lenB=0;

#define ENCORDER_UNIT 0.3157232
#define LEVEL 200
#define HBetSen 9.5
#define HlenHigher 21.5

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
  
    lowerStart=0;
    lowerFinish=0;
    higherStart=0;
    higherFinish=0;
  
    moveForward(255,255);
    
    while(getLowerIrRead() < LEVEL);
    lowerStart=geActualEnc();
    Serial.println(lowerStart);
    while(getHigherIrRead() < LEVEL);
    higherStart = geActualEnc();
    Serial.println(higherStart);
    while(getLowerIrRead() > LEVEL);
    lowerFinish=geActualEnc();
    Serial.println(lowerFinish);
    while(getHigherIrRead() > LEVEL);
    higherFinish = geActualEnc();
    Serial.println(higherFinish);

    setAandH();
    doEncorder(50);
    
    while(1);
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
  digitalWrite(4,HIGH);
  digitalWrite(3,HIGH);
  analogWrite(2, 255);
  digitalWrite(6,HIGH);
  digitalWrite(5,HIGH);
  analogWrite(7, 255);
  delay(100);
   digitalWrite(4,LOW);
  digitalWrite(3,LOW);
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

int geActualEnc(){
  return (getREncoderCount() + getLEncoderCount() )/2 ; 
}

void doEncorder(int i){
   rEncoderCount=0;
   while(getREncoderCount() < i); 
}

void setAandH(){

    lcd.clear();
    int lowerDiff  = lowerFinish - lowerStart ;
    int higherDiff = higherFinish-higherStart;

    double hlen = getDistance(higherDiff);
    double llen=  getDistance(lowerDiff);
    
    if ( (lowerDiff - higherDiff) >7){
      
      double ang = atan(HBetSen / (llen-hlen));
      double W =  (HlenHigher * (llen-hlen))/ HBetSen; 
      lenA = hlen +W ;
      lenH = lenA * (HBetSen / (llen-hlen));
    }else {
      lenA = llen;
      lenH = llen;
    }
    
    lcd.print(lenA);
    lcd.setCursor(7,0);
    lcd.print(lenH);
      lcd.setCursor(0,1);
      lcd.print(lowerDiff);
      lcd.setCursor(5,1);
      lcd.print(higherDiff);
    
}

void turnLeft(){
  
    int leftCounter =23;
  
    motorBrake();
    
    lEncoderCount=0;
    rEncoderCount=0;
    
    rMotorForward(255);
    lMotorReverse(255);
    while(lEncoderCount<leftCounter ){
      Serial.print(lEncoderCount);
      Serial.print(",");
      Serial.println(rEncoderCount);
    }
    motorStop();  
      
}
