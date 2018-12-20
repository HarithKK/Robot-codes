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
#include <SharpIR.h>
#include <Servo.h> 

int lEncoderCount;
int rEncoderCount;
char type=0;
int lowerStart=0,lowerFinish=0,higherStart=0,higherFinish=0;

double lenA=0,lenH=0,lenB=0;

#define ENCORDER_UNIT (2/5)
#define LEVEL 200
#define HBetSen 5
#define HlenHigher 14
#define DEFULT_ 50

LiquidCrystal lcd(34, 36, 38, 40, 42, 44);
SharpIR hsh(A1, 25, 93, 1080);//1080 for GP2Y0A21Y, 20150 for GP2Y0A02Y 
Servo ser;

void setup() {
  // motor Pins
  for(int i=0;i<6;i++){
    pinMode(2+i,OUTPUT);
  }

  attachInterrupt(5, lEncoderIncrement, CHANGE);
  attachInterrupt(4, rEncoderIncrement, CHANGE);

   ser.attach(11);
   Serial.begin(9600);
   delay(100);

   lcd.begin(16, 2);
   lcd.print("TIF 13");
   for(int i=0;i<DEFULT_;i++){
        ser.write(i);
        delay(5);
  }
}

void loop() {
  
    double Vol=0;
   
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
    motorStop();
    while(1);
    turnLeft();

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

    setB();
    motorStop();
    delay(100);

    Vol=calcArea();
    lcd.clear();
    lcd.setCursor(0,0);

    if(type=='c'){
      moveBack();
      doEncorder(10);
      motorStop();
    }
    setH();
//    lcd.clear();
//    lcd.print(lenA);
//    lcd.setCursor(7,0);
//    lcd.print(lenB);
//    lcd.setCursor(1,1);
//    lcd.print(lenH);
    while(1);
    
    if(type=='c')
      lcd.print("Vol of Cuboids");
    else
      lcd.print("Vol of Prism");
    
    lcd.setCursor(1,1);
    printDouble(Vol);
    lcd.print(" cm2");
    while(1);
}

void moveForward(int lpwm, int rpwm){
  
  lMotorForward(lpwm);
  rMotorForward(rpwm);
  
}

void moveBack(){
  digitalWrite(4,LOW);
  digitalWrite(3,HIGH);
  analogWrite(2, 255);
  digitalWrite(6,LOW);
  digitalWrite(5,HIGH);
  analogWrite(7, 255);
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

void lMotorReverse(int pwm){
  digitalWrite(4,LOW);
  digitalWrite(3,HIGH);
  analogWrite(2, pwm);
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
  dtostrf(d,7, 1, ch);
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
   while(getREncoderCount() < i){
    Serial.print(lEncoderCount);
   }
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
     
      type='p';
    }else {
      lenA = llen;
     // lenH = llen;
      type='c';
    }
    printLCDV();
}

void turnLeft(){
  
    int leftCounter =27;

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
    delay(300);  
}

void setB(){
    int lowerDiff  = lowerFinish - lowerStart ;
    int higherDiff = higherFinish-higherStart;

    double hlen = getDistance(higherDiff);
    double llen=  getDistance(lowerDiff);

    lenB = llen; 

}

double calcArea(){
  if(type=='c')
    return (lenA * lenB * lenH);
  else
    return ((lenA * lenB * lenH)/2);  
}

int readIRDist(){
  return hsh.distance();  
}

void setH(){
     int i,read;
     double height=0;
    
     read = readIRDist();
     for(int i=DEFULT_;i>0;i--){
        ser.write(i);
        delay(5);
     }
     for(i=0;i<120;i++){
        ser.write(i);
        if(readIRDist()>200)
          break;
        else
          continue;
        delay(15);
      }  

     int ang=i-90;
     double rd= ((double)ang * 1000) / 57296;
     height =  (tan(rd))* read;
     height += HlenHigher;
//         lcd.clear();
//         lcd.print(rd);
//         lcd.setCursor(7,1);
//         lcd.print(read);
//         lcd.setCursor(0,1);
//         lcd.print(height);
//         //lcd.setCursor(7,0);
}

void printLCDV(){
         lcd.clear();
         lcd.print(lenA);
         lcd.setCursor(7,0);
         lcd.print(lenB);
         lcd.setCursor(0,1);
         lcd.print(lenH);  
}

