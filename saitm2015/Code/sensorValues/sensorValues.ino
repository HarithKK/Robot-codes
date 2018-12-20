#include <Servo.h>

Servo servoMain;
Servo servoSub; 

int sensorReadings[6];
int sensorValues[3]={0,0,0};
short error = 0;

int count = 0;
short errorValues[] = {3,2,1,-1,-2,-3};

int maxS = 255, minS = 140, baseSpeed = 250;//190

int p = 25;
int d = 45;

int leftSpeed, rightSpeed;
int l_error = 0, diff;

int lEncoderCount;
int rEncoderCount;


void setup() {
  initPins();
  initEncoders();
  initArmPins();
  initSensorVlues();
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  getSensorValues();
  for(int i=0;i<3;i++){
    Serial.print(sensorValues[i]);
    Serial.print(" , ");
  }
  Serial.println();
  delay(100);
}

void initSensorVlues(){
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
}

void getSensorValues(){
 for(int i=0;i<3;i++){
   sensorValues[i]=analogRead(A0+i);
 }
}

void turn90digRight(){

    motorBrake();
    lEncoderCount=0;
    rMotorReverse(255);
    lMotorForward(255);
    while(lEncoderCount<36){
      Serial.println(lEncoderCount);
    }
    motorStop();
}

void turn90digLeft(){

    motorBrake();
    lEncoderCount=0;
    rMotorForward(255);
    lMotorReverse(255);
    while(lEncoderCount<36){
      Serial.println(lEncoderCount);
    }
    motorStop();
}

void turn45digLeft(){

    motorBrake();
    lEncoderCount=0;
    rMotorForward(255);
    lMotorReverse(255);
    while(lEncoderCount<22){
      Serial.println(lEncoderCount);
    }
    motorStop();
}

void turn45digRight(){

    motorBrake();
    lEncoderCount=0;
    rMotorReverse(255);
    lMotorForward(255);
    while(lEncoderCount<22){
      Serial.println(lEncoderCount);
    }
    motorStop();
}

void turn180digLeft(){

    motorBrake();
    lEncoderCount=0;
    rMotorForward(255);
    lMotorReverse(255);
    while(lEncoderCount<85){
      Serial.println(lEncoderCount);
    }
    motorStop();
}

void turn135digLeft(){

    motorBrake();
    lEncoderCount=0;
    rMotorForward(255);
    lMotorReverse(255);
    while(lEncoderCount<67){
      Serial.println(lEncoderCount);
    }
    motorStop();
}

void turn135digRight(){

    motorBrake();
    lEncoderCount=0;
    rMotorReverse(255);
    lMotorForward(255);
    while(lEncoderCount<67){
      Serial.println(lEncoderCount);
    }
    motorStop();
}

void initArmPins(){
  servoMain.attach(10);
  servoSub.attach(11);
  
  armUp();
  armCatch();
}

void armUp(){
  servoMain.write(180);
  delay(15);
}

void armDown(){
  servoMain.write(60);
  delay(15);
}

void armCatch(){
  servoSub.write(0);
  delay(15);
}

void armRelese(){
  servoSub.write(118);
  delay(15);
}

void catchBall(){
  armRelese();
  delay(100);
  armDown();
  delay(700);
  armCatch();
  delay(100);
  armUp();
  delay(100);
}

void releseBall(){
  
  armDown();
  delay(100);
  armRelese();
  delay(700);
  armUp();
  delay(100);
  armCatch();
  delay(100);
  
}

void initEncoders(){
  
  attachInterrupt(5, lEncoderIncrement, CHANGE);
  attachInterrupt(4, rEncoderIncrement, CHANGE);
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

void readLine(){
 updateSensorArray();
 error = 0;
 count = 0;
 for(int i = 0; i < 6;  i++){
   error += sensorReadings[i] * errorValues[i] ;
   count += sensorReadings[i];
 }
 
}

void doPID(){
  l_error = error;
  readLine();
  diff = l_error - error;
 if(count == 0){
   
   while(count == 0){
     readLine();
     if(l_error < 0)
       moveForward(140,0);
     else
       moveForward(0,140);
   
   }
   moveForward(130,130);
   
 }else{
 
  leftSpeed = baseSpeed - error * p + d * diff;
  rightSpeed = baseSpeed + error * p - d * diff;

  leftSpeed = leftSpeed > maxS ? maxS: leftSpeed ;
  leftSpeed = leftSpeed < minS ? minS : leftSpeed ;

  rightSpeed = rightSpeed > maxS ? maxS: rightSpeed ;
  rightSpeed = rightSpeed < minS ? minS : rightSpeed ;

  moveForward(leftSpeed, rightSpeed);
 }
}



void updateSensorArray(){
  for(int i=0;i<12;i+=2){
    sensorReadings[i/2]=digitalRead(40-i);
  }
}

void moveForward(int lpwm, int rpwm){
  
  lMotorForward(lpwm);
  rMotorForward(rpwm);
  
}

void motorBrake(){

  digitalWrite(4,HIGH);
  digitalWrite(3,HIGH);
  analogWrite(2, 255);
  digitalWrite(6,HIGH);
  digitalWrite(5,HIGH);
  analogWrite(7, 255);
  
}

void motorStop(){
  
  digitalWrite(4,LOW);
  digitalWrite(3,LOW);
  analogWrite(2, 0);
  digitalWrite(6,LOW);
  digitalWrite(5,LOW);
  analogWrite(7, 0);
  
}
  

void rMotorForward(int pwm){
  digitalWrite(4,HIGH);
  digitalWrite(3,LOW);
  analogWrite(2, pwm);
}

void lMotorForward(int pwm){
  digitalWrite(6,HIGH);
  digitalWrite(5,LOW);
  analogWrite(7, pwm);
}

void rMotorReverse(int pwm){
  digitalWrite(3,HIGH);
  digitalWrite(4,LOW);
  analogWrite(2, pwm);
}

void lMotorReverse(int pwm){
  digitalWrite(5,HIGH);
  digitalWrite(6,LOW);
  analogWrite(7, pwm);
}

void initPins(){
  
// set Motor PINS  
  for(int i=0;i<6;i++){
    pinMode(2+i,OUTPUT);
  }

//set SensorPanel Pins
  for(int i=0;i<12;i+=2){
    pinMode(30+i,INPUT);
  }
  
}
