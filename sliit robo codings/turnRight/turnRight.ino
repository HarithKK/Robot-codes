#define MAX_MAIN_SERVO_POSITION 20
#define MIN_MAIN_SERVO_POSITION 200
#define INT_MAIN_SERVO_POSITION 160
#define MAX_SUB_SERVO_POSITION 150
#define MIN_SUB_SERVO_POSITION 43


#include <Servo.h> 
 
Servo mainS; 
Servo subS;

int sensorReadings[6];
short error = 0;

int count = 0;
short errorValues[] = {3,2,1,-1,-2,-3};

int maxS = 255, minS = 50, baseSpeed = 190;

int p = 25;
int d = 45;

int leftSpeed, rightSpeed;
int l_error = 0, diff;

int lEncoderCount;
int rEncoderCount;

void setup() 
{ 

  mainS.attach(10);
  subS.attach(11);

  //set sensor pins
  for(int i=0;i<12;i+=2){
    pinMode(30+i,INPUT);
  }

  // motor Pins
  for(int i=0;i<6;i++){
    pinMode(2+i,OUTPUT);
  }
  // encorder intialize
  attachInterrupt(5, lEncoderIncrement, CHANGE);
  attachInterrupt(4, rEncoderIncrement, CHANGE);
  
  Serial.begin(9600);
  // initialize
  centerPosition();   // get center positon
  delay(1000);
}
 
void loop() 
{ 
  turnBack();
  while(1);
} 

void getBoxMax(){

  subS.write(MAX_SUB_SERVO_POSITION); // open hatch
  delay(700);
  moveServoSlowDown();//go down
  delay(1000);
  subS.write(MIN_SUB_SERVO_POSITION);   // close hatch
  delay(700);
  moveServoSlowUp();  // go up
  delay(1000);
  
}

void releaseBox(){
  moveServoSlowDown(); //go down
  delay(1000);
  subS.write(MAX_SUB_SERVO_POSITION); // open hatch
  delay(700);
  moveServoSlowUp();  // go up 
  delay(1000);
  subS.write(MIN_SUB_SERVO_POSITION); // open hatch
  delay(700);
}

void centerPosition(){
    mainS.write(MAX_MAIN_SERVO_POSITION);
    delay(1000);
    subS.write(MIN_SUB_SERVO_POSITION);
    delay(1000);
}

void moveServoSlowUp(){
  for(int i=MIN_MAIN_SERVO_POSITION;i>=MAX_MAIN_SERVO_POSITION;i--){
    mainS.write(i);
    delay(15);
  }
}
void moveServoSlowDown(){
  for(int i=MAX_MAIN_SERVO_POSITION;i<=MIN_MAIN_SERVO_POSITION;i++){
    mainS.write(i);
    delay(5);
  }
}


void updateSensorArray(){
  for(int i=0;i<=12;i+=2){
    sensorReadings[i/2]=digitalRead(40-i);
  }
}

void moveForward(int lpwm, int rpwm){
  
  lMotorForward(lpwm);
  rMotorForward(rpwm);
  
}

void lMotorForward(int pwm){
  digitalWrite(3,HIGH);
  digitalWrite(4,LOW);
  analogWrite(2, pwm);
}

void rMotorForward(int pwm){
  digitalWrite(6,HIGH);
  digitalWrite(5,LOW);
  analogWrite(7, pwm);
}

void lMotorReverse(int pwm){
  digitalWrite(4,HIGH);
  digitalWrite(3,LOW);
  analogWrite(2, pwm);
}

void rMotorReverse(int pwm){
  digitalWrite(5,HIGH);
  digitalWrite(6,LOW);
  analogWrite(7, pwm);
}

void motorStop(){
  
  digitalWrite(4,LOW);
  digitalWrite(3,LOW);
  analogWrite(2, 0);
  digitalWrite(6,LOW);
  digitalWrite(5,LOW);
  analogWrite(7, 0);
  
}

void motorBrake(){

  digitalWrite(4,HIGH);
  digitalWrite(3,HIGH);
  analogWrite(2, 255);
  digitalWrite(6,HIGH);
  digitalWrite(5,HIGH);
  analogWrite(7, 255);
  
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
  //Serial.println(error);
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

void turnRight(){
    int rightCounter =23;
  
    motorBrake();
    
    lEncoderCount=0;
    rEncoderCount=0;
    
    lMotorForward(255);
    rMotorReverse(255);
    while(rEncoderCount<rightCounter ){
      Serial.print(lEncoderCount);
      Serial.print(",");
      Serial.println(rEncoderCount);
    }
    motorStop();  
}

void turnBack(){
    motorBrake();
    
    lEncoderCount=0;
    rEncoderCount=0;
    
    lMotorForward(255);
    rMotorReverse(255);
    while(rEncoderCount<backCounter && lEncoderCount<backCounter){
      Serial.print(lEncoderCount);
      Serial.print(",");
      Serial.println(rEncoderCount);
    }
    motorStop();  
}
