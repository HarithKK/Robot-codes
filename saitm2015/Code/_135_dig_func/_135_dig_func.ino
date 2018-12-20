#include <Servo.h>
#include <SharpIR.h>

#define L_LEFT_90 (sensorReadings[0]==1)&&(sensorReadings[1]==1)&&(sensorReadings[2]==1)
#define L_RIGHT_90 (sensorReadings[3]==1)&&(sensorReadings[4]==1)&&(sensorReadings[5]==1)
#define CROSS sensorReadings[0]==1 && sensorReadings[5]==1
#define L_ANGLE_45 (sensorReadings[0]==1)&&(sensorReadings[2]==1)
#define R_ANGLE_45 (sensorReadings[3]==1)&&(sensorReadings[4]==0) &&(sensorReadings[5]==1)
#define R_ANGLE_135 (sensorReadings[4]==1))
#define L_ANGLE_135 (sensorReadings[1]==1))
#define SE_POINT (sensorReadings[0]==1)&&(sensorReadings[5]==1)&&(sensorReadings[1]==1)&&(sensorReadings[2]==1)&&(sensorReadings[3]==1)&&(sensorReadings[4]==1)
#define FREERIDE_COUNTER 25

#define SET_P Posision=1
#define SET_Q Posision=2
#define SET_R Posision=3
#define SET_S Posision=4

#define SET_LOWER_SPEED maxS=200;baseSpeed=100
#define SET_HIGHER_SPEED maxS=255;baseSpeed=190
#define SET_MIDDLE_SPEED maxS=255;baseSpeed=130


#define POS_S read>65 && read <85
#define POS_R read>55 && read <63
#define POS_PQ read>40 && read <50 


Servo servoMain;
Servo servoSub;

SharpIR rSharp(A0, 25, 93, 1080);//1080 for GP2Y0A21Y, 20150 for GP2Y0A02Y 		
SharpIR lsharp(A2, 25, 93, 1080);
SharpIR fsharp(A1, 25, 93, 1080);

int x=0,y=0;
int ballGot=0;
int Posision=0;
int sensorReadings[6];
//int sensorValues[3]={0,0,0};
short error = 0;

int count = 0;
int IS_BALL_DETECTED=0;
short errorValues[] = {3,2,1,-1,-2,-3};

int maxS = 255, minS = 140, baseSpeed = 190;//190 minspeed=140 mx 255

int p = 25;
int d = 45;

int leftSpeed, rightSpeed;
int l_error = 0, diff;

int lEncoderCount;
int rEncoderCount;

int ballDetected=0;

void setup() {
  initPins();
  initEncoders();
  initArmPins();
  initSensorVlues();
  pinMode(13,OUTPUT);
  Serial.begin(9600);
}
int read;
// the loop routine runs over and over again forever:
void loop() {

 SET_HIGHER_SPEED;
 
 while(!((sensorReadings[3]==1)&&(sensorReadings[4]==0)  &&(sensorReadings[5]==1))){
   doPID();
 }

 turn135RightSens();
 stopMotors();
 //process_R_posision();
 while(1);
 
 while(1){
   if(L_RIGHT_90){
     setGridValue(1,0);
     doencode(5);
     break;
   }else{
     doPID();
   }
 }
  while(1){
   if(L_RIGHT_90){
     setGridValue(2,0);
     doencode(5);
     break;
   }else{
     doPID();
   }
 }

 // 1st junc pass
 /*
 while(1){
   
   read= rSharp.distance();
   if(L_RIGHT_90){
      setGridValue(2,0);
      stopMotors();
      delay(100);
      break;
   }else{
     if(POS_R){
       SET_R;
       ballDetected=1;
       break;
     }else if(POS_S){
       SET_S;
       ballDetected=1;
       break;
     }else{
       doPID();
     }
   }
 }
 */
 if(ballDetected==1){
   if(Posision==3){
      process_R_posision();
   }
   if(Posision==4){
      process_S_posision();
   }
 }
 
 // junction 2 passed
 
 stopMotors();
 while(1);
 
 
}

void process_R_posision(){
  updateSensorArray();
  
  while(!(sensorReadings[3]==1 && sensorReadings[4]==1 && sensorReadings[5]==1)){
    doPID();
    updateSensorArray();
  }
  freeRide();
  turn90Right();
  updateSensorArray();
  
  while(!(sensorReadings[0]==1 && sensorReadings[5]==1)){
    doPID();
    updateSensorArray();
  }
  freeRide();
  turn90Right();
  updateSensorArray();
  
  passCross();

  while(!((sensorReadings[0]==1) && (sensorReadings[5]==1) && (sensorReadings[1]==1) && (sensorReadings[4]==1))){
    doPID();
    updateSensorArray();
  }
  edgeFreeRide();
  turn90Left();
  updateSensorArray();
  
  while(!((sensorReadings[0]==1)&&(sensorReadings[1]==1)&&(sensorReadings[2]==1))){
    doPID();
    updateSensorArray();
  }
  freeRide();
  turn90Left();
  updateSensorArray();
  passCross();
  updateSensorArray();
  
  grabBall();
}


void process_S_posision(){
 turn90digRight();
  
  while(1){
    if(CROSS){
      break;
    }else{
      doPID();
    }
  }
  
  turn90digRight();
  while(1){
    if(CROSS){
      break;
    }else{
      doPID();
    }
  }
  
  while(1){
    if(CROSS){
      break;
    }else{
      doPID();
    }
  }
  
  turn90digLeft();
  
  while(1){
    if(L_LEFT_90){
      break;
    }else{
      doPID();
    }
  }
  
  while(1){
    if(L_LEFT_90){
      break;
    }else{
      doPID();
    }
  }

  while(1){
    if(L_ANGLE_45){
      break;
    }else{
      doPID();
    }
  }
  
  turn135digLeft();
  
  while(1){
    if(L_ANGLE_45){
      break;
    }else{
      doPID();
    }
  }
  
}



void freeRide(){
  lEncoderCount=0;
  while(lEncoderCount<FREERIDE_COUNTER){
      doPID();
  }
  motorBrake();
  delay(700);
  motorStop();
  delay(700);
}

void edgeFreeRide(){
  lEncoderCount = 0;
  while(lEncoderCount<20){
      moveForward(150, 150);
  }
  motorBrake();
  delay(250);
  motorStop();
  delay(700);
  
}

void doFineTune(){
    while(1){
      
      if(sensorReadings[1]==1 && (sensorReadings[2]==1 )){
        lMotorReverse(100);
        rMotorForward(100);
      }
      stopMotors();
      delay(100);
      if(sensorReadings[4]==1 && (sensorReadings[2]==1 )){
        rMotorReverse(100);
        lMotorForward(100);
      }
      delay(100);
      if(sensorReadings[3]==1 && sensorReadings[2]==1){
        break;
      }
    }
    delay(100);
    stopMotors();
}

void turn90digRight(){
    
    lEncoderCount=0;
    rMotorReverse(200);
    lMotorForward(200);
    while(lEncoderCount<35){
      Serial.println(lEncoderCount);
    }
    motorBrake();
    delay(250);
    motorStop();
    delay(500);
}

void turn90Right(){
  
  lEncoderCount=0;
    rMotorReverse(200);
    lMotorForward(200);
    while(lEncoderCount<25){
      Serial.println(lEncoderCount);
    }
    lEncoderCount=0;
    rMotorReverse(120);
    lMotorForward(120);
    while(lEncoderCount<10){
      Serial.println(lEncoderCount);
    }
    motorBrake();
    delay(250);
    motorStop();
    delay(500);
  
  
}

void turn90digLeft(){
    
    lEncoderCount=0;
    rMotorForward(200);
    lMotorReverse(200);
    while(lEncoderCount<35){
      Serial.println(lEncoderCount);
    }
    motorBrake();
    delay(250);
    motorStop();
    delay(500);
}

void turn90Left(){
    
    lEncoderCount=0;
    rMotorForward(200);
    lMotorReverse(200);
    while(lEncoderCount<25){
      Serial.println(lEncoderCount);
    }
    lEncoderCount=0;
    rMotorForward(120);
    lMotorReverse(120);
    while(lEncoderCount<10){
      Serial.println(lEncoderCount);
    }
    motorBrake();
    delay(250);
    motorStop();
    delay(500);
}


void turn45digLeft(){
    
    lEncoderCount=0;
    rMotorForward(200);
    lMotorReverse(200);
    while(lEncoderCount<22){
      Serial.println(lEncoderCount);
    }
    motorStop();
    delay(500);
}

void turn45digRight(){
   
    lEncoderCount=0;
    rMotorReverse(200);
    lMotorForward(200);
    while(lEncoderCount<22){
      Serial.println(lEncoderCount);
    }
    motorStop();
    delay(500);
}

void turn180digLeft(){
  
    motorBrake();
    lEncoderCount=0;
    rMotorForward(200);
    lMotorReverse(200);
    while(lEncoderCount<85){
      Serial.println(lEncoderCount);
    }
    motorStop();
    delay(500);
}

void turn135digLeft(){
    
    lEncoderCount=0;
    rMotorForward(200);
    lMotorReverse(200);
    while(lEncoderCount<67){
      Serial.println(lEncoderCount);
    }
    motorStop();
    delay(500);
}

void turn135Left(){
    
    lEncoderCount = 0;
      while(lEncoderCount<25){
        moveForward(150, 150);
    }
    motorBrake();
    delay(250);
    motorStop();
    delay(700); 
  
    lEncoderCount=0;
    rMotorForward(200);
    lMotorReverse(200);
    while(lEncoderCount<57){
      Serial.println(lEncoderCount);
    }
    rMotorForward(120);
    lMotorReverse(120);
    while(lEncoderCount<10){
      Serial.println(lEncoderCount);
    }
    motorStop();
    delay(500);
}

void turn135digRight(){
    
     lEncoderCount = 0;
      while(lEncoderCount<25){
        moveForward(150, 150);
    }
    motorBrake();
    delay(250);
    motorStop();
    delay(700); 
    
    rMotorReverse(200);
    lMotorForward(200);
    while(lEncoderCount<67){
      Serial.println(lEncoderCount);
    }
    motorStop();
    delay(500);
}

void turn135Right(){
  
    lEncoderCount = 0;
      while(lEncoderCount<25){
        moveForward(150, 150);
    }
    motorBrake();
    delay(250);
    motorStop();
    delay(700);  
  
    rMotorReverse(200);
    lMotorForward(200);
    while(lEncoderCount<57){
      Serial.println(lEncoderCount);
    }
    
    rMotorReverse(120);
    lMotorForward(120);
    while(lEncoderCount<10){
      Serial.println(lEncoderCount);
    }
    motorBrake();
    delay(250);
    motorStop();
    delay(500);
}
void turn135LeftSens(){
    
    lEncoderCount = 0;
      while(lEncoderCount<25){
        moveForward(150, 150);
    }
    motorBrake();
    delay(250);
    motorStop();
    delay(700); 
  
    updateSensorArray();
    while(!(sensorReadings[0]==1)){
      updateSensorArray();
      rMotorForward(200);
      lMotorReverse(200);
    }
    
    while(!(sensorReadings[3]==1 || sensorReadings[2]==1 || sensorReadings[4]==1)){
      updateSensorArray();
      rMotorForward(120);
      lMotorReverse(120);
    }
    motorStop();
    delay(500);
}
void turn135RightSens(){
    
    lEncoderCount = 0;
      while(lEncoderCount<25){
        moveForward(150, 150);
    }
    motorBrake();
    delay(250);
    motorStop();
    delay(700); 
    updateSensorArray();
    while(!(sensorReadings[5]==1)){
      updateSensorArray();
      rMotorReverse(200);
      lMotorForward(200);
    }
    
    while(!(sensorReadings[3]==1 || sensorReadings[2]==1 || sensorReadings[1]==1)){
      updateSensorArray();
      rMotorReverse(120);
      lMotorForward(120);
    }
    
    motorBrake();
    delay(250);
    motorStop();
    delay(700);  
  
    
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
  
  attachInterrupt(4, lEncoderIncrement, CHANGE);
  attachInterrupt(5, rEncoderIncrement, CHANGE);
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

void moveReverse(int lpwm,int rpwm,int enCount){

    lEncoderCount=0;
    rMotorReverse(lpwm);
    lMotorReverse(rpwm);
    while(lEncoderCount<enCount){
      Serial.println(lEncoderCount);
    }
    motorStop();
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
  delay(700);
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


void stopMotors(){
    motorBrake();
    delay(100);
    motorStop();
    delay(700);
}
void BallDetected(){
  read=fsharp.distance();

  if(read<23){
    motorBrake();
    delay(100);
    motorStop();
    delay(500);
    catchBall();
    delay(1000);

  }else
    doPID();
}

void BallRelease(){
  
  if(SE_POINT){
    
    lEncoderCount=0;
    while(lEncoderCount<10){
      doPID();
    }
    
    motorBrake();
    delay(100);
    motorStop();
    delay(500);
    releseBall();
    delay(300);
    while(1);
  }else
    doPID();
}

void checkValue(){

    if(L_RIGHT_90){
       x++;
       delay(50);
    }
    
    if(L_LEFT_90){
      y++;
      delay(50);
    }
    
}

void setGridValue(int x_,int y_){
  x=x_;
  y=y_;
}

void initSensorVlues(){
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  
}


void process_Q_posision(){
  
  stopMotors();
  delay(700);
  turn90Right();
  delay(200);
  updateSensorArray();
   while(1){
    updateSensorArray();
    read=fsharp.distance();
    if(read<15)
    {
      break;
    }else{
      doPID();
    }
  }
  updateSensorArray();
  stopMotors();
  delay(1000);
  BallDetected();
  while(sensorReadings[4]==1){
    updateSensorArray();
    doPID();
  }
  
  turn135digRight();
  
  while(!((sensorReadings[0]==1)&&(sensorReadings[5]==1)&&(sensorReadings[1]==1)&&(sensorReadings[2]==1)&&(sensorReadings[3]==1)&&(sensorReadings[4]==1))){
    updateSensorArray();
  }
  
  stopMotors();
  delay(1000);
  releseBall();
}

void process_P_posision(){  
  
  while(1){
    if(L_RIGHT_90){
      break;
    }else{
      doPID();
    }
  }
  
  while(1){
    if(R_ANGLE_45){
      break;
    }else{
      doPID();
    }
  }
  
}

void grabBall(){
   updateSensorArray();
   while(1){
    updateSensorArray();
    read=fsharp.distance();
    if(read<18)
    {
      break;
    }else{
      doPID();
    }
  }
  updateSensorArray();
  stopMotors();
  delay(500);
  BallDetected();
}

void passCross(){
   while(!(sensorReadings[1]==1 && sensorReadings[4]==1)){
    doPID();
    updateSensorArray();
  }
  doencode(7);
  updateSensorArray();
}

void doencode(int enccount ){
  lEncoderCount=0;
  while(lEncoderCount<enccount)
    moveForward(180,180);
   
}
