#include<math.h>

#define LEFT_MOTOR_REVERSE digitalWrite(3,HIGH);digitalWrite(4,LOW);
#define LEFT_MOTOR_FORWERD digitalWrite(4,HIGH);digitalWrite(3,LOW);
#define RIGHT_MOTOR_REVERSE digitalWrite(5,HIGH);digitalWrite(6,LOW);
#define RIGHT_MOTOR_FORWERD digitalWrite(6,HIGH);digitalWrite(5,LOW);

int read;
int sensorReadings[6];

void setup() {
  initPins();
}

// the loop routine runs over and over again forever:
void loop() {
  
  rMotorForward(150);
  
}


int count = 0;
short errorValues[] = {-3,-2,-1,1,2,3};

void readLine(){
 updateSensorArray();
 error = 0;
 count = 0;
 for(int i = 0; i < 6;  i++){
   error += sensorReadings[i] * errorValues[i] ;
   count += sensorReadings[i];
 }
 
}

int maxS = 255, minS = 140, baseSpeed = 175;
int p = 38;
int d = 55;


int leftSpeed, rightSpeed;
int l_error = 0, diff;

void doPID(){
  l_error = error;
  readLine();
  diff = l_error - error;
 if(count == 0){
   
   while(count == 0){
     readLine();
     if(l_error < 0)
       myRobot.right(140,0);
      else
        myRobot.left(0,140);
   
 }
   myRobot.forward(130,130);
   
 }else{
 
  
  leftSpeed = baseSpeed - error * p + d * diff;
  rightSpeed = baseSpeed + error * p - d * diff;

  leftSpeed = leftSpeed > maxS ? maxS: leftSpeed ;
  leftSpeed = leftSpeed < minS ? minS : leftSpeed ;

   rightSpeed = rightSpeed > maxS ? maxS: rightSpeed ;
  rightSpeed = rightSpeed < minS ? minS : rightSpeed ;

myRobot.forward(leftSpeed, rightSpeed);
 }
}



void updateSensorArray(){
  for(int i=0;i<12;i+=2){
    sensorReadings=digitalRead(40-i);
  }
}

void moveForward(int lpwm, int rpwm){
  
  lMotorForward(lpwm);
  rMotorForwrd(rpwm);
  
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
