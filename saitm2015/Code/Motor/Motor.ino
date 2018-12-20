#include<math.h>

#define LEFT_MOTOR_REVERSE digitalWrite(3,HIGH);digitalWrite(4,LOW);
#define LEFT_MOTOR_FORWERD digitalWrite(4,HIGH);digitalWrite(3,LOW);
#define RIGHT_MOTOR_REVERSE digitalWrite(5,HIGH);digitalWrite(6,LOW);
#define RIGHT_MOTOR_FORWERD digitalWrite(6,HIGH);digitalWrite(5,LOW);

int read;
int irArray[6];

void setup() {

// set Motor PINS  
  for(int i=0;i<6;i++){
    pinMode(2+i,OUTPUT);
  }

//set SensorPanel Pins
  for(int i=0;i<12;i+=2){
    pinMode(30+i,INPUT);
  }
  
}

// the loop routine runs over and over again forever:
void loop() {
  
  rMotorForward(150);
  lMotorForward(150);

  delay(3000);

  motorBrake();
  motorStop();
  delay(3000);
}

void lMotorForward(int pwm){
  digitalWrite(4,HIGH);
  digitalWrite(3,LOW);
  analogWrite(2, pwm);
}

void rMotorForward(int pwm){
  digitalWrite(5,HIGH);
  digitalWrite(6,LOW);
  analogWrite(7, pwm);
}

void motorForward(int pwml,int pwmr){
  rMotorForward(pwmr);
  lMotorForward(pwml);  
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

