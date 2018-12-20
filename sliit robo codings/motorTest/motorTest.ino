#define MAX_MAIN_SERVO_POSITION 20
#define MIN_MAIN_SERVO_POSITION 200
#define INT_MAIN_SERVO_POSITION 160
#define MAX_SUB_SERVO_POSITION 150
#define MIN_SUB_SERVO_POSITION 43


#include <Servo.h> 
 
Servo mainS; 
Servo subS;

int sensorReadings[6];

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

  Serial.begin(9600);
  // initialize
  centerPosition();   // get center positon
  delay(1000);
}
 
void loop() 
{ 
  moveForward(255,255);
  
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

