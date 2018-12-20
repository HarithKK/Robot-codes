#include <QTRSensors.h>
#include <NewPing.h>
#include <Servo.h>

#define TRIGGER_PIN  13  
#define ECHO_PIN     12 
#define MAX_DISTANCE 200
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

#define NUM_SENSORS   8     
#define TIMEOUT       2500  
#define EMITTER_PIN   2     
QTRSensorsRC qtrrc((unsigned char[]) {48, 46, 42, 40, 38, 36,34,32},NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

#define LEFT_PLUS 7
#define LEFT_MINUS 9
#define LEFT_PWM 8

#define RIGHT_PLUS 4
#define RIGHT_MINUS 6
#define RIGHT_PWM 5

#define RH_ENCODER_A 21 //black
#define RH_ENCODER_B 20 // red
#define LH_ENCODER_A 18 //blue
#define LH_ENCODER_B 19 // white

#define SWITCH 50
#define BUZZ 52

#define IS_SWITCH_OPEN digitalRead(SWITCH)>0

#define LIMIT 31
#define LIMIT_NOT_PRESSED (digitalRead(LIMIT)>0)

volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;

#define MAIN_SERVO 10
#define SUB_SERVO 11

#define MOTOR_PLUS 24
#define MOTOR_MINUS 22

Servo mainServo;
Servo subServo;

void setup() {

  // set Motor PINS  
  for(int i=4;i<=9;i+=2){
    pinMode(i,OUTPUT);
  }
  // Encorder
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  
  // initialize hardware interrupts
  attachInterrupt(4, leftEncoderEvent, CHANGE);
  attachInterrupt(2, rightEncoderEvent, CHANGE);
  //for sensor panel
  for(int i=32;i<=48;i+=2){
    pinMode(i,INPUT);
  }

  pinMode(BUZZ,OUTPUT);
  Serial.begin(9600);

  //servo
  mainServo.attach(MAIN_SERVO);
  mainServo.write(0); 

  subServo.attach(SUB_SERVO);
  subServo.write(0); 
  //limit
  pinMode(LIMIT,INPUT);

  // motor
  pinMode(MOTOR_PLUS,OUTPUT);
  pinMode(MOTOR_MINUS,OUTPUT);
}



void loop() {
  //testA();
  delay(1000);
  for (int pos = 0; pos <=60; pos +=1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    subServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  while(1);
}

void testA(){
  //checkPanel();
  //lMotorForward(255);
  //rMotorForward(255);

  /* check Enc*/

//  motorForward(255,255);
//  Serial.print("L: ");
//  Serial.print(leftCount);
//  Serial.print("\tright: ");
//  Serial.println(rightCount);
/*PWM*/
//  for(int i=255;i>20;i--){
//    motorForward(i,i);
//    Serial.println(i);
//    delay(1);
//  }

/* sonar */
//Serial.println(readSonar());
//delay(300);

/* switch and buzer*/
//while(IS_SWITCH_OPEN);
//beep(3);

/*limit*/
//while(LIMIT_NOT_PRESSED);
//beep(3);

/*motor*/
//motorUp();
//delay(60);
//while(LIMIT_NOT_PRESSED);
//motorStopUpDown();
//while(1);

/*servo main*/
//for (int pos = 0; pos > -90; pos --) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    mainServo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
//  while(1);

}

void beep(int i){
    for(int j=0;j<i;j++){
      digitalWrite(52,HIGH);
      delay(30);
      digitalWrite(52,LOW);
      delay(50);   
    }
}

void checkPanel(){
  qtrrc.read(sensorValues);
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]/2500);
    Serial.print('\t'); 
  }
  Serial.println();
  
  delay(250);  
}

void leftEncoderEvent() {
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount++;
    } else {
      leftCount--;
    }
  } else {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  }
}
 
void rightEncoderEvent() {
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  }
}

void motorForward(int pwml,int pwmr){
  rMotorForward(pwmr);
  lMotorForward(pwml);  
}

void motorReverse(int pwml,int pwmr){
  rMotorReverse(pwmr);
  lMotorReverse(pwml);  
}

void lMotorForward(int pwm){
  digitalWrite(LEFT_PLUS,HIGH);
  digitalWrite(LEFT_MINUS,LOW);
  analogWrite(LEFT_PWM,pwm);
}

void rMotorForward(int pwm){
  digitalWrite(RIGHT_PLUS,HIGH);
  digitalWrite(RIGHT_MINUS,LOW);
  analogWrite(RIGHT_PWM, pwm);
}

void lMotorReverse(int pwm){
  digitalWrite(LEFT_PLUS,LOW);
  digitalWrite(LEFT_MINUS,HIGH);
  analogWrite(LEFT_PWM,pwm);
}

void rMotorReverse(int pwm){
  digitalWrite(RIGHT_PLUS,LOW);
  digitalWrite(RIGHT_MINUS,HIGH);
  analogWrite(RIGHT_PWM, pwm);
}

void motorStop(){
  
  digitalWrite(RIGHT_PLUS,HIGH);
  digitalWrite(RIGHT_MINUS,HIGH);
  analogWrite(RIGHT_PWM, 0);
  digitalWrite(LEFT_PLUS,HIGH);
  digitalWrite(LEFT_MINUS,HIGH);
  analogWrite(LEFT_PWM,0);
  delay(10);
  digitalWrite(RIGHT_PLUS,LOW);
  digitalWrite(RIGHT_MINUS,LOW);
  digitalWrite(LEFT_PLUS,LOW);
  digitalWrite(LEFT_MINUS,LOW);
  
}

double readSonar(){
  int uS = sonar.ping(); 
  return (int)(uS / US_ROUNDTRIP_CM);
}

void motorUp(){
  digitalWrite(MOTOR_PLUS,HIGH);
  digitalWrite(MOTOR_MINUS,LOW);  
  delay(60);
}


void motorDown(){
  digitalWrite(MOTOR_PLUS,LOW);
  digitalWrite(MOTOR_MINUS,HIGH);
  delay(60);
}


void motorStopUpDown(){
  digitalWrite(MOTOR_PLUS,HIGH);
  digitalWrite(MOTOR_MINUS,HIGH);
  delay(50);
  digitalWrite(MOTOR_PLUS,LOW);
  digitalWrite(MOTOR_MINUS,LOW);
}
