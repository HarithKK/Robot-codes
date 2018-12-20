
#include <Servo.h> 
#include <NewPing.h>

#define MAX_MAIN_SERVO_POSITION 20
#define MIN_MAIN_SERVO_POSITION 200
#define INT_MAIN_SERVO_POSITION 160
#define CENTER_MAIN_SERVO_POSITION 100
#define MAX_SUB_SERVO_POSITION 150
#define MIN_SUB_SERVO_POSITION 44

#define L_LEFT_90 (sensorReadings[0]==1)&&(sensorReadings[1]==1)&&(sensorReadings[2]==1)
#define L_RIGHT_90 (sensorReadings[4]==1)&&(sensorReadings[5]==1)
#define CROSS sensorArrayValue==(sensorReadings[0]==1)&&(sensorReadings[5]==1)
#define SE_POINT sensorArrayValue=63

#define TRIGGER_PIN  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     9  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
 
Servo mainS; 
Servo subS;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

int sensorReadings[6];
short error = 0;

int count = 0;
short errorValues[] = {3,2,1,-1,-2,-3};

int maxS = 255, minS = 140, baseSpeed = 170;

int p = 25;
int d = 55;

int leftSpeed, rightSpeed;
int l_error = 0, diff;

int lEncoderCount;
int rEncoderCount;

// color sensor
int s0=39,s1=37,s2=33,s3=31;
int out_=21;
int flag_=0;
int counter_=0;
int countR=0,countG=0,countB=0;

void setup() 
{ 

  mainS.attach(10);
  subS.attach(11);

  //set sensor pins
  for(int i=0;i<16;i+=2){
    pinMode(32+i,INPUT);
  }

  // motor Pins
  for(int i=0;i<6;i++){
    pinMode(2+i,OUTPUT);
  }
  // encorder intialize
  attachInterrupt(5, lEncoderIncrement, CHANGE);
  attachInterrupt(4, rEncoderIncrement, CHANGE);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // color sensor init
  pinMode(s0,OUTPUT);
  pinMode(s1,OUTPUT); 
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  
  Serial.begin(9600);
  // initialize
  centerPosition();   // get center positon
  delay(1000);
}
 
void loop() 
{
  secondRound();
  //firstRound();
  while(1);
} 

void goUntilRightJunction(){
  updateSensorArray();
  while( !(sensorReadings[4]==1 && sensorReadings[4]==1 && sensorReadings[5]==1)){
    doPID();
    updateSensorArray();
  }
  doEncorder(19);
  turnRight();
  fineTune();
}

void goUntilLeftJunction(){
  updateSensorArray();
  while( !(sensorReadings[0]==1 && sensorReadings[1]==1 && sensorReadings[2]==1)){
    doPID();
    updateSensorArray();
  }
  doEncorder(19);
  turnLeft();
  fineTune();
}

void goUntilRightTJunction(){
  updateSensorArray();
  while( !(sensorReadings[3]==1 && sensorReadings[4]==1 && sensorReadings[5]==1)){
    doPID();
    updateSensorArray();
  }
}

void goUntilLeftTJunction(){
  updateSensorArray();
  while( !(sensorReadings[0]==1 && sensorReadings[1]==1 && sensorReadings[2]==1)){
    doPID();
    updateSensorArray();
  }
}

void goUntilTJunction(){
  updateSensorArray();
  while( !(sensorReadings[1]==1 && sensorReadings[2]==1 && sensorReadings[3]==1 && sensorReadings[4]==1)){
    doPID();
    updateSensorArray();
  }
  
}

void goUntilEndPosition(){
   int position_ =53,Counter=100; 

   rEncoderCount=0;
   lEncoderCount=0;
   
  while(rEncoderCount < position_ && lEncoderCount< position_){
        doPID();  
  }
  motorBrake();
  delay(200);
  motorStop();

}

void gotoEnd(){
  updateSensorArray();
  while( !(sensorReadings[1]==1 && sensorReadings[2]==1 && sensorReadings[3]==1 && sensorReadings[4]==1 && sensorReadings[0]==1 && sensorReadings[5]==1)){
    doPID();
    updateSensorArray();
  }
  doEncorder(35);
  motorBrake();
  delay(100);
  motorStop();
  delay(700);
}

void getBoxMax(){

  openHatchServoSlow(); // open hatch
  delay(700);
  moveServoSlowDown();//go down
  delay(1000);
  closeHatchServoSlow();   // close hatch
  delay(700);
  moveServoSlowUp();  // go up
  delay(1000);
  
}

void getBoxAndPush(){

  openHatchServoSlow(); // open hatch
  delay(700);
  moveServoSlowDown();//go down
  delay(1000);
  closeHatchServoSlow();   // close hatch
  delay(700);
  
}

void releaseBox(){
  moveServoSlowDown(); //go down
  delay(1000);
  openHatchServoSlow(); // open hatch
  delay(700);
  moveServoSlowUp();  // go up 
  delay(1000);
 closeHatchServoSlow();   // close hatch
  delay(700);
}

void centerPosition(){

    moveServoSlowUp();
    delay(500);
    closeHatchServoSlow();
    delay(500);
    
}

void moveServoSlowUp(){
  for(int i=MIN_MAIN_SERVO_POSITION;i>=MAX_MAIN_SERVO_POSITION;i--){
    mainS.write(i);
    delay(10);
  }
}
void moveServoSlowDown(){
  for(int i=MAX_MAIN_SERVO_POSITION;i<=MIN_MAIN_SERVO_POSITION;i++){
    mainS.write(i);
    delay(10);
  }
}

void closeHatchServoSlow(){
  for(int i=MAX_SUB_SERVO_POSITION;i>=MIN_SUB_SERVO_POSITION;i--){
    subS.write(i);
    delay(15);
  }
}

void openHatchServoSlow(){
  for(int i=MIN_SUB_SERVO_POSITION;i<=MAX_SUB_SERVO_POSITION;i++){
    subS.write(i);
    delay(15);
  }
}

void moveServoSlowCenter(){
  for(int i=MAX_MAIN_SERVO_POSITION;i<=CENTER_MAIN_SERVO_POSITION;i++){
    mainS.write(i);
    delay(15);
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

void moveBackward(int lpwm, int rpwm){
  
  lMotorReverse(lpwm);
  rMotorReverse(rpwm);
  
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
    int leftCounter =30;
  
    motorBrake();
    
    lEncoderCount=0;
    rEncoderCount=0;
    
    rMotorForward(170);
    lMotorReverse(170);
    while(lEncoderCount<leftCounter ){
      Serial.print(lEncoderCount);
      Serial.print(",");
      Serial.println(rEncoderCount);
    }
    
    motorStop();  
}

void turnRight(){
    int rightCounter =35;
  
    motorBrake();
    
    lEncoderCount=0;
    rEncoderCount=0;
    
    lMotorForward(170);
    rMotorReverse(170);
    while(lEncoderCount<rightCounter  ){
      Serial.print(lEncoderCount);
      Serial.print(",");
      Serial.println(rEncoderCount);
    }
    motorBrake();  
    delay(200);
    motorStop();
    delay(300);
}

void turnBack(){

    int backCounter=63;
  
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
    motorBrake();
    delay(200);
    motorStop();
    delay(500);  
}

void doEncorder(int enc){
  
  rEncoderCount=0;
  while(rEncoderCount<enc){
    moveForward(140,140);
  }
  motorBrake();
  delay(200);
  motorStop();
}

void doEncorder_(int enc){
  
  rEncoderCount=0;
  lEncoderCount=0;
  while(rEncoderCount<enc ){
    moveForward(140,140);
  }
}

void leftEncorde(int enc){
  lEncoderCount=0;
  while(lEncoderCount<enc){
    lMotorForward(120);
  }
  motorBrake();
  delay(200);
  motorStop();
}

void firstRound(){

  passStartPosition();
  goUntilRightJunction();
  goUntilRightJunction();
  detectBox();
  goUntilLeftJunction();
  goUntilLeftJunction();
  goUntilRightJunction();
  //ramp
  //passRamp();
  goUntilRightTJunction();
  doEncorder_(10);
  goUntilEndPosition();
  leftEncorde(7);
  releaseBox();
  turnBack();
  goUntilLeftJunction();
  goUntilTJunction();
  doEncorder(23);
  turnRight();
  fineTune();
  gotoEnd();
}

void passRamp(){

 int MaxS=maxS;
 int MINS=minS;
 int BassSpeed=baseSpeed;
 
 rEncoderCount=0;
 
  while(rEncoderCount<70){
    doPID();
  }

  maxS = 255;
  minS = 170;
  baseSpeed = 255;
  moveServoSlowCenter();

  rEncoderCount=0;
 
  while(rEncoderCount<70){
    doPID();
  }
  
  maxS = MaxS;
  minS = MINS;
  baseSpeed = BassSpeed;
  motorBrake();
  delay(100);
  motorStop();
  moveServoSlowUp();
  
}

void fineTune(){
 /* updateSensorArray();
   while(sensorReadings[4]==1 || sensorReadings[3]==1){
      updateSensorArray();
      lMotorForward(100);
   } 
   while (sensorReadings[1]==1 || sensorReadings[2]==1){
      updateSensorArray();
      rMotorForward(100);
    }*/
}

void detectBox(){

  float value;
  float minDist = 10.00;

  updateSensorArray();
  while(1){
      value=getSonarValue();
      Serial.println(value);
      if(value<=20.00 && value >15.0)
        break;
      doPID();
      delay(1);       
  }
    motorBrake();
    delay(300);
    motorStop();
    delay(1000);
    openHatchServoSlow(); // open hatch
    delay(700);
    moveServoSlowDown();//go down
    delay(1000);
    doEncorder(30);
    motorBrake();
    delay(300);
    motorStop();
    delay(1000);
    closeHatchServoSlow();   // close hatch
    delay(700);
    moveServoSlowUp();
    delay(700);
  
}

float getSonarValue(){
  float val;
  delay(31);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  val=(float)(uS / US_ROUNDTRIP_CM);
  return val;
}

void passStartPosition(){

      doEncorder(20);
      fineTune();
}

void TCS()
 {
   digitalWrite(s1,HIGH);
   digitalWrite(s0,LOW);
   flag_=0;
   attachInterrupt(2, ISR_INTO, CHANGE);
   timer2_init();
 }
 
void ISR_INTO()
 {
   counter_++;
 }
 void timer2_init(void)
 {
   TCCR2A=0x00;
   TCCR2B=0x07; //the clock frequency source 1024 points
   TCNT2= 100;    //10 ms overflow again
   TIMSK2 = 0x01; //allow interrupt
 }
 int i=0;
 ISR(TIMER2_OVF_vect)//the timer 2, 10ms interrupt overflow again. Internal overflow interrupt executive function
{
 TCNT2=100;
 flag_++;
 if(flag_==1)
  {
    counter_=0;
  }
 else if(flag_==2)
   {
    digitalWrite(s2,LOW);
    digitalWrite(s3,LOW); 
    countR=counter_/1.051;
    digitalWrite(s2,HIGH);
    digitalWrite(s3,HIGH);   
   }
 else if(flag_==3)
    {
     countG=counter_/1.0157;
     digitalWrite(s2,LOW);
     digitalWrite(s3,HIGH); 
   
    }
 else if(flag_==4)
    {
     countB=counter_/1.114;
     digitalWrite(s2,LOW);
     digitalWrite(s3,LOW);
     }
 else
     {
     flag_=0; 
      TIMSK2 = 0x00;
     }
     counter_=0;
     delay(2);
}
int colorDetect()
 {
  delay(10);
  TCS();
  if((countR>10)||(countG>10)||(countB>10))
   {
      if((countR>countG)&&(countR>countB))
       {
            Serial.print("red");
            Serial.print("\n");
            delay(1000);
       }
      else if((countG>=countR)&&(countG>countB))
       {
            Serial.print("green");
            Serial.print("\n");
            delay(1000);
       } 
     else if((countB>countG)&&(countB>countR))
      {
            Serial.print("blue");
            Serial.print("\n");
           delay(1000);
      }
    }
  else 
  {
     delay(1000);       
  }
 }

void secondRound(){
int boxColor=0;
  passStartPosition();
  goUntilRightJunction();
  goUntilRightJunction();
  detectBox();
  boxColor= colorDetect();
  goUntilLeftJunction();
  goUntilLeftJunction();
  goUntilRightJunction();
  //ramp
  goUntilRightJunction();
  goUntilTJunction();
  turnLeft();

  if(boxColor==1)
    gotoRed();
  if(boxColor==2)
    gotoGreen();
  if(boxColor==3)
    gotoBlue();
  
  doEncorder_(32);
  goUntilEndPosition();
  leftEncorde(6);
  goUntilLeftJunction();
  goUntilTJunction();
  doEncorder(23);
  turnRight();
  
  gotoEnd();
}

void gotoRed(){
goUntilTJunction();
  doEncorder(19);
    turnLeft();
    moveBackward(100,100);
    delay(500);
    motorBrake();
    rEncoderCount=0;
  motorBrake();
  delay(100);
  motorStop();
    releaseBox();
    turnBack();
    goUntilTJunction();
    turnRight();
    gotoEnd();
}

void  gotoGreen(){
  goUntilTJunction();
  doEncorder(15);
  releaseBox();
  turnBack();
  goUntilTJunction();
  doEncorder_(15);
  gotoEnd();
}

void gotoBlue(){
    goUntilTJunction();
  doEncorder(19);
    turnRight();
    moveBackward(100,100);
    delay(500);
    motorBrake();
    rEncoderCount=0;
  motorBrake();
  delay(100);
  motorStop();
    releaseBox();
    lEncoderCount=0;
    while(lEncoderCount<40){
      moveForward(100,100);
    }
    motorBrake();
    delay(100);
     turnRight();
     goUntilTJunction();
     doEncorder_(15);
     gotoEnd();
}
