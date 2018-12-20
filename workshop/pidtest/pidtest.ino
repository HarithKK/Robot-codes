#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanMotors.h>
#include <OrangutanAnalog.h>
#include <OrangutanLEDs.h>
#include <OrangutanLCD.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanBuzzer.h>

#define PASS_DELAY 200
#define IS_BLACK(p) p>200
#define IS_WHITE(p) p<100

Pololu3pi robot;
OrangutanPushbuttons buttons;
OrangutanLCD lcd;

unsigned int sensors[5]; 
unsigned int sensorsT[5]; 
unsigned int counter;

float KP=0.05;
float KD=3/2;
float KI=1/10000;
int lastError=0;
int integral=0;
int M=60;

char turnMode='A';

int m1Speed;
int m2Speed;

void setup() {
 robot.init(2000);
 OrangutanPushbuttons::waitForRelease(BUTTON_B);
 lcd.clear();
 lcd.print("calibrate");
 buttons.waitForPress(BUTTON_B);
 delay(1000);
 for (counter=0; counter<40; counter++)
  {
    if (counter <= 20 || counter >= 40)
      OrangutanMotors::setSpeeds(40, -40);
    else
      OrangutanMotors::setSpeeds(-40, 40);
    robot.calibrateLineSensors(IR_EMITTERS_ON);

    delay(20);
  }
  for (counter=0; counter<40; counter++)
  {
    if (counter <= 20 || counter >= 40)
      OrangutanMotors::setSpeeds(-40, 40);
    else
      OrangutanMotors::setSpeeds(40, -40);
    robot.calibrateLineSensors(IR_EMITTERS_ON);

    delay(20);
  }
  OrangutanMotors::setSpeeds(0, 0);
  lcd.clear();
  lcd.print("OK 1");
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);
    lcd.clear();
    lcd.print(position);
    delay(100);
  }
  OrangutanPushbuttons::waitForRelease(BUTTON_B);
  delay(1000);

}

void loop() {
  while(1){
   doPID();

    if(getPos()==0){
      turnMode='B';
      break;
    }
    if (IS_BLACK(sensors[4]))  //right || cross
    {
      turnMode='R';
      break;
    }else
    if (IS_BLACK(sensors[0]))  //right || cross
    {
      turnMode='L';
      break; 
    }
  }
  
  OrangutanMotors::setSpeeds(50, 50);
  delay(PASS_DELAY);
  motorStop();
  
  switch(turnMode){
    case 'R':  {
        robot.readLine(sensorsT, IR_EMITTERS_ON);
        if(IS_BLACK(sensorsT[0])){
          OrangutanMotors::setSpeeds(50, 50);
          delay(300);
          motorStop();
          while(1);
        }else{
          turnRight();
        }
      
      };break;
    case 'L':  turnLeft();break;
    case 'B':  turnBack();break;
    case 'E':  endComes();break;
  }
  OrangutanMotors::setSpeeds(50, 50);
  delay(PASS_DELAY);

  
}

void turnLeft(){
    motorStop();
    OrangutanMotors::setSpeeds(-100, +100);
    delay(137);
    motorStop();
}

void turnRight(){
    motorStop();
    OrangutanMotors::setSpeeds(+100,-100);
    delay(137);
    motorStop();
}

void turnBack(){
    motorStop();
    OrangutanMotors::setSpeeds(100, -100);
    delay(278);
    motorStop();
}

void endComes(){
     motorStop();
     while(1);
}

void doPID(){
  int position = robot.readLine(sensors, IR_EMITTERS_ON);
  int error = position - 2000;
  integral+=error;
  int motorSpeed = KP * error + KD * (error - lastError)+ integral*KI;
  lastError = error;

  if(motorSpeed > M){
    motorSpeed=M;
  }
  if(motorSpeed<-M){
    motorSpeed-M;
  }
  
   m1Speed = M + motorSpeed;
   m2Speed = M - motorSpeed;
   if(motorSpeed<0)
      OrangutanMotors::setSpeeds(m1Speed, M);
   else
      OrangutanMotors::setSpeeds(M, m2Speed);
}

int getPos(){
 return robot.readLine(sensors, IR_EMITTERS_ON);  
}

void moveForward(int spl,int spr){
  OrangutanMotors::setSpeeds(spl, spr);  
}

void motorStop(){
 
 OrangutanMotors::setSpeeds(0, 0);  
}
