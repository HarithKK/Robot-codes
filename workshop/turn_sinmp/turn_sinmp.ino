#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanMotors.h>
#include <OrangutanAnalog.h>
#include <OrangutanLEDs.h>
#include <OrangutanLCD.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanBuzzer.h>

#define PASS_DELAY 200
#define IS_BLACK(p) p>400
#define IS_WHITE(p) p<100

Pololu3pi robot;
OrangutanPushbuttons buttons;
OrangutanLCD lcd;
OrangutanBuzzer buzzer;

unsigned int sensors[5]; 
unsigned int sensorsT[5]; 
unsigned int counter;
char pathTurn[100];
int turnCount=0;

float KP=0.05;
float KD=3/2;
float KI=1/10000;
int lastError=0;
int integral=0;
int M=60;
int y=0;

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
    playS();
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);
    lcd.clear();
    lcd.print(position);
    delay(100);
  }
  OrangutanPushbuttons::waitForRelease(BUTTON_B);
  delay(1000);
  playS();

}

void loop() {
  while(1){
   doPID();

    if(getPos()==0){
      turnMode='B';
      delay(2);
      break;
    }
    if (IS_BLACK(sensors[4]))  //right || cross
    {
      turnMode='R';
      break;
    }else
    if (IS_BLACK(sensors[0]))  //right || cross
    {
      turnMode='S';
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
          endComes();
        }else{
          pathTurn[turnCount++]='R';
          turnRight();
        }
      
      };break;
    case 'L':  turnLeft();break;
    case 'B':  {pathTurn[turnCount++]='B';turnBack();};break;
    case 'E':  endComes();break;
    case 'S':  {
       robot.readLine(sensorsT, IR_EMITTERS_ON);
        if(IS_WHITE(sensorsT[3])){
          pathTurn[turnCount++]='L';
          turnLeft();
        }else{
          pathTurn[turnCount++]='S';  
        }
      }break;
  }
  simplify_path();
  OrangutanMotors::setSpeeds(50, 50);
  delay(PASS_DELAY);

  
}

void turnLeft(){
    motorStop();
    playS();
    OrangutanMotors::setSpeeds(-100, +100);
    delay(137);
    motorStop();
}

void turnRight(){
    motorStop();
    playS();
    OrangutanMotors::setSpeeds(+100,-100);
    delay(137);
    motorStop();
}

void turnBack(){
    motorStop();
    playS();
    OrangutanMotors::setSpeeds(100, -100);
    delay(278);
    motorStop();
}

void endComes(){
     motorStop();
     playS();
     lcd.clear();
     int i=0;
     for(i=0;i<turnCount;i++)
      lcd.print(pathTurn[i]);
     
     delay(100);

     //send signal

     Serial.begin(38400);
     delay(100);
     
     for(i=0;i<turnCount;i++)
      Serial.print(pathTurn[i]);

     lcd.clear();
     lcd.print("Done");
     Serial.print('A');

     while(1);
//     while(1){
//     while (!OrangutanPushbuttons::isPressed(BUTTON_B));
//     OrangutanPushbuttons::waitForRelease(BUTTON_B);
//      delay(1000);
//      lcd.clear();
//      lcd.print("Start");
//      goShort();
//      y=0;
//     }
     
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

void goShort(){
  getTurn();
  moveForward(50,50);
  delay(PASS_DELAY);
  lcd.clear();
  lcd.print("Finish");
 while(1){
   doPID();
    if (IS_BLACK(sensors[4]))  //right || cross
    {
      turnMode='R';
      break;
    }
  }
  
  OrangutanMotors::setSpeeds(50, 50);
  delay(100);
  motorStop();
  
  switch(turnMode){
    case 'R':  {
        robot.readLine(sensorsT, IR_EMITTERS_ON);
        if(IS_BLACK(sensorsT[0])){
          OrangutanMotors::setSpeeds(50, 50);
          delay(300);
          motorStop();
          lcd.clear();
          lcd.print("Turn OK");
        }
      
      };break;
    
  }
  
  

}

void getTurn(){
int j=0;
for(j=0;j<turnCount;j++){
   while(1){
    doPID();

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

  moveForward(50,50);
  delay(60);
  motorStop();
  
  lcd.clear();
  lcd.print(j);
  lcd.print(pathTurn[j]);
  
  robot.readLine(sensorsT, IR_EMITTERS_ON);
  moveForward(50,50);
  delay(100);
  motorStop();
  
  if (IS_BLACK(sensorsT[4]))  //right || cross
    {
      turn();
    }else
    if (IS_BLACK(sensorsT[0]))  //right || cross
    {
      turn();
    }
  
  
}  
}

void turn(){
     motorStop();
     playS();
    switch(pathTurn[y++]){
      case 'R': turnRight();break;
      case 'L': turnLeft();break;
      case 'S': {OrangutanMotors::setSpeeds(50, 50);delay(300);}break; 
    }  
}


void simplify_path()
{
  if (turnCount < 3 || pathTurn[turnCount-2] != 'B')
    return;

  int total_angle = 0;
  int i;
  for (i = 1; i <= 3; i++)
  {
    switch (pathTurn[turnCount - i])
    {
    case 'R':
      total_angle += 90;
      break;
    case 'L':
      total_angle += 270;
      break;
    case 'B':
      total_angle += 180;
      break;
    }
  }

  total_angle = total_angle % 360;

  switch (total_angle)
  {
  case 0:
    pathTurn[turnCount - 3] = 'S';
    break;
  case 90:
    pathTurn[turnCount - 3] = 'R';
    break;
  case 180:
    pathTurn[turnCount - 3] = 'B';
    break;
  case 270:
    pathTurn[turnCount - 3] = 'L';
    break;
  }

  turnCount -= 2;
  
}

void playS(){
  buzzer.playNote(NOTE_G(5), 30, 15);
  
}
