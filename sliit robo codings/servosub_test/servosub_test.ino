#define MAX_MAIN_SERVO_POSITION 20
#define MIN_MAIN_SERVO_POSITION 200
#define INT_MAIN_SERVO_POSITION 160
#define MAX_SUB_SERVO_POSITION 150
#define MIN_SUB_SERVO_POSITION 43


#include <Servo.h> 
 
Servo mainS; 
Servo subS;

void setup() 
{ 

  mainS.attach(10);
  subS.attach(11);

  // initialize
  centerPosition();   // get center positon
  delay(1000);
}
 
void loop() 
{ 
  getBoxMax();
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

void getBoxInt(){

  subS.write(MAX_SUB_SERVO_POSITION); // open hatch
  delay(700);
  mainS.write(MIN_MAIN_SERVO_POSITION); //go down
  delay(1000);
  subS.write(MIN_SUB_SERVO_POSITION);   //close hatch
  delay(700);
  mainS.write(INT_MAIN_SERVO_POSITION); // go up to int
  delay(1000);
  
}

void releaseBox(){
  mainS.write(MIN_MAIN_SERVO_POSITION); //go down
  delay(1000);
  subS.write(MAX_SUB_SERVO_POSITION); // open hatch
  delay(700);
  mainS.write(MAX_MAIN_SERVO_POSITION); // go up to int
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


