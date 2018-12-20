int irArray[4];

int maxS = 255, minS = 140, baseSpeed = 175;
int p = 38;
int d = 55;

int count = 0;
short errorValues[] = {-2,-1,1,2};

int leftSpeed, rightSpeed;
int l_error = 0, diff,error;

void setup() {
  Serial.begin(9600);
  delay(200);
  // set sensor
  for(int i=2;i<=5;i++){
      pinMode(i,INPUT);
  }

  //analog write
  pinMode(10, OUTPUT);//leftmotor
  pinMode(11, OUTPUT);//rightmotor
  
  //leftMotor
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  //rightMotor
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  //switch
  pinMode(12,INPUT);

  isPressed();
  delay(1000);
}

void loop() {
 doPID();

 if(getLine()==6){
  motorStop();
  isPressed();
 }
}

void updateSensorArray(){
  for(int i=2;i<=5;i++){
      irArray[i-2]=digitalRead(i);
  }
}

void moveForward(int pwml,int pwmr){
    leftMotorForwrd(pwml);
    rightMotorForwrd(pwmr);
}

void leftMotorForwrd(int pwm){
    digitalWrite(6,LOW);
    digitalWrite(7,HIGH);
    analogWrite(10,pwm); 
}

void rightMotorForwrd(int pwm){
    digitalWrite(8,HIGH);
    digitalWrite(9,LOW);
    analogWrite(11,pwm); 
}

void motorStop(){
    digitalWrite(6,LOW);
    digitalWrite(7,LOW);
    analogWrite(10,0);

    digitalWrite(8,LOW);
    digitalWrite(9,LOW);
    analogWrite(11,0); 
}

void isPressed(){
   int i;
  do{
   i = digitalRead(12);
  }while(i>0);  
}


void readLine(){
 updateSensorArray();
 error = 0;
 count = 0;
 for(int i = 0; i < 4;  i++){
   error += irArray[i] * errorValues[i] ;
   count += irArray[i];
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
       moveForward(0,140);
     else
       moveForward(140,0);
   
   }
   moveForward(baseSpeed,baseSpeed);
 }else{
 
  leftSpeed = baseSpeed + error * p - d * diff;
  rightSpeed = baseSpeed - error * p + d * diff;

  leftSpeed = leftSpeed > maxS ? maxS: leftSpeed ;
  leftSpeed = leftSpeed < minS ? minS : leftSpeed ;

  rightSpeed = rightSpeed > maxS ? maxS: rightSpeed ;
  rightSpeed = rightSpeed < minS ? minS : rightSpeed ;

  moveForward(leftSpeed, rightSpeed);
 }
}

int getLine(){
 updateSensorArray();
 int val=0;
 for(int i = 0; i < 4;  i++){
   val += irArray[i] *i;
 }
 return val;
}

