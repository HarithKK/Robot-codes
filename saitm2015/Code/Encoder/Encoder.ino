int sensorReadings[6];
short error = 0;

int count = 0;
short errorValues[] = {3,2,1,-1,-2,-3};

int maxS = 255, minS = 140, baseSpeed = 190;

int p = 25;
int d = 45;

int leftSpeed, rightSpeed;
int l_error = 0, diff;

int lEncoderCount;
int rEncoderCount;


void setup() {
  initPins();
  initEncoders();
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  
  while(getLEncoderCount()<100){
    doPID();
  }
  
  motorBrake();
  delay(100);
  motorStop();
  
  while(1);

  
}

void initEncoders(){
  
  attachInterrupt(5, lEncoderIncrement, CHANGE);
  attachInterrupt(4, rEncoderIncrement, CHANGE);
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
