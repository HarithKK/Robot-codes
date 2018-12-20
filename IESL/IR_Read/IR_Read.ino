int lEncoderCount;
int rEncoderCount;



void setup() {
  // motor Pins
  for(int i=0;i<6;i++){
    pinMode(2+i,OUTPUT);
  }

  attachInterrupt(5, lEncoderIncrement, CHANGE);
  attachInterrupt(4, rEncoderIncrement, CHANGE);

   Serial.begin(9600);
   delay(100);
}

void loop() {
  Serial.print(getLowerIrRead());
  Serial.print(",");
  Serial.println(getHigherIrRead());
  delay(200);
}

void moveForward(int lpwm, int rpwm){
  
  lMotorForward(lpwm);
  rMotorForward(rpwm);
  
}

void rMotorForward(int pwm){
  digitalWrite(4,HIGH);
  digitalWrite(3,LOW);
  analogWrite(7, pwm);
}

void lMotorForward(int pwm){
  digitalWrite(6,HIGH);
  digitalWrite(5,LOW);
  analogWrite(2, pwm);
}

void motorStop(){
    digitalWrite(3,LOW);
    digitalWrite(4,LOW);
    analogWrite(2, 0);
    digitalWrite(6,LOW);
    digitalWrite(5,LOW);
    analogWrite(7, 0);
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

int getLowerIrRead(){
  return analogRead(A0);  
}

int getHigherIrRead(){
  return analogRead(A1);  
}
