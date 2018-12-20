int irArray[4];

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

  
}

void loop() {
  
}

void updateSensorArray(){
  for(int i=2;i<=5;i++){
      irArray[i-2]=digitalRead(i);
  }
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
