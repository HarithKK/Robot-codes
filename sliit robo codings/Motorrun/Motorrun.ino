

void setup() {

// set Motor PINS  
  for(int i=2;i<=7;i++){
    pinMode(i,OUTPUT);
  }

  
}

// the loop routine runs over and over again forever:
void loop() {
  
  for(int i=255;i>30;i--){
    lMotorForward(i);
  rMotorForward(i);
  delay(30);
    }
  //while(1);
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
