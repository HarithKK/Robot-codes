

#define LEFT_MOTOR_REVERSE digitalWrite(3,HIGH);digitalWrite(4,LOW);
#define LEFT_MOTOR_FORWERD digitalWrite(4,HIGH);digitalWrite(3,LOW);
#define RIGHT_MOTOR_REVERSE digitalWrite(5,HIGH);digitalWrite(6,LOW);
#define RIGHT_MOTOR_FORWERD digitalWrite(6,HIGH);digitalWrite(5,LOW);

int read;
int sensorReadings[6];

void setup() {
  initPins();
}

// the loop routine runs over and over again forever:
void loop() {
  
  if(sensorReadings[5] && sensorReadings[4])
    rMotorForward(130);
  if(sensorReadings[0] && sensorReadings[1])
    lMotorForward(130);
  else{
    motorForward(255,255);  
  }
  delay(100);
}


void updateSensorArray(){
  for(int i=0;i<12;i+=2){
    sensorReadings=digitalRead(40-i);
  }
}

void moveForward(int lpwm, int rpwm){
  
  lMotorForward(lpwm);
  rMotorForwrd(rpwm);
  
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
