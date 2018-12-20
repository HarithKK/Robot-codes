#include<math.h>

#define LEFT_MOTOR_REVERSE digitalWrite(3,HIGH);digitalWrite(4,LOW);
#define LEFT_MOTOR_FORWERD digitalWrite(4,HIGH);digitalWrite(3,LOW);
#define RIGHT_MOTOR_REVERSE digitalWrite(5,HIGH);digitalWrite(6,LOW);
#define RIGHT_MOTOR_FORWERD digitalWrite(6,HIGH);digitalWrite(5,LOW);

int read;
int irArray[6];

void setup() {
  Serial.begin(9600);
// set Motor PINS  
  for(int i=0;i<6;i++){
    pinMode(2+i,OUTPUT);
  }

//set SensorPanel Pins
  for(int i=0;i<12;i+=2){
    pinMode(30+i,INPUT);
  }
  
}

// the loop routine runs over and over again forever:
void loop() {
  updateSensorArray();
  for(int i=0;i<12;i+=2){
    Serial.print(irArray[i/2]);
    Serial.print(" ");
  }
  Serial.println(" ");
  delay(100);
}

void updateSensorArray(){
  for(int i=0;i<12;i+=2){
    irArray[i/2]=digitalRead(40-i);
  }
}
