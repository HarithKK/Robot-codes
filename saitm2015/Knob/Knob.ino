// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
Servo myservo1;
 //180up
 //109 left
 //65 down

int val;    // variable to read the value from the analog pin 
 
void setup() 
{ 
   Serial.begin(9600);
  myservo.attach(10); 
  myservo1.attach(11);  
  // attaches the servo on pin 9 to the servo object 
  myservo.write(180);
  myservo1.write(0);
  
 
  myservo1.write(105);
   delay(500);
 myservo.write(65);
  delay(500);
  myservo1.write(0);
   delay(500);
   myservo.write(180);
   
   delay(2000);
   
    myservo.write(65);
   delay(500);
 myservo1.write(109);
  delay(500);
  myservo.write(180);
   delay(500);
   myservo1.write(0);
} 
 
void loop() 
{ 
 
   
   while(1);
  if(Serial.available())
  {
    int i=Serial.parseInt();  
    myservo1.write(i);
  }
} 
