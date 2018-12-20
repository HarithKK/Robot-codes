/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

 This example code is in the public domain.
 */

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  delay(100);
  for(int i=9;i<=12;i++)
    pinMode(i,INPUT);
}

// the loop routine runs over and over again forever:
void loop() {

  for(int i=9;i<=12;i++){
    Serial.print(digitalRead(i));
    Serial.print(",");  
  }
  Serial.println();
  delay(100);
  
}
