#define LEFT_PLUS 4
#define LEFT_MINUS 3
#define LEFT_PWM 2

#define RIGHT_PLUS 5
#define RIGHT_MINUS 6
#define RIGHT_PWM 7

#define RH_ENCODER_A 21 
#define RH_ENCODER_B 20
#define LH_ENCODER_A 18
#define LH_ENCODER_B 19

volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;


void setup() {

  // set Motor PINS  
  for(int i=0;i<6;i++){
    pinMode(2+i,OUTPUT);
  }
  // Encorder
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  
  // initialize hardware interrupts
  attachInterrupt(4, leftEncoderEvent, CHANGE);
  attachInterrupt(2, rightEncoderEvent, CHANGE);

  Serial.begin(9600);
  
}

void loop() {
  if(leftCount >5000)
  {  motorStop();
     while(1);
  }
 rMotorForward(255);
 lMotorForward(255);
  Serial.print("Right Count: ");
  Serial.println(rightCount);
  Serial.print("Left Count: ");
  Serial.println(leftCount);
  Serial.println();
  
}

void leftEncoderEvent() {
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount++;
    } else {
      leftCount--;
    }
  } else {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  }
}
 
void rightEncoderEvent() {
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  }
}

void motorForward(int pwml,int pwmr){
  rMotorForward(pwmr);
  lMotorForward(pwml);  
}

void motorReverse(int pwml,int pwmr){
  rMotorReverse(pwmr);
  lMotorReverse(pwml);  
}

void lMotorForward(int pwm){
  digitalWrite(LEFT_PLUS,HIGH);
  digitalWrite(LEFT_MINUS,LOW);
  analogWrite(LEFT_PWM,pwm);
}

void rMotorForward(int pwm){
  digitalWrite(RIGHT_PLUS,HIGH);
  digitalWrite(RIGHT_MINUS,LOW);
  analogWrite(RIGHT_PWM, pwm);
}

void lMotorReverse(int pwm){
  digitalWrite(LEFT_PLUS,LOW);
  digitalWrite(LEFT_MINUS,HIGH);
  analogWrite(LEFT_PWM,pwm);
}

void rMotorReverse(int pwm){
  digitalWrite(RIGHT_PLUS,LOW);
  digitalWrite(RIGHT_MINUS,HIGH);
  analogWrite(RIGHT_PWM, pwm);
}

void motorStop(){
  
  digitalWrite(RIGHT_PLUS,HIGH);
  digitalWrite(RIGHT_MINUS,HIGH);
  analogWrite(RIGHT_PWM, 0);
  digitalWrite(LEFT_PLUS,HIGH);
  digitalWrite(LEFT_MINUS,HIGH);
  analogWrite(LEFT_PWM,0);
  delay(10);
  digitalWrite(RIGHT_PLUS,LOW);
  digitalWrite(RIGHT_MINUS,LOW);
  digitalWrite(LEFT_PLUS,LOW);
  digitalWrite(LEFT_MINUS,LOW);
  
}
