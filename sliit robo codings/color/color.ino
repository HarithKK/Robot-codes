//int s0=39,s1=37,s2=33,s3=31;
//int out_=21;
int s0=37,s1=35,s2=39,s3=41;
int out_=3;
int flag_=0;
int counter_=0;
int countR=0,countG=0,countB=0;
void setup()
 {
 Serial.begin(9600);
 pinMode(s0,OUTPUT);
 pinMode(s1,OUTPUT); 
 pinMode(s2,OUTPUT);
 pinMode(s3,OUTPUT);
 }
void TCS()
 {
   digitalWrite(s1,HIGH);
   digitalWrite(s0,LOW);
   flag_=0;
   attachInterrupt(0, ISR_INTO, CHANGE);
   timer2_init();
 }
void ISR_INTO()
 {
   counter_++;
 }
 void timer2_init(void)
 {
   TCCR2A=0x00;
   TCCR2B=0x07; //the clock frequency source 1024 points
   TCNT2= 100;    //10 ms overflow again
   TIMSK2 = 0x01; //allow interrupt
 }
 int i=0;
 ISR(TIMER2_OVF_vect)//the timer 2, 10ms interrupt overflow again. Internal overflow interrupt executive function
{
 TCNT2=100;
 flag_++;
 if(flag_==1)
  {
    counter_=0;
  }
 else if(flag_==2)
   {
    digitalWrite(s2,LOW);
    digitalWrite(s3,LOW); 
    countR=counter_/1.051;
    digitalWrite(s2,HIGH);
    digitalWrite(s3,HIGH);   
   }
 else if(flag_==3)
    {
     countG=counter_/1.0157;
     digitalWrite(s2,LOW);
     digitalWrite(s3,HIGH); 
   
    }
 else if(flag_==4)
    {
     countB=counter_/1.114;
     digitalWrite(s2,LOW);
     digitalWrite(s3,LOW);
     }
 else
     {
      
     Serial.println();
     flag_=0; 
      TIMSK2 = 0x00;
     }
     counter_=0;
     delay(2);
}
void loop()
 {
  delay(10);
  TCS();
   Serial.print("red");
   Serial.print(countR,DEC);
   Serial.print("\t");

   Serial.print("Green");
   Serial.print(countG,DEC);
   Serial.print("\t");

   Serial.print("blue");
   Serial.print(countB,DEC);
   Serial.print("\t");

   Serial.println();
   delay(500);
//  if((countR>10)||(countG>10)||(countB>10))
//   {
//      if((countR>countG)&&(countR>countB))
//       {
//            Serial.print("red");
//            Serial.print("\n");
//            delay(1000);
//       }
//      else if((countG>=countR)&&(countG>countB))
//       {
//            Serial.print("green");
//            Serial.print("\n");
//            delay(1000);
//       } 
//     else if((countB>countG)&&(countB>countR))
//      {
//            Serial.print("blue");
//            Serial.print("\n");
//           delay(1000);
//      }
//    }
//  else 
//  {
//     delay(1000);       
//  }
 }
