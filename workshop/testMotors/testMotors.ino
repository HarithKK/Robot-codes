#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanMotors.h>
#include <OrangutanAnalog.h>
#include <OrangutanLEDs.h>
#include <OrangutanLCD.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanBuzzer.h>

Pololu3pi robot;
OrangutanPushbuttons buttons;
OrangutanLCD lcd;

unsigned int sensors[5]; 
unsigned int counter;

void setup() {
 robot.init(2000);
 OrangutanPushbuttons::waitForRelease(BUTTON_B);
 lcd.clear();
 lcd.print("calibrate");
 buttons.waitForPress(BUTTON_B);
 delay(1000);
 for (counter=0; counter<40; counter++)
  {
    if (counter <= 20 || counter >= 40)
      OrangutanMotors::setSpeeds(40, -40);
    else
      OrangutanMotors::setSpeeds(-40, 40);
    robot.calibrateLineSensors(IR_EMITTERS_ON);

    delay(20);
  }
  for (counter=0; counter<40; counter++)
  {
    if (counter <= 20 || counter >= 40)
      OrangutanMotors::setSpeeds(-40, 40);
    else
      OrangutanMotors::setSpeeds(40, -40);
    robot.calibrateLineSensors(IR_EMITTERS_ON);

    delay(20);
  }
  OrangutanMotors::setSpeeds(0, 0);
  lcd.clear();
  lcd.print("OK 1");
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);
    lcd.clear();
    lcd.print(position);
    delay(100);
  }
  OrangutanPushbuttons::waitForRelease(BUTTON_B);
}

void loop() {
  // put your main code here, to run repeatedly:

}
