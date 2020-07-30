#include <Arduino.h>
#include <Robot.h>

Robot robot(41,31, 9, 355);

void setup() {
  Serial.begin(9600);
  while(!Serial) {}
  Serial.println("");

  robot.setupServo();
  //robot.setupOLED();
  robot.setupMotors();
  //robot.setupDistanceSensor();
  robot.setupNavSensor(false);


}

void loop() {

  robot.orient();

  while(1) {}

}
