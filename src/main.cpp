#include <Arduino.h>
#include <Robot.h>

float dist;
bool first = true;

Robot robot(41,31, 9, 309);

void setup() {
  Serial.begin(9600);
  while(!Serial) {}

  Serial.println("\nInitializing D.A.V.E.");

  robot.setupServo();
  //robot.setupOLED();
  robot.setupMotors();
  robot.setupDistanceSensor();
  robot.setupNavSensor(false);

}

void loop() {
  if (first) {
    robot.orient();
    first = false;
  }

  while (1) {}

  robot.goForward(255);
  dist = robot.readDistanceSensor();

  while (dist > 30) {
    dist = robot.readDistanceSensor();
  }

  robot.stop();

  while (1) {}
}
