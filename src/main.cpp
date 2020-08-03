#include <Arduino.h>
#include <Robot.h>

float dist;
bool first = true;

Robot robot(41,31, 9, 355);

void setup() {
  Serial.begin(9600);
  while(!Serial) {}

  Serial.println("\nInitializing D.A.V.E.");

  //robot.setupServo();
  //robot.setupOLED();
  //robot.setupMotors();
  //robot.setupDistanceSensor();
  robot.setupNavSensor(false);

}

void loop() {
  float heading;

  heading = robot.readNavSensor();

  Serial.println(heading);

  delay(500);

  // if (first) {
  //   robot.orient();
  //   first = false;
  // }
  //
  // robot.goForward(255);
  // dist = robot.readDistanceSensor();
  //
  // while (dist > 30) {
  //   dist = robot.readDistanceSensor();
  // }
  //
  // robot.stop();
  //
  // while (1) {}
}
