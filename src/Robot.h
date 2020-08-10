/*
  Robot.h - Library for controling four-wheeled robot on Arduino Mega 2560 with motor shield
  Created by Kevin A. Wilson, July 22, 2020
*/

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

class Robot
{
  public:
    Robot(byte, byte, byte, int);

    void setupServo();
    void setupOLED();
    void setupMotors();
    void setupDistanceSensor();
    void setupNavSensor(bool);

    void orient();
    void goForward(byte);
    void goBackward(byte);
    void rotateToTarget(int);
    void turnLeft();
    void turnRight();
    void stop();

    float * runFullSensorSweep();
    float readDistanceSensor();
    float readNavSensor();
    float selectDirection();

    int currentHeading;

  private:
    float normalizeCompass(float);

    int _referenceNorth;
    int _referenceEast;
    int _referenceSouth;
    int _referenceWest;
    byte _sensorTrig;
    byte _sensorEcho;
    byte _servoPin;

};
