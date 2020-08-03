/*
  Robot.c - Library for controling four-wheeled robot on Arduino Mega 2560 with motor shield
  Created by Kevin A. Wilson, July 22, 2020
*/

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU9250_asukiaaa.h>
#include <Robot.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

// Objects
Adafruit_MotorShield motorShield = Adafruit_MotorShield();

Adafruit_DCMotor *motorLF = motorShield.getMotor(1);
Adafruit_DCMotor *motorLR = motorShield.getMotor(2);
Adafruit_DCMotor *motorRR = motorShield.getMotor(3);
Adafruit_DCMotor *motorRF = motorShield.getMotor(4);

Adafruit_DCMotor *motors[4] = { motorLF, motorLR, motorRR, motorRF };

Servo sensorServo;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MPU9250_asukiaaa nav;

// Global Variables
byte _sensorTrig;
byte _sensorEcho;
byte _servoPin;
int currentHeading;
int _referenceNorth;
int _referenceEast;
int _referenceSouth;
int _referenceWest;
float magDecl = 6.9;

Robot::Robot(byte sensorTrig, byte sensorEcho, byte servoPin, int referenceNorth) {
  _sensorTrig = sensorTrig;
  _sensorEcho = sensorEcho;
  _servoPin = servoPin;

  Wire.begin();

  _referenceNorth = referenceNorth;
  _referenceEast = referenceNorth + 90;
  _referenceSouth = referenceNorth + 180;
  _referenceWest = referenceNorth + 270;

  _referenceEast = normalizeCompass(_referenceEast);
  _referenceSouth = normalizeCompass(_referenceSouth);
  _referenceWest = normalizeCompass(_referenceWest);
}

void Robot::setupOLED() {
  //Serial.begin(115200);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
  //   Serial.println(F("SSD1306 allocation failed"));
  //   for(;;);
  // }

  delay(2000);
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(20, 0);
  display.println("D.A.V.E.");
  display.display();
}

void Robot::setupNavSensor(bool cal) {
  float mX;
  float mY;
  float maxX = -100;
  float minX = 100;
  float maxY = -100;
  float minY = 100;
  int i;

  nav.setWire(&Wire);
  nav.beginMag();

  if (cal) {
    Serial.println("Beginning magnetometer calibration. Rotate sensor until values print.");
    delay(3000);
    for (i=0; i<100; i++) {
      nav.magUpdate();
      mX = nav.magX();
      mY = nav.magY();

      Serial.println("X: " + String(mX));
      Serial.println("Y: " + String(mY));

      maxX = (mX > maxX) ? mX : maxX;
      maxY = (mY > maxY) ? mY : maxY;
      minX = (mX < minX) ? mX : minX;
      minY = (mY < minY) ? mY : minY;

      delay(250);
    }

    Serial.println("maxX: " + String(maxX));
    Serial.println("minX: " + String(minX));
    Serial.println("maxY: " + String(maxY));
    Serial.println("minY: " + String(minY));

    float offsetX = (maxX + minX) / 2;
    float offsetY = (maxY + minY) / 2;

    Serial.println("X Offset: " + String(offsetX));
    Serial.println("Y Offset: " + String(offsetY));

    while (1) {}

  } else {
    nav.magXOffset = 4.78;
    nav.magYOffset = -19.13;
  }
}

void Robot::setupMotors() {
  uint8_t i;

  motorShield.begin();

	for (i=0; i<4; i++) {
		motors[i]->setSpeed(0);
	}

	for (i=0; i<4; i++) {
		motors[i]->run(RELEASE);
	}

  Serial.println("Motors set up");
}

void Robot::setupServo() {
  Serial.println("Starting servo.");

	sensorServo.attach(_servoPin);
  sensorServo.write(90);
  delay(500);
  sensorServo.write(60);
  delay(500);
  sensorServo.write(120);
  delay(500);
  sensorServo.write(90);
}

void Robot::setupDistanceSensor() {
  pinMode(_sensorTrig, OUTPUT);
  pinMode(_sensorEcho, INPUT);
  Serial.println("Range sensor online.");
}

void Robot::orient() {
  Serial.println("Orienting robot.");
  int targetHeading;
  float lookLeft;
  float lookRight;
  float distanceLeft;
  float distanceRight;
  int servoLeft;
  int servoRight;

  currentHeading = readNavSensor();

  Serial.print("Current heading: ");
  Serial.println(currentHeading);

  if (currentHeading >= _referenceEast && currentHeading < _referenceSouth) {
    lookLeft = _referenceEast;
    lookRight = _referenceSouth;
  } else if (currentHeading >= _referenceSouth && currentHeading < _referenceWest) {
    lookLeft = _referenceSouth;
    lookRight = _referenceWest;
  } else if (currentHeading >= _referenceWest && currentHeading < _referenceNorth) {
    lookLeft = _referenceWest;
    lookRight = _referenceNorth;
  } else {
    lookLeft = _referenceNorth;
    lookRight = _referenceEast;
  }

  lookLeft += 720;
  lookRight += 720;
  currentHeading += 720;

  servoLeft = (int)(currentHeading - lookLeft);
  servoRight = (int)(currentHeading + lookRight);

  sensorServo.write(90 + servoLeft);
  delay(500);
  distanceLeft = readDistanceSensor();
  delay(1000);

  sensorServo.write(90 - servoRight);
  delay(500);
  distanceRight = readDistanceSensor();
  delay(1000);

  sensorServo.write(90);

  Serial.print("Distance left: ");
  Serial.println(distanceLeft);
  Serial.print("Distance right: ");
  Serial.println(distanceRight);

  // if (currentHeading >= 45 && currentHeading < 135) {
  //   targetHeading = _referenceEast;
  // } else if (currentHeading >= 135 && currentHeading < 225) {
  //   targetHeading = _referenceSouth;
  // } else if (currentHeading >= 225 && currentHeading < 315) {
  //   targetHeading = _referenceWest;
  // } else {
  //   targetHeading = _referenceNorth;
  // }

  if (distanceLeft > distanceRight) {
    targetHeading = lookLeft - 720;
  } else {
    targetHeading = lookRight - 720;
  }



  currentHeading -= 720;
  currentHeading = normalizeCompass(currentHeading);
  targetHeading = normalizeCompass(targetHeading);

  Serial.print("Target heading: ");
  Serial.println(targetHeading);

  rotateToTarget(targetHeading);
}

void Robot::goForward(byte vel) {
  Serial.println("goForward command received.");
  uint8_t i;

  for (i=0; i<4; i++) {
		motors[i]->setSpeed(vel);
	}

  for (i=0; i<4; i++) {
		motors[i]->run(FORWARD);
	}

  motorLF->setSpeed(vel);
  motorLF->run(FORWARD);

}

void Robot::goBackward(byte vel) {

}

void Robot::rotateToTarget(int deg) {
  Serial.println("Rotating to target.");
  bool turn = true;

  currentHeading = readNavSensor();
  currentHeading += 720;
  deg +=720;

  Serial.println(deg);
  Serial.println(currentHeading);

  while (1) {}

  if (currentHeading > deg) {
    turnLeft();
  } else {
    turnRight();
  }

  while (turn) {
    float actualHeading;
    currentHeading = readNavSensor() + 720;

    if (currentHeading > deg - 10 && currentHeading < deg + 10) {
      turn = false;
    } else {
      turn = true;
    }

    actualHeading = normalizeCompass(currentHeading - 720);

    Serial.print("Current heading in turn: ");
    Serial.println(actualHeading);
    delay(100);
  }

  stop();

  currentHeading = readNavSensor();

}

void Robot::turnLeft() {
  byte i;

  for (i=0; i<4; i++) {
		motors[i]->setSpeed(200);
	}

	motors[0]->run(BACKWARD);
  motors[1]->run(BACKWARD);
  motors[2]->run(FORWARD);
  motors[3]->run(FORWARD);
}

void Robot::turnRight() {
  byte i;

  for (i=0; i<4; i++) {
		motors[i]->setSpeed(200);
	}

  motors[0]->run(FORWARD);
  motors[1]->run(FORWARD);
  motors[2]->run(BACKWARD);
  motors[3]->run(BACKWARD);
}

void Robot::stop() {
  uint8_t i;

	for (i=0; i<4; i++) {
		motors[i]->setSpeed(0);
	}

	for (i=0; i<4; i++) {
		motors[i]->run(RELEASE);
	}
}

float * Robot::runFullSensorSweep() {
  static float distances[7];

  byte i;

  for (i=0; i < 7; i++) {
    sensorServo.write(i*30);
    delay(500);
    distances[i] = readDistanceSensor();
  }

  sensorServo.write(90);

  return distances;
};

float Robot::readDistanceSensor() {
  float duration;
  float distance;

  digitalWrite(_sensorTrig, LOW);
	delayMicroseconds(2);
	digitalWrite(_sensorTrig, HIGH);
	delayMicroseconds(10);
	digitalWrite(_sensorTrig, LOW);

	duration = pulseIn(_sensorEcho, HIGH);
	distance = (duration*.0343)/2;

	return distance;
}

float Robot::readNavSensor() {
  float heading;
  int numberOfReadings = 10;
  float headingSum = 0;
  byte i;

  for (i=0; i<numberOfReadings; i++) {
    if (nav.magUpdate() == 0) {
      heading = nav.magHorizDirection();
      headingSum += heading;
    } else {
      Serial.println("Cannot read mag values");
    }
  }

  heading = headingSum / numberOfReadings;
  heading =  heading - 90.0f - magDecl;
  heading = normalizeCompass(heading);

  return heading;
}

float Robot::normalizeCompass(float deg) {

  while (deg > 360 || deg < 0) {
    if (deg < 0) {
      deg += 360;
    } else if (deg > 360) {
      deg -= 360;
    }
  }

  return deg;
}
