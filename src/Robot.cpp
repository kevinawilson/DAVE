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
#include <MPU9250.h>
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

MPU9250 nav(Wire,0x68);

// Global Variables
byte _sensorTrig;
byte _sensorEcho;
byte _servoPin;
int currentHeading;
int _referenceNorth;
int _referenceEast;
int _referenceSouth;
int _referenceWest;
float magDecl = 10.91;

Robot::Robot(byte sensorTrig, byte sensorEcho, byte servoPin, int referenceNorth) {
  _sensorTrig = sensorTrig;
  _sensorEcho = sensorEcho;
  _servoPin = servoPin;

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
  int status;

  status = nav.begin();
  nav.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_10HZ);
  nav.setSrd(19);
  nav.enableDataReadyInterrupt();

  if (status < 0) {
    Serial.println("Navigation sensor initialization unsuccessful.");
    Serial.println("Check sensor wiring or try cycling power.");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  } else {
    Serial.println("Navigation sensors online.");
  }

  if (cal) {
    Serial.println("----------");

    Serial.println("Calibrating magnetometer. Move the sensor in a figure 8 while calibration takes place.");
    status = nav.calibrateMag();

    if (status > 0 ) {
      Serial.println("Magnetomer calibration complete.");

      float hxb;
      hxb = nav.getMagBiasX_uT();

      float hxs;
      hxs = nav.getMagScaleFactorX();

      float hyb;
      hyb = nav.getMagBiasY_uT();

      float hys;
      hys = nav.getMagScaleFactorY();

      float hzb;
      hzb = nav.getMagBiasZ_uT();

      float hzs;
      hzs = nav.getMagScaleFactorZ();

      Serial.print("MagBiasX: ");
      Serial.println(hxb);

      Serial.print("MagBiasY: ");
      Serial.println(hyb);

      Serial.print("MagBiasZ: ");
      Serial.println(hzb);

      Serial.println("");

      Serial.print("MagScaleX: ");
      Serial.println(hxs);

      Serial.print("MagScaleY: ");
      Serial.println(hys);

      Serial.print("MagScaleZ: ");
      Serial.println(hzs);

    } else {
      Serial.println("Magnetometer calibration unsuccessful.");
    }

    Serial.println("----------");

    Serial.println("Preparing to calibrate gyroscope. Robot should be motionless during calibration.");
    delay(3000);
    Serial.println("Calibrating gyroscope.");

    status = nav.calibrateGyro();

    if (status > 0 ) {
      Serial.println("Gyroscope calibration complete.");

      float gxb;
      gxb = nav.getGyroBiasX_rads();

      float gyb;
      gyb = nav.getGyroBiasY_rads();

      float gzb;
      gzb = nav.getGyroBiasZ_rads();

      Serial.print("GyroBiasX: ");
      Serial.println(gxb);

      Serial.print("GyroBiasY: ");
      Serial.println(gyb);

      Serial.print("GyroBiasZ: ");
      Serial.println(gzb);

    } else {
      Serial.println("Gyroscope calibration unsuccessful.");
    }

    Serial.println("----------");

    Serial.println("Calibrating accelerometer.");
    status = nav.calibrateAccel();

    if (status > 0 ) {
      Serial.println("Accelerometer calibration complete.");

      float axb;
      axb = nav.getAccelBiasX_mss();

      float axs;
      axs = nav.getAccelScaleFactorX();

      float ayb;
      ayb = nav.getAccelBiasY_mss();

      float ays;
      ays = nav.getAccelScaleFactorY();

      float azb;
      azb = nav.getAccelBiasZ_mss();

      float azs;
      azs = nav.getAccelScaleFactorZ();

      Serial.print("AccelBiasX: ");
      Serial.println(axb);

      Serial.print("AccelBiasY: ");
      Serial.println(ayb);

      Serial.print("AccelBiasZ: ");
      Serial.println(azb);

      Serial.println("");

      Serial.print("AccelScaleX: ");
      Serial.println(axs);

      Serial.print("AccelScaleY: ");
      Serial.println(ays);

      Serial.print("AccelScaleZ: ");
      Serial.println(azs);

    } else {
      Serial.println("Accelerometer calibration unsuccessful.");
    }


  } else {

    nav.setMagCalX(-12.41, 0.93);
    nav.setMagCalY(10.47, 1.05);
    nav.setMagCalZ(11.25, 1.03);

    Serial.println("   Magnetometer calibration loaded.");

    nav.setGyroBiasX_rads(0.00);
    nav.setGyroBiasY_rads(-0.01);
    nav.setGyroBiasX_rads(0.02);

    Serial.println("   Gyroscope calibration loaded.");

    nav.setAccelCalX(0, 1);
    nav.setAccelCalY(0, 1);
    nav.setAccelCalZ(0, 1);

    Serial.println("   Accelerometer calibration loaded.");
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
  float aX;
  float aY;
  float aZ;
  float normalizer;
  float yaw_rad;
  float heading;
  float numReadings = 100;
  float multiple_readings = 0;
  byte i;

  for (i=0; i < numReadings; i++) {

    nav.readSensor();

    aX = nav.getMagX_uT();
    aY = nav.getMagY_uT();
    aZ = nav.getMagZ_uT();

    normalizer = sqrtf(aX * aX + aY * aY + aZ * aZ);
    aX /= normalizer;
    aY /= normalizer;
    aZ /= normalizer;

    yaw_rad = atan2f(-aY, aX);
    yaw_rad = fmod(yaw_rad, 2.0 * PI);

    if (yaw_rad < 0.0) {
      yaw_rad += 2.0 * PI;
    }

    multiple_readings += yaw_rad * 180.0f / PI;
  }

  heading = multiple_readings / numReadings;
  heading -=  magDecl;
  heading = normalizeCompass(heading);

  currentHeading = heading;
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
