#include <AccelStepper.h>

#define X_STEP_PIN 2
#define X_DIR_PIN  5
#define Y_STEP_PIN 4
#define Y_DIR_PIN  9
#define EN_PIN   8

#define MAX_X_SPEED 300
#define MAX_X_ACCEL 100
#define MAX_Y_SPEED 300
#define MAX_Y_ACCEL 100

#define MAX_X_TILT_ANG  11.0
#define MAX_X_MOTOR_ANG 45.0
#define MAX_Y_TILT_ANG  5.0
#define MAX_Y_MOTOR_ANG 30.0

AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

float tiltToMotorX(float tiltX) {
  // Constrain to platform tilt limits
  tiltX = constrain(tiltX, -MAX_X_TILT_ANG, MAX_X_TILT_ANG);
  
  // Scale and return motor angle
  return tiltX * MAX_X_MOTOR_ANG / MAX_X_TILT_ANG;
}

float tiltToMotorY(float tiltY) {
  // Constrain to platform tilt limits
  tiltY = constrain(tiltY, -MAX_Y_TILT_ANG, MAX_Y_TILT_ANG);
  
  // Scale and return motor angle
  return tiltY * MAX_Y_MOTOR_ANG / MAX_Y_TILT_ANG;
}

const int stepsPerRev = 200;   // 1.8° motor

// Converts angle to step position and moves there
void moveToAngX(float angleDeg) {
  long steps = (long)stepsPerRev * angleDeg / 360.0;
  stepperX.moveTo(steps);
  // blocks until done
  stepperX.runToPosition();
}

void moveToAngY(float angleDeg) {
  long steps = (long)stepsPerRev * angleDeg / 360.0;
  stepperY.moveTo(steps);
  // blocks until done
  stepperY.runToPosition();
}

void setup() {
  Serial.begin(9600);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  // Limit max speeds and accelerations
  stepperX.setMaxSpeed(MAX_X_SPEED);
  stepperX.setAcceleration(MAX_X_ACCEL);
  stepperY.setMaxSpeed(MAX_Y_SPEED);
  stepperY.setAcceleration(MAX_Y_ACCEL);

  // Define zero, current motor position
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  delay(5000);
}

void loop() {
  float tiltAngX = 6;
  float tiltAngY = 2.5;
  moveToAngX(tiltToMotorX(tiltAngX));
  Serial.println(tiltToMotorX(tiltAngX));
  moveToAngY(tiltToMotorY(tiltAngY));
  Serial.println(tiltToMotorY(tiltAngY));
  delay(1000);

  moveToAngX(0);
  moveToAngY(0);
  delay(1000);

  moveToAngX(45);
  moveToAngY(30);
  Serial.println("X: 15");
  delay(1000);

  moveToAngY(-30);
  Serial.println("X: 0");
  delay(1000);

  moveToAngX(-45);
  Serial.println("X: 15");
  delay(1000);

  moveToAngY(30);
  Serial.println("X: 0");
  delay(1000);
}

