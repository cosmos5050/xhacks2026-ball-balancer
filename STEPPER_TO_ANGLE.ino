#include <AccelStepper.h>

#define STEP_PIN 2
#define DIR_PIN  5
#define EN_PIN   8

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

const int stepsPerRev = 200;   // 1.8° motor

// Converts angle (deg) to step position and moves there
void moveToAngle(float angleDeg) {
  long steps = stepsPerRev * angleDeg / 360.0;
  stepper.moveTo(steps);
  stepper.runToPosition();   // blocks until done
}

void setup() {
  Serial.begin(9600);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  stepper.setMaxSpeed(300);
  stepper.setAcceleration(100);

  stepper.setCurrentPosition(0);  // define zero
}

void loop() {
  moveToAngle(270);
  Serial.println("Degrees: 270");
  delay(1000);

  moveToAngle(180);
  Serial.println("Degrees: 180");
  delay(1000);

  Serial.println("Restarting");
  Serial.println();
  delay(2000);
}
