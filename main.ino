#include <SoftwareSerial.h>
#include <AccelStepper.h>

// UART Pin definitions
#define ftcUartRx 10
#define ftcUartTx 11

// Motor pins definitions
#define X_STEP_PIN 2
#define X_DIR_PIN  5
#define Y_STEP_PIN 4
#define Y_DIR_PIN  9
#define EN_PIN   8

// Coordinate value definitions
const int MIN_X_VAL = 20;
const int MAX_X_VAL = 210;
const int MIN_Y_VAL = 148;
const int MAX_Y_VAL = 248;
const int X_TARGET_VAL = 139;
const int Y_TARGET_VAL = 185;

// PID definitions
float prevDervXError = 0;
float prevDervYError = 0;
unsigned long prevTime = 0;
const int MAX_D_CHANGE = 4000;

// PID constants
float kpx = 0.2;
float kdx = 0.01;
float kpy = 0.2;
float kdy = 0.01;

// Motor constants
#define MAX_X_SPEED 300
#define MAX_X_ACCEL 100
#define MAX_Y_SPEED 300
#define MAX_Y_ACCEL 100

#define MAX_X_TILT_ANG  11.0
#define MAX_X_MOTOR_ANG 45.0
#define MAX_Y_TILT_ANG  5.0
#define MAX_Y_MOTOR_ANG 30.0
const int stepsPerRev = 200;   // 1.8° motor

SoftwareSerial ftcUART(ftcUartRx, ftcUartTx);

AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

// Heavy median and exponential for X
#define X_MEDIAN_SIZE 9

byte x_buffer[X_MEDIAN_SIZE] = {0};
int x_index = 0;
float filtered_x_ema = 127.0;
float alpha_x = 0.25;

float filtered_y = 191.0;
float alpha_y = 0.4;

// Flag for new data so prev arnt affecting filters
bool newDataReady = false;

// Map tilt angle to motor angle
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

// Converts angle to step position and moves there
void moveToAngX(float angleDeg) {
  long steps = (long)stepsPerRev * angleDeg / 360.0;
  stepperX.moveTo(steps);
  // blocks until done
}

void moveToAngY(float angleDeg) {
  long steps = (long)stepsPerRev * angleDeg / 360.0;
  stepperY.moveTo(steps);
  // blocks until done
}

void setup() {
  Serial.begin(9600);
  ftcUART.begin(9600);
  // Take time for start of loop for first dervivative measurement
  prevTime = micros();

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
  // Delay for manual homeing
  delay(10000);
}

void loop() {
  // UART packets
  static byte packet[4];
  static int packetIndex = 0;

  // ============================================================================
  // READ AND FILTER TOUCHSCREEN DATA
  // ============================================================================
  // Run while UART data available in buffer
  while (ftcUART.available() > 0) {
    byte uartByte = ftcUART.read();
    packet[packetIndex++] = uartByte;
    
    // Store x and y values when you read delimiter, 0 at end of packet
    if (packetIndex == 4) {
      if (packet[3] == 0) {
        byte raw_x = packet[1];
        byte raw_y = packet[2];
        
        // X filtering
        if (raw_x >= MIN_X_VAL && raw_x <= MAX_X_VAL) {
          x_buffer[x_index] = raw_x;
          x_index = (x_index + 1) % X_MEDIAN_SIZE;
        }
        
        // Y filtering
        if (raw_y >= MIN_Y_VAL && raw_y <= MAX_Y_VAL) {
          filtered_y = alpha_y * raw_y + (1.0 - alpha_y) * filtered_y;
        }
        
        // Get median X and apply exponential
        byte median_x = getMedianX(x_buffer);
        filtered_x_ema = alpha_x * median_x + (1.0 - alpha_x) * filtered_x_ema;
        
        // Flag that we have new data so not repeating for no new data
        newDataReady = true;
      }
      
      packetIndex = 0;
    }
  }
  
  // ============================================================================
  // RUN PID ONLY WHEN NEW DATA ARRIVES
  // ============================================================================
  if (newDataReady) {
    newDataReady = false;
    
    // Calculate errors
    int xError = X_TARGET_VAL - (int)filtered_x_ema;
    int yError = Y_TARGET_VAL - (int)filtered_y;
    
    // Change in time since last measurement
    unsigned long timeNow = micros();
    float dt = (timeNow - prevTime) * 0.000001;
    prevTime = timeNow;
    
    // Prevent division by zero or too small dt
    if (dt < 0.001) {
      dt = 0.001;
    }
    
    // Derivative of error
    float dervXError = (xError - prevDervXError) / dt;
    float dervYError = (yError - prevDervYError) / dt;
    
    // Constrain derivative so no blowing up
    dervXError = constrain(dervXError, -MAX_D_CHANGE, MAX_D_CHANGE);
    dervYError = constrain(dervYError, -MAX_D_CHANGE, MAX_D_CHANGE);
    
    // Update previous errors
    prevDervXError = xError;
    prevDervYError = yError;
    
    // Calculate tilt angles
    float xTiltAngle = kpx * xError + kdx * dervXError;
    float yTiltAngle = kpy * yError + kdy * dervYError;
    
    // Debug output
    Serial.print("X: ");
    Serial.print((int)filtered_x_ema);
    Serial.print("\tY: ");
    Serial.print((int)filtered_y);
    Serial.print("\tErr X: ");
    Serial.print(xError);
    Serial.print("\tErr Y: ");
    Serial.print(yError);
    Serial.print("\tTilt X: ");
    Serial.print(xTiltAngle);
    Serial.print("\tTilt Y: ");
    Serial.println(yTiltAngle);
    
    // ============================================================================
    // 3. MOTOR CONTROL
    // ============================================================================
    // Move motor to new tilt
    moveToAngX(-tiltToMotorX(xTiltAngle));
    moveToAngY(-tiltToMotorY(yTiltAngle));
  }
  // Run steppers to adjust angle if new value
  stepperX.run();
  stepperY.run();
}

// Median filter fxn
byte getMedianX(byte* buffer) {
  byte tempArr[X_MEDIAN_SIZE];
  memcpy(tempArr, buffer, X_MEDIAN_SIZE);
  
  // Bubble sort
  for (int i = 0; i < X_MEDIAN_SIZE - 1; i++) {
    for (int j = 0; j < X_MEDIAN_SIZE - i - 1; j++) {
      if (tempArr[j] > tempArr[j + 1]) {
        byte swap = tempArr[j];
        tempArr[j] = tempArr[j + 1];
        tempArr[j + 1] = swap;
      }
    }
  }
  
  return tempArr[X_MEDIAN_SIZE / 2];
}