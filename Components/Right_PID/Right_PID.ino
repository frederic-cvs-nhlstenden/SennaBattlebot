#include <Arduino.h>

// ----------------- ARDUINO CONSTANTS -----------------
const int MOTOR_LEFT_FWD  = 10; // A2: left motor forward
const int MOTOR_LEFT_REV  = 11; // A1: left motor reverse
const int MOTOR_RIGHT_FWD = 6;  // B1: right motor forward
const int MOTOR_RIGHT_REV = 5;  // B2: right motor reverse

const int TRIG_PIN_RIGHT  = 4;  // right ultrasonic trig
const int ECHO_PIN_RIGHT  = 8;  // right ultrasonic echo

// Target distance from right wall in cm
const int RIGHT_TARGET = 5;   

// Base forward speed (0-255)
int baseForwardSpeed = 150;

// Proportional gain for the right-wall PID
float Kp_right = 10.0;

// Minimum PWM to avoid stalling
const int MIN_PWM = 75;

// Debug function to test logic between forward and backward mode
// setting this to 'true' means: error => -(distRight - RIGHT_TARGET)
bool flipPidSign = true;

// ---------------------------------------------------

// Measuring distance via ultrasonic
long measureDistanceRight() {
  digitalWrite(TRIG_PIN_RIGHT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_RIGHT, LOW);

  long duration = pulseIn(ECHO_PIN_RIGHT, HIGH, 30000); //
  if (duration == 0) {
    // No echo => out of range
    return -1;
  }
  // Convert microseconds to cm
  return (long)(duration * 0.034 / 2.0);
}

// Clamping speeds so we don't stall and we don't get any unexpected behavior
int clampPWM(int val) {
  if (val < 0)   val = 0; 
  if (val > 255) val = 255;
  if (val > 0 && val < MIN_PWM) {
    val = MIN_PWM;
  }
  return val;
}

// Set motors to drive forward with given clamped speeds
void setMotorSpeedsForward(int leftSpeed, int rightSpeed) {
  leftSpeed  = clampPWM(leftSpeed);
  rightSpeed = clampPWM(rightSpeed);

  // Left motor forward
  analogWrite(MOTOR_LEFT_FWD, leftSpeed);
  analogWrite(MOTOR_LEFT_REV, 0);

  // Right motor forward
  analogWrite(MOTOR_RIGHT_FWD, rightSpeed);
  analogWrite(MOTOR_RIGHT_REV, 0);

  // Debug
  Serial.print("[FWD] L_PWM=");
  Serial.print(leftSpeed);
  Serial.print("  R_PWM=");
  Serial.println(rightSpeed);
}

void setup() {
  Serial.begin(9600);

  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_REV, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_REV, OUTPUT);

  // Stop motors initially
  analogWrite(MOTOR_LEFT_FWD, 0);
  analogWrite(MOTOR_LEFT_REV, 0);
  analogWrite(MOTOR_RIGHT_FWD, 0);
  analogWrite(MOTOR_RIGHT_REV, 0);

  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);

  Serial.println("=== Minimal Right-Wall-Hugging PID ===");
  Serial.println("Robot will drive forward indefinitely,");
  Serial.println("adjusting speeds to maintain RIGHT_TARGET cm from right wall.");
  Serial.println("--------------------------------------");
  delay(10);
}

void loop() {
  // Measure right distance
  long distRight = measureDistanceRight();
  if (distRight < 0) distRight = 999;

  // Error
  float error = distRight - RIGHT_TARGET;
  if (flipPidSign) {
    error = -error;
  }

  // Correction
  float correction = Kp_right * error;

  // Raw speeds
  int leftSpeed  = (int)(baseForwardSpeed - correction);
  int rightSpeed = (int)(baseForwardSpeed + correction);

  // --- Debug prints ---
  Serial.println("---- PID Debug ----");
  Serial.print("distRight = ");
  Serial.print(distRight);
  Serial.print(" cm | error = ");
  Serial.print(error);
  Serial.print(" | correction = ");
  Serial.print(correction);
  Serial.print(" | rawL = ");
  Serial.print(leftSpeed);
  Serial.print(" | rawR = ");
  Serial.println(rightSpeed);

  // Update motors
  setMotorSpeedsForward(leftSpeed, rightSpeed);

  // Waiting to loop again
  delay(100);
}