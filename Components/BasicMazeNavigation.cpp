#include <Arduino.h>

// ----------------- ARDUINO CONSTANTS -----------------
const int MOTOR_LEFT_FWD = 10; // A2: left motor forward
const int MOTOR_LEFT_REV = 11; // A1: left motor reverse
const int MOTOR_RIGHT_FWD = 6; // B1: right motor forward
const int MOTOR_RIGHT_REV = 5; // B2: right motor reverse

const int TRIG_PIN_RIGHT = 4; // right ultrasonic trig
const int ECHO_PIN_RIGHT = 8; // right ultrasonic echo

// Target distance from right wall in cm
const int RIGHT_TARGET = 5;

// Base forward speed (0-255)
int baseForwardSpeed = 225;

// Proportional gain for the right-wall PID
float Kp_right = 5.0;

// Minimum PWM to avoid stalling
const int MIN_PWM = 100;

// Debug function to test logic between forward and backward mode
// setting this to 'true' means: error => -(distRight - RIGHT_TARGET)
bool flipPidSign = true;

// Rotation sensor pins
const int LEFT_SENSOR_PIN = 3;  // Left Wheel
const int RIGHT_SENSOR_PIN = 2; // Right Wheel

// Gripper Parameters
const int GRIP_PIN = 7;
const int OPEN_GRIP = 1600; // 1600
const int CLOSE_GRIP = 990; // 990
bool pickedUpCone = false;
bool droppedOffCone = false;

const int SENSOR_PINS[] = {A0, A1, A2, A3, A4, A5, A6, A7};
int SENSOR_VALUES[8];
int SENSOR_WEIGHTS[] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

int sensorMin[8];
int sensorMax[8];
int sensorThresholds[8];

bool LINE_SENSOR_IS_CALIBRATED = false;

// ----- TURN PARAMETERS -----
// Wheel and Rotation Sensor Parameters
const float WHEEL_DIAMETER = 6.0; // cm
const float WHEEL_CIRCUMFERENCE = 3.14 * WHEEL_DIAMETER;
const int PULSES_PER_REV = 20;
const float DIST_PER_PULSE = WHEEL_CIRCUMFERENCE / PULSES_PER_REV;
const float TRACK_WIDTH = 9.3; // cm
const int LEFT_TARGET_SPEED = 255;
const int RIGHT_TARGET_SPEED = 240;

// Motor ramp up speeds for L&R
const int leftRampSpeeds[4] = {
    LEFT_TARGET_SPEED / 4,
    LEFT_TARGET_SPEED / 2,
    (LEFT_TARGET_SPEED * 3) / 4,
    LEFT_TARGET_SPEED};

const int rightRampSpeeds[4] = {
    RIGHT_TARGET_SPEED / 4,
    RIGHT_TARGET_SPEED / 2,
    (RIGHT_TARGET_SPEED * 3) / 4,
    RIGHT_TARGET_SPEED};

const unsigned long RAMP_DELAY = 250;

// Global Variables for ISR
volatile unsigned long pulseCountLeft = 0;
volatile unsigned long pulseCountRight = 0;
volatile bool lastStateLeft = LOW;
volatile bool lastStateRight = LOW;

volatile bool motorsActive = false;
volatile bool testActive = true;

// Pins for forward ultrasound
const int TRIG_PIN = 13;
const int ECHO_PIN = 12;
const int OBSTACLE_THRESHOLD = 10; // cm

// ISRs
void pulseLeft()
{
  if (!motorsActive)
    return;

  bool currentState = digitalRead(LEFT_SENSOR_PIN);
  if (currentState != lastStateLeft)
  {
    pulseCountLeft++;
    lastStateLeft = currentState;
  }
}

void pulseRight()
{
  if (!motorsActive)
    return;

  bool currentState = digitalRead(RIGHT_SENSOR_PIN);
  if (currentState != lastStateRight)
  {
    pulseCountRight++;
    lastStateRight = currentState;
  }
}

// ---------------------------------------------------

// Measuring distance via ultrasonic
long measureDistanceRight()
{
  digitalWrite(TRIG_PIN_RIGHT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_RIGHT, LOW);

  long duration = pulseIn(ECHO_PIN_RIGHT, HIGH, 30000); //
  if (duration == 0)
  {
    // No echo => out of range
    return -1;
  }
  // Convert microseconds to cm
  return (long)(duration * 0.034 / 2.0);
}

// Clamping speeds so we don't stall and we don't get any unexpected behavior
int clampPWM(int val)
{
  if (val < 0)
    val = 0;
  if (val > 255)
    val = 255;
  if (val > 0 && val < MIN_PWM)
  {
    val = MIN_PWM;
  }
  return val;
}

// Set motors to drive forward with given clamped speeds
void setMotorSpeedsForward(int leftSpeed, int rightSpeed)
{
  leftSpeed = clampPWM(leftSpeed);
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

void setup()
{
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
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(GRIP_PIN, OUTPUT);
  digitalWrite(GRIP_PIN, LOW);

  // Wheel rotation sensors
  pinMode(LEFT_SENSOR_PIN, INPUT_PULLUP);
  pinMode(RIGHT_SENSOR_PIN, INPUT_PULLUP);

  // Initialize last rotation sensor states to the current reads
  lastStateLeft = digitalRead(LEFT_SENSOR_PIN);
  lastStateRight = digitalRead(RIGHT_SENSOR_PIN);

  attachInterrupt(digitalPinToInterrupt(LEFT_SENSOR_PIN), pulseLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SENSOR_PIN), pulseRight, CHANGE);

  Serial.println("=== Minimal Right-Wall-Hugging PID ===");
  Serial.println("Robot will drive forward indefinitely,");
  Serial.println("adjusting speeds to maintain RIGHT_TARGET cm from right wall.");
  Serial.println("--------------------------------------");
  delay(10);
}

void stopMotors()
{
  analogWrite(MOTOR_LEFT_REV, 0);
  analogWrite(MOTOR_LEFT_FWD, 0);
  analogWrite(MOTOR_RIGHT_FWD, 0);
  analogWrite(MOTOR_RIGHT_REV, 0);
  motorsActive = false;
}

// Calculating distance travelled on the basis of the rotary encoders
float getDistanceTraveled()
{
  noInterrupts();
  unsigned long leftCount = pulseCountLeft;
  unsigned long rightCount = pulseCountRight;
  interrupts();

  unsigned long avgPulse = (leftCount + rightCount) / 2;
  return avgPulse * DIST_PER_PULSE;
}

// Measuring distance with front mounted ultrasound
long measureDistance()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2; // cm
  return distance;
}

void turnDegrees(int degrees)
{
  // Reseting encoder and pulse states to mitigate any anomalous readings
  noInterrupts();
  pulseCountLeft = 0;
  pulseCountRight = 0;
  lastStateLeft = digitalRead(LEFT_SENSOR_PIN);
  lastStateRight = digitalRead(RIGHT_SENSOR_PIN);
  interrupts();

  // Calculating the distance and respective pulses required
  float arcDistance = (3.14 * TRACK_WIDTH * abs(degrees)) / 360.0;
  float requiredPulses = arcDistance / DIST_PER_PULSE;

  motorsActive = true;
  const int highTurnSpeed = LEFT_TARGET_SPEED; // initial higher speed
  const int lowTurnSpeed = 150;                // slow near the end
  float speedSwitchThreshold = requiredPulses * 0.8;

  // Setting motors up depending on turn direction
  if (degrees > 0)
  {
    // Turn left
    analogWrite(MOTOR_LEFT_REV, highTurnSpeed);
    analogWrite(MOTOR_LEFT_FWD, 0);
    analogWrite(MOTOR_RIGHT_FWD, highTurnSpeed);
    analogWrite(MOTOR_RIGHT_REV, 0);
  }
  else
  {
    // Turn right
    analogWrite(MOTOR_LEFT_FWD, highTurnSpeed);
    analogWrite(MOTOR_LEFT_REV, 0);
    analogWrite(MOTOR_RIGHT_REV, highTurnSpeed);
    analogWrite(MOTOR_RIGHT_FWD, 0);
  }

  // Time limit check; if turn not complete in 3 seconds, escape
  unsigned long startTime = millis();
  while (((pulseCountLeft + pulseCountRight) / 2.0) < requiredPulses)
  {
    // Esacpe squence after 3 seconds
    if (millis() - startTime >= 3000UL)
    {
      Serial.println("Turn timed out (3s). Reversing to free ourselves.");
      stopMotors();

      analogWrite(MOTOR_LEFT_REV, clampPWM(200));
      analogWrite(MOTOR_LEFT_FWD, 0);
      analogWrite(MOTOR_RIGHT_REV, clampPWM(200));
      analogWrite(MOTOR_RIGHT_FWD, 0);
      delay(1000);
      stopMotors();
      delay(1000);
      turnDegrees(90);
      delay(100);

      stopMotors();
      motorsActive = false;
      return; // Returning early to go back to main loop
    }

    // Switching to lower speed near the end to avoid overshoot
    float avgPulses = (pulseCountLeft + pulseCountRight) / 2.0;
    if (avgPulses >= speedSwitchThreshold)
    {
      if (degrees > 0)
      {
        analogWrite(MOTOR_LEFT_REV, lowTurnSpeed);
        analogWrite(MOTOR_RIGHT_FWD, lowTurnSpeed);
      }
      else
      {
        analogWrite(MOTOR_LEFT_FWD, lowTurnSpeed);
        analogWrite(MOTOR_RIGHT_REV, lowTurnSpeed);
      }
    }

    delay(10);
  }

  // If the turn is finished on time, stop and do a quick braking pulse to counteract inertia
  stopMotors();
  if (degrees > 0)
  {
    analogWrite(MOTOR_LEFT_FWD, 100);  // left forward
    analogWrite(MOTOR_RIGHT_REV, 100); // right reverse
  }
  else
  {
    analogWrite(MOTOR_LEFT_REV, 100);  // left reverse
    analogWrite(MOTOR_RIGHT_FWD, 100); // right forward
  }
  delay(50);

  stopMotors();
  motorsActive = false;
}

void checkLeft()
{
  if (measureDistance() < OBSTACLE_THRESHOLD)
  {
    stopMotors();
    delay(1000);
    turnDegrees(90);
    delay(1000);
  }
  return;
}

void gripper(int pulse)
{
  static unsigned long timer;
  static int lastPulse;

  if (pulse > 0)
  {
    lastPulse = pulse; // Store the last requested pulse width
  }

  if (millis() >= timer)
  { // Every 20ms (50Hz signal)
    digitalWrite(GRIP_PIN, HIGH);
    delayMicroseconds(lastPulse); // Keep the gripper holding position
    digitalWrite(GRIP_PIN, LOW);
    timer = millis() + 20;
  }
}

void calibrateSensors()
{
  // Initializing each sensor min and max to their respective values
  for (int i = 0; i < 8; i++)
  {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  // Speed of motors for calibration
  const int CALIB_SPEED = 255;

  motorsActive = true; // Letting ISRs count pulses

  // Drive Forward for 1 second
  analogWrite(MOTOR_LEFT_FWD, CALIB_SPEED);
  analogWrite(MOTOR_LEFT_REV, 0);
  analogWrite(MOTOR_RIGHT_FWD, CALIB_SPEED);
  analogWrite(MOTOR_RIGHT_REV, 0);

  unsigned long startTime = millis();
  while (millis() - startTime < 1000UL)
  {
    // Continuously updating sensorMin / sensorMax
    for (int i = 0; i < 8; i++)
    {
      int sensorValue = analogRead(SENSOR_PINS[i]);
      if (sensorValue < sensorMin[i])
        sensorMin[i] = sensorValue;
      if (sensorValue > sensorMax[i])
        sensorMax[i] = sensorValue;
    }
    delay(5); // Small delay to reduce noise
  }

  // Driving backwards to original position
  analogWrite(MOTOR_LEFT_FWD, 0);
  analogWrite(MOTOR_LEFT_REV, CALIB_SPEED);
  analogWrite(MOTOR_RIGHT_FWD, 0);
  analogWrite(MOTOR_RIGHT_REV, CALIB_SPEED);

  startTime = millis();
  while (millis() - startTime < 1000UL)
  {
    for (int i = 0; i < 8; i++)
    {
      int sensorValue = analogRead(SENSOR_PINS[i]);
      if (sensorValue < sensorMin[i])
        sensorMin[i] = sensorValue;
      if (sensorValue > sensorMax[i])
        sensorMax[i] = sensorValue;
    }
    delay(5);
  }

  stopMotors();
  motorsActive = false;

  // Thresholds computed as aveg of min and max for each sensor
  for (int i = 0; i < 8; i++)
  {
    sensorThresholds[i] = (sensorMin[i] + sensorMax[i]) / 2;
  }

  for (int i = 0; i < 8; i++)
  {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": min=");
    Serial.print(sensorMin[i]);
    Serial.print(", max=");
    Serial.print(sensorMax[i]);
    Serial.print(", threshold=");
    Serial.println(sensorThresholds[i]);
  }

  LINE_SENSOR_IS_CALIBRATED = true;
}

bool isAllBlack()
{
  // Read each sensor and compare to its threshold
  for (int i = 0; i < 8; i++)
  {
    int reading = analogRead(SENSOR_PINS[i]);
    // If the square underneath the sensor is black, the readings exceed the thresholds
    // otherwise, the readings stay below, and so we return false
    if (reading <= sensorThresholds[i])
    {
      return false;
    }
  }

  return true;
}

void loop()
{

  /* if (!LINE_SENSOR_IS_CALIBRATED)
  {
    calibrateSensors();
  }

  Serial.print("Sensor readings: ");
  for (int i = 0; i < 8; i++)
  {
    int reading = analogRead(SENSOR_PINS[i]);
    Serial.print(reading);
    Serial.print("/");
    Serial.print(sensorThresholds[i]);
    Serial.print("  ");
  }
  Serial.println();

  if (isAllBlack() && !pickedUpCone)
  {
    for (int i = 0; i < 5; i++)
    {
      gripper(CLOSE_GRIP);
    }
    pickedUpCone = true;
    turnDegrees(90);
  }
  if (isAllBlack() && pickedUpCone && !droppedOffCone)
  {
    for (int i = 0; i < 5; i++)
    {
      gripper(OPEN_GRIP);
    }
    droppedOffCone = true;
    stopMotors();
  } */
  checkLeft();
  // Measure right distance
  long distRight = measureDistanceRight();
  if (distRight < 0)
    distRight = 999;

  // Error
  float error = distRight - RIGHT_TARGET;
  if (flipPidSign)
  {
    error = -error;
  }

  // Correction
  float correction = Kp_right * error;

  // Raw speeds
  int leftSpeed = (int)(baseForwardSpeed - correction);
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