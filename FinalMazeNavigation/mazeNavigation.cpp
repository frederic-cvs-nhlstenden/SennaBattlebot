#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ============================================================================
// CONSTANTS
// ============================================================================

// ----------------- ARDUINO CONSTANTS -----------------
const int MOTOR_LEFT_FWD = 10; // A2: left motor forward
const int MOTOR_LEFT_REV = 11; // A1: left motor reverse
const int MOTOR_RIGHT_FWD = 6; // B1: right motor forward
const int MOTOR_RIGHT_REV = 5; // B2: right motor reverse

const int TRIG_PIN_RIGHT = 4; // right ultrasonic trig
const int ECHO_PIN_RIGHT = 8; // right ultrasonic echo

// Target distance from right wall in cm
const int RIGHT_TARGET = 5;

// Minimum PWM to avoid stalling
const int MIN_PWM = 100;

// Rotation sensor pins
const int LEFT_SENSOR_PIN = 3;  // Left Wheel
const int RIGHT_SENSOR_PIN = 2; // Right Wheel

// Gripper Parameters
const int GRIP_PIN = 7;
const int OPEN_GRIP = 1600;
const int CLOSE_GRIP = 990;

// Maze states
bool sensorsCalibrated = false;
bool secondRobotCleared = false;
bool pickedUpCone = false;
bool turnedLeftAfterPickup = false;
bool droveTwoSecAfterPickup = false;
bool droppedOffCone = false;

// Helper variable to track time for certain steps in Maze sequence
unsigned long stepStartTime = 0;

const int SENSOR_PINS[] = {A0, A1, A2, A3, A4, A5, A6, A7};
int sensorValues[8];
int sensorWeights[] = {-3500, -2500, -1800, -500, 500, 1500, 2500, 3500};
int sensorMin[8];
int sensorMax[8];
int sensorThresholds[8];

// ----- TURN PARAMETERS -----
const float WHEEL_DIAMETER = 6.0; // cm
const float WHEEL_CIRCUMFERENCE = 3.14 * WHEEL_DIAMETER;
const int PULSES_PER_REV = 20;
const float DIST_PER_PULSE = WHEEL_CIRCUMFERENCE / PULSES_PER_REV;
const float TRACK_WIDTH = 9.3; // cm
const int LEFT_TARGET_SPEED = 255;
const int RIGHT_TARGET_SPEED = 240;

const int LEFT_RAMP_SPEEDS[4] = {
    LEFT_TARGET_SPEED / 4,
    LEFT_TARGET_SPEED / 2,
    (LEFT_TARGET_SPEED * 3) / 4,
    LEFT_TARGET_SPEED};

const int RIGHT_RAMP_SPEEDS[4] = {
    RIGHT_TARGET_SPEED / 4,
    RIGHT_TARGET_SPEED / 2,
    (RIGHT_TARGET_SPEED * 3) / 4,
    RIGHT_TARGET_SPEED};

// Global Variables for ISR
volatile unsigned long pulseCountLeft = 0;
volatile unsigned long pulseCountRight = 0;
volatile bool lastStateLeft = LOW;
volatile bool lastStateRight = LOW;
volatile bool motorsActive = false;

// Pins for forward ultrasound
const int TRIG_PIN = 13;
const int ECHO_PIN = 12;
const int OBSTACLE_THRESHOLD = 10; // cm

// Neopixel Setup
#define LED_PIN 9
#define LED_COUNT 4
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Color constants (treated as pseudo-constants)
uint32_t RED = strip.Color(0, 255, 0);
uint32_t GREEN = strip.Color(255, 0, 0);
uint32_t BLUE = strip.Color(0, 0, 255);
uint32_t WHITE = strip.Color(255, 255, 255);
uint32_t OFF = strip.Color(0, 0, 0);

// Adjustable PID parameters
int baseForwardSpeed = 225;
float kpRight = 5.0;
bool flipPidSign = true;

// ============================================================================
// SETUP-CRITICAL METHODS + SETUP
// ============================================================================

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

void setAllNeopixels(uint32_t c)
{
  for (int i = 0; i < LED_COUNT; i++)
  {
    strip.setPixelColor(i, c);
  }
  strip.show();
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

  // NeoPixel init
  strip.begin();
  strip.show();
  strip.setBrightness(50);
  setAllNeopixels(BLUE);

  Serial.println("=== Minimal Right-Wall-Hugging PID ===");
  Serial.println("Robot will drive forward indefinitely,");
  Serial.println("adjusting speeds to maintain RIGHT_TARGET cm from right wall.");
  Serial.println("--------------------------------------");
  delay(10);
}

// ============================================================================
// REMAINING METHODS
// ============================================================================

// Measuring distance on the right ultrasonic sensor
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

// Clamping speeds to mitigate stalling and unexpected behavior
int clampPwm(int val)
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
  leftSpeed = clampPwm(leftSpeed);
  rightSpeed = clampPwm(rightSpeed);

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
  // required to add a fix multiplier, as a result of all the added code introducing unexpected behavior
  float requiredPulses = (arcDistance / DIST_PER_PULSE) * 1.25;

  motorsActive = true;
  const int highTurnSpeed = LEFT_TARGET_SPEED; // initial higher speed
  const int lowTurnSpeed = 150;                // slow near the end
  float speedSwitchThreshold = requiredPulses * 0.6;

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

      setAllNeopixels(RED);

      analogWrite(MOTOR_LEFT_REV, clampPwm(200));
      analogWrite(MOTOR_LEFT_FWD, 0);
      analogWrite(MOTOR_RIGHT_REV, clampPwm(200));
      analogWrite(MOTOR_RIGHT_FWD, 0);
      delay(1000);
      stopMotors();
      delay(1000);
      turnDegrees(90);
      delay(100);

      stopMotors();
      motorsActive = false;
      setAllNeopixels(BLUE);
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
  }
  return;
}

void gripper(int pulse)
{
  static unsigned long timer;
  static int lastPulse;

  if (pulse > 0)
  {
    lastPulse = pulse;
  }

  if (millis() >= timer)
  { // Every 20ms (50Hz signal)
    digitalWrite(GRIP_PIN, HIGH);
    delayMicroseconds(lastPulse); // Keep the gripper holding position
    digitalWrite(GRIP_PIN, LOW);
    timer = millis() + 20;
  }
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

// ============================================================================
// LOOP
// ============================================================================
void loop()
{
  // Wating for the line maze robot to drop the cone in front
  if (!secondRobotCleared)
  {
    long distFront = measureDistance();
    if (distFront > 0 && distFront < 20)
    {
      // Once detected, wait until it leaves
      Serial.println("Second robot in front - waiting...");

      setAllNeopixels(GREEN);

      while (true)
      {
        distFront = measureDistance();
        if (distFront < 0 || distFront >= 15)
          break;
        delay(50);
      }
      //
      delay(5000);
      secondRobotCleared = true;
      setAllNeopixels(BLUE);
    }
    else
    {
      // If not deteced, keep motors off
      stopMotors();
    }
    return;
  }

  // Calibrating on the way to pick up the cone
  // Ignoring the gripper close signal for 1s
  static bool calibrating = true; // Maze state boolean to ensure we only calibrate once
  static unsigned long calibrateStartTime = 0;

  if (calibrating)
  {
    // In our first pass of the calibration block, we set the calibrationStartTime
    if (calibrateStartTime == 0)
    {
      calibrateStartTime = millis();
      Serial.println("Starting 1s on-the-fly calibration, ignoring black lines...");
      setAllNeopixels(WHITE);
    }

    // Continuously update min & max for each sensor
    for (int i = 0; i < 8; i++)
    {
      int val = analogRead(SENSOR_PINS[i]);
      if (val < sensorMin[i])
        sensorMin[i] = val;
      if (val > sensorMax[i])
        sensorMax[i] = val;
    }

    setMotorSpeedsForward(180, 180);

    // Check if 1 second has elapsed
    if (millis() - calibrateStartTime >= 1000UL)
    {
      // finalize thresholds
      for (int i = 0; i < 8; i++)
      {
        sensorThresholds[i] = (sensorMin[i] + sensorMax[i]) / 2;
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(": min=");
        Serial.print(sensorMin[i]);
        Serial.print(", max=");
        Serial.print(sensorMax[i]);
        Serial.print(", threshold=");
        Serial.println(sensorThresholds[i]);
      }

      sensorsCalibrated = true;
      calibrating = false;
      stopMotors();
      Serial.println("Done with calibration -> now normal pickup logic...");
      setAllNeopixels(BLUE);
    }
    return;
  }

  // If we haven't picked up the cone yet ->
  // drive until black surface -> stop & pickup -> turn -> set stepStartTime -> done
  if (!pickedUpCone)
  {
    // If we are not on black surface -> drive forward, else pick up
    if (!isAllBlack())
    {
      setMotorSpeedsForward(180, 180);
      return;
    }
    else
    {
      stopMotors();

      unsigned long closeStart = millis();
      while (millis() - closeStart < 1000)
      {
        // Repeating gripper signal for 1s to ensure it closes
        gripper(CLOSE_GRIP);
      }

      pickedUpCone = true;
      turnDegrees(90);
      stepStartTime = millis();
      return;
    }
  }

  // If we have picked up the cone but haven't driven 2s yet, do so
  if (pickedUpCone && !droveTwoSecAfterPickup)
  {
    unsigned long elapsed = millis() - stepStartTime;
    if (elapsed < 2000)
    {
      setMotorSpeedsForward(180, 180);
      return;
    }
    else
    {
      stopMotors();
      droveTwoSecAfterPickup = true;
      Serial.println("Done with 2s drive after pickup -> now do PID maze...");
      // fall through to normal maze logic
    }
  }

  // Check if we see black surface again -> drop cone
  if (pickedUpCone && !droppedOffCone && isAllBlack())
  {
    Serial.println("Black again => dropping cone => done.");
    stopMotors();
    for (int i = 0; i < 5; i++)
    {
      gripper(OPEN_GRIP);
    }
    droppedOffCone = true;

    setAllNeopixels(GREEN);
  }

  // Normal Maze/PID Logic
  checkLeft();

  long distRightVal = measureDistanceRight();
  if (distRightVal < 0)
    distRightVal = 999;

  float error = distRightVal - RIGHT_TARGET;
  if (flipPidSign)
    error = -error;
  float correction = kpRight * error;

  int leftSpeed = (int)(baseForwardSpeed - correction);
  int rightSpeed = (int)(baseForwardSpeed + correction);

  Serial.println("---- PID Debug ----");
  Serial.print("distRight = ");
  Serial.print(distRightVal);
  Serial.print(" cm | error = ");
  Serial.print(error);
  Serial.print(" | correction = ");
  Serial.print(correction);
  Serial.print(" | rawL = ");
  Serial.print(leftSpeed);
  Serial.print(" | rawR = ");
  Serial.println(rightSpeed);

  setMotorSpeedsForward(leftSpeed, rightSpeed);
  delay(100);
}