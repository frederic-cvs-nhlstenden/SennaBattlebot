// Motor and Sensor Pin Definitions
const int MOTOR_A_1 = 11; // Left motor reverse
const int MOTOR_A_2 = 10; // Left motor forward
const int MOTOR_B_1 = 6;  // Right motor forward
const int MOTOR_B_2 = 5;  // Right motor reverse

// Rotation Sensor pins
// Note: Right sensor on pin 3, Left sensor on pin 2.
const int ROT_SENSOR_R1 = 3; // Right wheel sensor
const int ROT_SENSOR_R2 = 2; // Left wheel sensor

// Light pins
const int GREEN_LED = 4;
const int RED_LED = 9;
// const int NEO_LED = 1;
const int NUM_PIXELS = 4; // number of pixels on the PCB
// Adafruit_NeoPixel strip(NUM_PIXELS, NEO_LED, NEO_GRB + NEO_KHZ800);

// Wheel and Encoder Parameters
const float WHEEL_DIAMETER = 6.0;
const float WHEEL_CIRCUMFERENCE = 3.14 * WHEEL_DIAMETER;
const int PULSES_PER_REV = 20;
const float DIST_PER_PULSE = WHEEL_CIRCUMFERENCE / PULSES_PER_REV;

// Robot Geometry
const float TRACK_WIDTH = 10.0;

// Motor Speed Settings
// Left motor is stronger: left motor target is 255, right motor target is 245.
const int LEFT_TARGET_SPEED = 255;
const int RIGHT_TARGET_SPEED = 245;

// 25% ramp steps (4 steps: 25%, 50%, 75%, 100%) for each motor
const int leftRampSpeeds[4] = {LEFT_TARGET_SPEED / 4, LEFT_TARGET_SPEED / 2, (LEFT_TARGET_SPEED * 3) / 4, LEFT_TARGET_SPEED};
const int rightRampSpeeds[4] = {RIGHT_TARGET_SPEED / 4, RIGHT_TARGET_SPEED / 2, (RIGHT_TARGET_SPEED * 3) / 4, RIGHT_TARGET_SPEED};

const unsigned long rampDelay = 250;

// Global Variables for ISR
volatile unsigned long pulseCountR1 = 0;
volatile unsigned long pulseCountR2 = 0;
volatile unsigned long lastPulseTimeR1 = 0;
volatile unsigned long lastPulseTimeR2 = 0;
volatile bool lastStateR1 = LOW;
volatile bool lastStateR2 = LOW;

volatile bool motorsActive = false;
volatile bool testActive = true;

// Pins for the HC-SR04 (Ultrasonic Distance Sensor)
const int TRIG_PIN = 13;
const int ECHO_PIN = 12;

// Gripper pins
const int GRIP_PIN = 7;
const int OPEN_GRIP = 1600; // 1600
const int CLOSE_GRIP = 990; // 990

// Simple threshold for obstacle
const int OBSTACLE_THRESHOLD = 10;

// Interrupt Service Routines
// Count only on state changes with a debounce interval
void pulseR1()
{
  if (!motorsActive)
    return;
  bool currentState = digitalRead(ROT_SENSOR_R1);
  unsigned long now = millis();
  if (currentState != lastStateR1 && (now - lastPulseTimeR1 > 10))
  {
    pulseCountR1++;
    lastStateR1 = currentState;
    lastPulseTimeR1 = now;
  }
}

void pulseR2()
{
  if (!motorsActive)
    return;
  bool currentState = digitalRead(ROT_SENSOR_R2);
  unsigned long now = millis();
  if (currentState != lastStateR2 && (now - lastPulseTimeR2 > 10))
  {
    pulseCountR2++;
    lastStateR2 = currentState;
    lastPulseTimeR2 = now;
  }
}

void setup()
{
  // Motor pins setup
  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_1, 0);
  analogWrite(MOTOR_B_2, 0);

  // Rotational sensors pins setup (with pull-ups)
  pinMode(ROT_SENSOR_R1, INPUT_PULLUP);
  pinMode(ROT_SENSOR_R2, INPUT_PULLUP);
  lastStateR1 = digitalRead(ROT_SENSOR_R1);
  lastStateR2 = digitalRead(ROT_SENSOR_R2);

  // LED pins setup
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, HIGH);

  // attach interrupts
  attachInterrupt(digitalPinToInterrupt(ROT_SENSOR_R1), pulseR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROT_SENSOR_R2), pulseR2, CHANGE);

  // Grip pin setup
  pinMode(GRIP_PIN, OUTPUT);
  digitalWrite(GRIP_PIN, LOW);

  Serial.begin(9600);
}

void loop()
{
  static int phase = 0; // keeps track of which "phase" we're in out of the 4 phases

  for (int i = 0; i < 50; i++)
  {
    gripper(0);
    delay(20);
  }

  switch (phase)
  {
  case 0: // 1st phase open and close grippers with 1 ssecond intervals
    Serial.println("Opening gripper");
    for (int i = 0; i < 50; i++)
    {
      gripper(OPEN_GRIP);
      delay(20);
    }

    Serial.println("Closing gripper");
    for (int i = 0; i < 50; i++)
    {
      gripper(CLOSE_GRIP);
      delay(20);
    }

    Serial.println("Opening gripper");
    for (int i = 0; i < 50; i++)
    {
      gripper(OPEN_GRIP);
      delay(20);
    }

    delay(1000);
    phase++;
    break;

  case 1: // 2nd phase move forward and grab the cone
    Serial.println("Moving forward");
    driveDistanceForward(25);
    delay(500);

    Serial.println("Closing gripper");
    for (int i = 0; i < 50; i++)
    {
      gripper(CLOSE_GRIP);
      delay(20);
    }
    delay(2000);

    phase++;
    break;

  case 2: // 3rd phase move forward again and release the cone
    Serial.println("Moving forward");
    driveDistanceForward(25);
    delay(1000);

    Serial.println("Opening gripper");
    for (int i = 0; i < 50; i++)
    {
      gripper(OPEN_GRIP);
      delay(20);
    }
    delay(2000);

    phase++;
    break;

  case 3: // 4th phase move backwards and close gripper
    Serial.println("Moving backwards");
    driveDistanceBackward(25);
    delay(500);

    Serial.println("Closing gripper");
    for (int i = 0; i < 50; i++)
    {
      gripper(CLOSE_GRIP);
      delay(20);
    }
    delay(2000);

    Serial.println("Task completed");
    phase++;
    break;

  default:
    for (int i = 0; i < 50; i++)
    {
      gripper(0);
      delay(20);
    }
    break;
  }
}

float getDistanceTraveled()
{
  noInterrupts();
  unsigned long countR1 = pulseCountR1;
  unsigned long countR2 = pulseCountR2;
  interrupts();
  unsigned long avgPulse = (countR1 + countR2) / 2;
  return avgPulse * DIST_PER_PULSE;
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

// Measure distance with HC-SR04
long measureDistance()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

// movement functions

// Drive forward a specified distance (in cm) using a motor speed ramp up
void driveDistanceForward(float targetDistance)
{
  noInterrupts();
  pulseCountR1 = 0;
  pulseCountR2 = 0;
  interrupts();

  motorsActive = true;
  digitalWrite(GREEN_LED, LOW);
  // Ramp up in 4 steps for moving forward
  for (int i = 0; i < 4; i++)
  {
    analogWrite(MOTOR_A_2, leftRampSpeeds[i]); // Left motor forward.
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_B_1, rightRampSpeeds[i]); // Right motor forward.
    analogWrite(MOTOR_B_2, 0);
    delay(rampDelay);
  }

  while (getDistanceTraveled() < targetDistance)
  {
    Serial.print("Forward Distance: ");
    Serial.println(getDistanceTraveled());

    gripper(0);
    delay(20);
  }

  stopMotors();
  Serial.print("Final Forward Distance: ");
  Serial.println(getDistanceTraveled());
}

void driveDistanceBackward(float targetDistance)
{
  // Reset encoder counts
  noInterrupts();
  pulseCountR1 = 0;
  pulseCountR2 = 0;
  interrupts();

  motorsActive = true;
  digitalWrite(RED_LED, LOW);

  // Ramp up in 4 steps to move backwards
  for (int i = 0; i < 4; i++)
  {
    analogWrite(MOTOR_A_1, leftRampSpeeds[i]); // left motor backwards
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, rightRampSpeeds[i]); // right motor backwards
    delay(rampDelay);
  }

  while (getDistanceTraveled() < targetDistance)
  {
    Serial.print("Reverse distance: ");
    Serial.println(getDistanceTraveled());

    gripper(0);
    delay(20);
  }

  stopMotors();
  Serial.print("Final Reverse Distance: ");
  Serial.println(getDistanceTraveled());
}

void stopMotors()
{
  analogWrite(MOTOR_A_1, 0);
  analogWrite(MOTOR_A_2, 0);
  analogWrite(MOTOR_B_1, 0);
  analogWrite(MOTOR_B_2, 0);
  motorsActive = false;

  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, HIGH);
}