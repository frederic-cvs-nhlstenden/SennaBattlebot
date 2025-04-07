// Motor and Sensor Pin Definitions
const int MOTOR_A_1 = 11; // Left motor reverse
const int MOTOR_A_2 = 10; // Left motor forward
const int MOTOR_B_1 = 6;  // Right motor forward
const int MOTOR_B_2 = 5;  // Right motor reverse

// Rotation sensor pins
const int LEFT_SENSOR_PIN = 3;  // Left Wheel
const int RIGHT_SENSOR_PIN = 2; // Right Wheel

// Wheel and Rotation Sensor Parameters
const float WHEEL_DIAMETER = 6.0; // cm
const float WHEEL_CIRCUMFERENCE = 3.14 * WHEEL_DIAMETER;
const int PULSES_PER_REV = 20;
const float DIST_PER_PULSE = WHEEL_CIRCUMFERENCE / PULSES_PER_REV;

// Geometry for calculating turns
const float TRACK_WIDTH = 9.3; // cm, distance between wheel centers

// Motor Speed Settings
// Right Motor is stronger, so lower target speed
const int LEFT_TARGET_SPEED = 255;
const int RIGHT_TARGET_SPEED = 240;

// 25% ramp steps (4 steps: 25%, 50%, 75%, 100%) for each motor
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

// Pins for the HC-SR04
const int TRIG_PIN = 13;
const int ECHO_PIN = 12;
const int OBSTACLE_THRESHOLD = 15; // cm

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

// Setup
void setup()
{
    // Motor pins
    pinMode(MOTOR_A_1, OUTPUT);
    pinMode(MOTOR_A_2, OUTPUT);
    pinMode(MOTOR_B_1, OUTPUT);
    pinMode(MOTOR_B_2, OUTPUT);

    // Ensure motors off to start
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);

    // Wheel rotation sensors
    pinMode(LEFT_SENSOR_PIN, INPUT_PULLUP);
    pinMode(RIGHT_SENSOR_PIN, INPUT_PULLUP);

    // Initialize last rotation sensor states to the current reads
    lastStateLeft = digitalRead(LEFT_SENSOR_PIN);
    lastStateRight = digitalRead(RIGHT_SENSOR_PIN);

    attachInterrupt(digitalPinToInterrupt(LEFT_SENSOR_PIN), pulseLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_SENSOR_PIN), pulseRight, CHANGE);

    // Ultrasonic sensor
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    Serial.begin(9600);
}

// Stop all motors method
void stopMotors()
{
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
    motorsActive = false;
}

// Distance Method: calculate the distance traveled (in cm)
float getDistanceTraveled()
{
    noInterrupts();
    unsigned long leftCount = pulseCountLeft;
    unsigned long rightCount = pulseCountRight;
    interrupts();

    unsigned long avgPulse = (leftCount + rightCount) / 2;
    return avgPulse * DIST_PER_PULSE;
}

// Measure distance in front of bot with HC-SR04 (Ultrasound Sensor)
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

// Drive forward method (Takes desired distance and uses motor ramp up to mitigate slip)
void driveDistanceForward(float targetDistance)
{
    // Reset pulses AND last states
    noInterrupts();
    pulseCountLeft = 0;
    pulseCountRight = 0;
    lastStateLeft = digitalRead(LEFT_SENSOR_PIN);
    lastStateRight = digitalRead(RIGHT_SENSOR_PIN);
    interrupts();

    motorsActive = true;

    // Ramp up
    for (int i = 0; i < 4; i++)
    {
        analogWrite(MOTOR_A_2, leftRampSpeeds[i]); // left forward
        analogWrite(MOTOR_A_1, 0);
        analogWrite(MOTOR_B_1, rightRampSpeeds[i]); // right forward
        analogWrite(MOTOR_B_2, 0);

        delay(RAMP_DELAY);
    }

    // Move until target or obstacle
    while (getDistanceTraveled() < targetDistance)
    {
        long d = measureDistance();
        if (d > 0 && d < OBSTACLE_THRESHOLD)
        {
            stopMotors();
            Serial.println("Obstacle detected. Stopping forward motion.");
            break;
        }
        delay(50);
    }

    stopMotors();
}

// Drive backward method (Takes desired distance and uses motor ramp up to mitigate slip)
void driveDistanceBackward(float targetDistance)
{
    // Reset pulses AND last states
    noInterrupts();
    pulseCountLeft = 0;
    pulseCountRight = 0;
    lastStateLeft = digitalRead(LEFT_SENSOR_PIN);
    lastStateRight = digitalRead(RIGHT_SENSOR_PIN);
    interrupts();

    motorsActive = true;

    // Ramp up in reverse
    for (int i = 0; i < 4; i++)
    {
        analogWrite(MOTOR_A_1, leftRampSpeeds[i]); // left reverse
        analogWrite(MOTOR_A_2, 0);
        analogWrite(MOTOR_B_2, rightRampSpeeds[i]); // right reverse
        analogWrite(MOTOR_B_1, 0);

        delay(RAMP_DELAY);
    }

    while (getDistanceTraveled() < targetDistance)
    {
        delay(50);
    }

    stopMotors();
}

// Method to rotate in place (by degrees, positive = left turn, negative = right turn)
void turnDegrees(int degrees)
{
    // Resetting pulses and last states from ISR
    noInterrupts();
    pulseCountLeft = 0;
    pulseCountRight = 0;
    lastStateLeft = digitalRead(LEFT_SENSOR_PIN);
    lastStateRight = digitalRead(RIGHT_SENSOR_PIN);
    interrupts();

    // Arc distance calculation and required pulses accordingly
    float arcDistance = (3.14 * TRACK_WIDTH * abs(degrees)) / 360.0;
    float requiredPulses = arcDistance / DIST_PER_PULSE;

    motorsActive = true;

    // highTurnSpeed is used for the initial phase and lowTurnSpeed for the end phase.
    const int highTurnSpeed = LEFT_TARGET_SPEED; // full power initially (using same speed for both)
    const int lowTurnSpeed = 150;                // reduced speed to minimize slip at the end of turn

    // Threshold for switching to low speed to mitigate slip at the end of the turn (avoids overrotation)
    float speedSwitchThreshold = requiredPulses * 0.8;

    if (degrees > 0)
    {
        // Turn left: left wheel reverse, right wheel forward
        analogWrite(MOTOR_A_1, highTurnSpeed);
        analogWrite(MOTOR_A_2, 0);
        analogWrite(MOTOR_B_1, highTurnSpeed);
        analogWrite(MOTOR_B_2, 0);
    }
    else
    {
        // Turn right: left wheel forward, right wheel reverse
        analogWrite(MOTOR_A_2, highTurnSpeed);
        analogWrite(MOTOR_A_1, 0);
        analogWrite(MOTOR_B_2, highTurnSpeed);
        analogWrite(MOTOR_B_1, 0);
    }

    // Continue turning while pulse counts
    while (((pulseCountLeft + pulseCountRight) / 2.0) < requiredPulses)
    {
        float avgPulses = (pulseCountLeft + pulseCountRight) / 2.0;
        // When threshold reached, reduce to lower speed (to slow down and avoid slipping at end)
        if (avgPulses >= speedSwitchThreshold)
        {
            if (degrees > 0)
            {
                analogWrite(MOTOR_A_1, lowTurnSpeed);
                analogWrite(MOTOR_B_1, lowTurnSpeed);
            }
            else
            {
                analogWrite(MOTOR_A_2, lowTurnSpeed);
                analogWrite(MOTOR_B_2, lowTurnSpeed);
            }
        }
        delay(10);
    }

    stopMotors();

    // Short braking pulse to counteract intertia
    if (degrees > 0)
    {
        analogWrite(MOTOR_A_2, 100); // left forward
        analogWrite(MOTOR_A_1, 0);
        analogWrite(MOTOR_B_2, 100); // right reverse
        analogWrite(MOTOR_B_1, 0);
    }
    else
    {
        analogWrite(MOTOR_A_1, 100); // left reverse
        analogWrite(MOTOR_A_2, 0);
        analogWrite(MOTOR_B_1, 100); // right forward
        analogWrite(MOTOR_B_2, 0);
    }
    delay(50);
    stopMotors();
}

// Simple obstacle detour
void avoidObstacle()
{
    stopMotors();
    delay(1000);
    turnDegrees(90); // turn left
    delay(1000);
    driveDistanceForward(30);
    delay(1000);
    turnDegrees(-90); // turn right
    delay(1000);
    driveDistanceForward(30);
    delay(1000);
    turnDegrees(-90); // turn right
    delay(1000);
    driveDistanceForward(30);
    delay(1000);
    turnDegrees(90); // turn left
    delay(1000);
}

// ---------------------------
// Main Loop
// ---------------------------
void loop()
{
    driveDistanceForward(50);

    if (measureDistance() > 0 && measureDistance() < OBSTACLE_THRESHOLD)
    {
        Serial.println("Obstacle detected. Initiating avoidance maneuver.");
        avoidObstacle();
    }
}