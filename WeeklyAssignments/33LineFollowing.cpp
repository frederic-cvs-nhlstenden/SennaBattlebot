const int MOTOR_A_1 = 11;  // Left motor reverse
const int MOTOR_A_2 = 10;  // Left motor forward
const int MOTOR_B_1 = 6;   // Right motor forward
const int MOTOR_B_2 = 5;   // Right motor reverse

// Line sensor array
const int SENSOR_PINS[] = {A0, A1, A2, A3, A4, A5, A6, A7};
int SENSOR_VALUES[8];
int SENSOR_WEIGHTS[] =  { -3500, -2500, -1500, -500, 500, 1500, 2500, 3500 };

// Calibration arrays for each sensor
int sensorMin[8];
int sensorMax[8];
int sensorThresholds[8];

float _Kp = 0.12;   // Proportional gain
float _Ki = 0.001;  // Integral gain
float _Kd = 0.04;   // Derivative gain

int _previousError = 0;
int _P = 0;
int _integral = 0; 
int _baseSpeed = 175;

void setup() {
    Serial.begin(9600);

    pinMode(MOTOR_A_1, OUTPUT); 
    pinMode(MOTOR_A_2, OUTPUT); 
    pinMode(MOTOR_B_1, OUTPUT); 
    pinMode(MOTOR_B_2, OUTPUT);

    for (int i = 0; i < 8; i++) {  
        pinMode(SENSOR_PINS[i], INPUT);
    }

    calibrateSensors();
}   

void loop() {
    int position = readSensors();
    int error = position;
    PIDcontrol(error);
}

void calibrateSensors() {
    // Min and max values for each sensor
    for (int i = 0; i < 8; i++) {
        sensorMin[i] = 1023; 
        sensorMax[i] = 0;
    }

    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        for (int i = 0; i < 8; i++) {
            int sensorValue = analogRead(SENSOR_PINS[i]);
            if (sensorValue < sensorMin[i]) {
                sensorMin[i] = sensorValue;
            }
            if (sensorValue > sensorMax[i]) {
                sensorMax[i] = sensorValue;
            }
        }
    }

    // Threshold for each sensor as avg of min and max
    for (int i = 0; i < 8; i++) {
        sensorThresholds[i] = (sensorMin[i] + sensorMax[i]) / 2;
    }
}

int readSensors() {
    int weightedSum = 0; // Sum of sensor weights
    int sum = 0;         // Count of sensors over threshold

    for (int i = 0; i < 8; i++) {
        SENSOR_VALUES[i] = analogRead(SENSOR_PINS[i]);

        // Using per-sensor threshold determined from calibration
        if (SENSOR_VALUES[i] > sensorThresholds[i]) {
            weightedSum += SENSOR_WEIGHTS[i];
            sum++;
        }
    }

    // If no sensor detects the line, use previous error as fallback
    if (sum == 0) {
        return (_previousError > 0) ? 1000 : -1000;
    }
    return weightedSum / sum;
}

void PIDcontrol(int error) {
    _P = error;
    _integral += error;

    _integral = constrain(_integral, -1000, 1000);

    int D = error - _previousError;
    int PIDvalue = (_Kp * _P) + (_Ki * _integral) + (_Kd * D);
    _previousError = error;

    int leftSpeed = _baseSpeed - PIDvalue;
    int rightSpeed = _baseSpeed + PIDvalue;

    motor_control(leftSpeed, rightSpeed);
}

void motor_control(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    if (leftSpeed >= 0) {
        analogWrite(MOTOR_A_2, leftSpeed);
        analogWrite(MOTOR_A_1, 0);
    } else {
        analogWrite(MOTOR_A_2, 0);
        analogWrite(MOTOR_A_1, abs(leftSpeed));
    }

    if (rightSpeed >= 0) {
        analogWrite(MOTOR_B_1, rightSpeed);
        analogWrite(MOTOR_B_2, 0);
    } else {
        analogWrite(MOTOR_B_1, 0);
        analogWrite(MOTOR_B_2, abs(rightSpeed));
    }
}

void moveForward() { 
    analogWrite(MOTOR_A_1, 0); 
    analogWrite(MOTOR_A_2, 170); 
    analogWrite(MOTOR_B_1, 170); 
    analogWrite(MOTOR_B_2, 0); 
}

void moveBackward() { 
    analogWrite(MOTOR_A_1, 170); 
    analogWrite(MOTOR_A_2, 0); 
    analogWrite(MOTOR_B_1, 0); 
    analogWrite(MOTOR_B_2, 170); 
} 

void turnRight() { 
    analogWrite(MOTOR_A_1, 0); 
    analogWrite(MOTOR_A_2, 170);  
    analogWrite(MOTOR_B_1, 0); 
    analogWrite(MOTOR_B_2, 170); 
} 

void turnLeft() { 
    analogWrite(MOTOR_A_1, 170); 
    analogWrite(MOTOR_A_2, 0); 
    analogWrite(MOTOR_B_1, 170);  
    analogWrite(MOTOR_B_2, 0); 
} 

void stopMotors() { 
    analogWrite(MOTOR_A_1, 0); 
    analogWrite(MOTOR_A_2, 0); 
    analogWrite(MOTOR_B_1, 0); 
    analogWrite(MOTOR_B_2, 0); 
}