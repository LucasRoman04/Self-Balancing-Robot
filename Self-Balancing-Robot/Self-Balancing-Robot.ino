#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include <avr/wdt.h>


//speed constants
#define MIN_SPEED 120
#define MAX_SPEED 200
const float MARGIN = 2;

// Define the motor control pins / mpu object
const int enA = 2;  // Enable for Motor A
const int in1 = 3;  // Input 1 for Motor A
const int in2 = 4;  // Input 2 for Motor A
const int enB = 5;  // Enable for Motor B
const int in3 = 6;  // Input 1 for Motor B
const int in4 = 7;  // Input 2 for Motor B

MPU6050 mpu;

/*********Tune these 4 values for your BOT*********/
double kp = 18;  //Set this first(22.5)
double ki = 0;   //Finally set this
double kd = 0;   //Set this second
/******End of values setting*********/

int16_t accY, accZ;
float accAngle, currentAngle, prevAngle = 0;

float setpoint = 5;  //set the value when the bot is perpendicular to ground using serial monitor.
float error, prev_error = 0;
float integral = 0;
float derivative = 0;
float pidOutput = 0;

// Kalman filter variables
float Q_angle = 0.001;  // Process noise covariance for the accelerometer
float R_angle = 0.01;   // Measurement noise covariance for the accelerometer

float P[2][2] = { { 1, 0 }, { 0, 1 } };  // Initial estimation error covariance matrix
float angle = 0;                         // Initial state estimate
float bias = 0;                          // Initial bias estimate
float rate;                              // Gyroscope reading

unsigned long prevTime = 0;
float dt;  // Time step

void setup() {
  mpu.initialize();
  Serial.begin(115200);

  // Set the motor control pins as output
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  wdt_enable(WDTO_15MS);
  // Stop motors by default
  motorStop();
}

void loop() {
  wdt_reset();
  // Calculate delta time
  unsigned long currTime = millis();
  dt = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  // Read sensor data
  accZ = mpu.getAccelerationZ();
  accY = mpu.getAccelerationY();
  accAngle = atan2(accZ, accY) * RAD_TO_DEG;

  // Gyroscope reading
  // Note: Uncomment this section if you want to use gyroscope data
  int16_t gyroX = mpu.getRotationX();
  rate = map(gyroX, -32768, 32767, -250, 250);

  // Kalman filter predict step
  angle += dt * (rate - bias);
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += R_angle * dt;

  // Kalman filter update step
  float y = accAngle - angle;
  float S = P[0][0] + R_angle;
  float K[2];  // Kalman gain
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  angle += K[0] * y;
  bias += K[1] * y;
  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];

  // Calculate PID output
  error = angle - setpoint;
  integral += error * dt;
  derivative = (error - prev_error) / dt;
  pidOutput = kp * error + ki * integral + kd * derivative;

  // Print values for testing
  // Serial.print("Angle: ");
  // Serial.print(accAngle);
  Serial.print(" error: ");
  Serial.print(error);


  // Send power to the motors
  powerMotors(pidOutput);

  prev_error = error;
}

void powerMotors(double pidOutput) {
  // Constrain motor speed between min and max speed
  int motorSpeed = constrain(abs(pidOutput), MIN_SPEED, MAX_SPEED);

  Serial.print(" Speed: ");
  Serial.println(motorSpeed);

  if (abs(error) > MARGIN) {
    // Serial.println("Movin");
    analogWrite(enA, motorSpeed);  // Set speed for Motor A
    analogWrite(enB, motorSpeed);  // Set speed for Motor B

    if (angle > setpoint) {
      // Falling forward
      // Serial.println(" forward");
      motorForward();
    } else if (angle < setpoint) {
      // Falling backward
      // Serial.println(" backward");
      motorBackward();
    } else {
      // Balanced
      motorStop();
    }
  } else {
    // Serial.println("not movin");
    motorStop(); // Set speed for Motor B
  }
}

// Function to run both motors forward
void motorForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

// Function to run both motors backward
void motorBackward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

// Function to stop both motors
void motorStop() {
  analogWrite(enA, 0);  // Stop Motor A
  analogWrite(enB, 0);  // Stop Motor B
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
