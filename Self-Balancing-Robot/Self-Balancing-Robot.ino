#include <Wire.h>
#include <MPU6050.h>

// Define the motor control pins
const int enA = 2;  // Enable for Motor A
const int in1 = 3;  // Input 1 for Motor A
const int in2 = 4;  // Input 2 for Motor A
const int enB = 5;  // Enable for Motor B
const int in3 = 6;  // Input 1 for Motor B
const int in4 = 7;  // Input 2 for Motor B


// MPU control/status vars
MPU6050 mpu;

/*********Tune these 4 values for your BOT*********/
double kp = 1;  //Set this first
double ki = 0;  //Finally set this
double kd = 0;  //Set this secound
/******End of values setting*********/


float setpoint = 103;  //set the value when the bot is perpendicular to ground using serial monitor.
float angle = 0;
float error, prev_error = 0;
float integral = 0;
float derivative = 0;
float pidOutput = 0;


void setup() {

  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(0);

  // Set the motor control pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //stop motors by default
  motorStop();
}

void loop() {
  // Read sensor data
  angle = atan2(mpu.getAccelerationY(), mpu.getAccelerationZ()) * RAD_TO_DEG;
  error = angle - setpoint;
  integral += error;
  derivative = error - prev_error;

  pidOutput = (kp * error) + (ki * integral) + (kd * derivative);

  Serial.print("error: ");
  Serial.print(error);
  Serial.print(" integral: ");
  Serial.print(integral);
  Serial.print(" Pid output: ");
  Serial.println(pidOutput);


  //send power to the motors
  powerMotors(pidOutput);

  prev_error = error;
}

void powerMotors(double output) {
  int motorSpeed = constrain(abs(pidOutput), 0, 255);
  int leftSpeed = motorSpeed;
  int rightSpeed = leftSpeed;

  if (pidOutput > 0) {
    //falling forward
    motorForward();
  } else if (pidOutput < 0) {
    //falling backward
    motorBackward();
  } else {
    //balanced
    motorStop();
  }


  analogWrite(enA, 255);  // Set speed for Motor A
  analogWrite(enB, 255);  // Set speed for Motor B
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
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);  // Stop Motor A
  analogWrite(enB, 0);  // Stop Motor B
}
