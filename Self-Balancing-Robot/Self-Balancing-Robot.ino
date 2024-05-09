#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

//speed constants
#define MIN_SPEED 0
#define MAX_SPEED 255

// Define the motor control pins / mpu object
const int enA = 2;  // Enable for Motor A
const int in1 = 3;  // Input 1 for Motor A
const int in2 = 4;  // Input 2 for Motor A
const int enB = 5;  // Enable for Motor B
const int in3 = 6;  // Input 1 for Motor B
const int in4 = 7;  // Input 2 for Motor B

MPU6050 mpu;

/*********Tune these 4 values for your BOT*********/
<<<<<<< HEAD
double kp = 26;  //Set this first
=======
double kp = 8;  //Set this first
>>>>>>> 17310d4d4ea1eb05fa0f30e8fe0d65677ba3eb23
double ki = 0;  //Finally set this
double kd = 0;  //Set this secound
/******End of values setting*********/

<<<<<<< HEAD
int16_t gyroX, gyroRate;
float gyroAngle=0;
unsigned long currTime, prevTime=0, loopTime;

int16_t accY, accZ;
float accAngle, currentAngle, prevAngle = 0;

float setpoint = 6.2;  //set the value when the bot is perpendicular to ground using serial monitor.
=======
int16_t accY, accZ;
float accAngle;

float setpoint = 0;  //set the value when the bot is perpendicular to ground using serial monitor.
>>>>>>> 17310d4d4ea1eb05fa0f30e8fe0d65677ba3eb23
float position = 0;
float error, prev_error = 0;
float integral = 0;
float derivative = 0;
float pidOutput = 0;
int margin = 1;

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
  
  //stop motors by default
  motorStop();

}

void loop() {  
<<<<<<< HEAD
  //grab sample time
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;

  //accelerometer data
  accZ = mpu.getAccelerationZ();
  accY = mpu.getAccelerationY();
  accAngle = atan2(accZ, accY)*RAD_TO_DEG;

  // //gyro data
  // gyroX = mpu.getRotationX();
  // gyroRate = map(gyroX, -32768, 32767, -250, 250);
  // gyroAngle = gyroAngle + (float)gyroRate*loopTime/1000;

  // //combine gyro and accelerometer into complimentary filter to find the current angle.
  // currentAngle =  0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accAngle);
  // prevAngle = currentAngle;
  //find how off we are from the setpoint
=======
  accZ = mpu.getAccelerationZ();
  accY = mpu.getAccelerationY();
   
  accAngle = atan2(accZ, accY)*RAD_TO_DEG;

      //find how off we are from the setpoint
>>>>>>> 17310d4d4ea1eb05fa0f30e8fe0d65677ba3eb23
  error = accAngle - setpoint;
  // integral += error;
  // derivative = error - prev_error;

  error = error < 0 ? -error : error;

  //generate a motor response relative to the amount of error
  pidOutput = (kp * error) + (ki * integral) + (kd * derivative);

  //print values for testing
  // Serial.print("bot position: ");
  // Serial.print(accAngle);
  Serial.print(" error: ");
  Serial.println(error);
  // Serial.print("Current angle: ");
  // Serial.println(currentAngle);
  // Serial.print(" integral: ");
  // Serial.print(integral);
  // Serial.print(" Pid output: ");
  // Serial.println(pidOutput);

  //send power to the motors
  powerMotors(pidOutput);
}

void powerMotors(double pidOutput) {
  //constrain motorspeed between min and max speed
  int motorSpeed = constrain(abs(output), MIN_SPEED, MAX_SPEED);

<<<<<<< HEAD
=======
  Serial.print(" motorspeed: ");
  Serial.println(motorSpeed);
>>>>>>> 17310d4d4ea1eb05fa0f30e8fe0d65677ba3eb23
  analogWrite(enA, motorSpeed);  // Set speed for Motor A
  analogWrite(enB, motorSpeed);  // Set speed for Motor B

  if (accAngle > setpoint) {
    //falling forward
    motorForward();
  } else if (accAngle < setpoint) {
    //falling backward
    motorBackward();
  } else {
    //balanced
    motorStop();
  }
}

// Function to run both motors forward
void motorForward() {
  // Serial.println(" backward");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

// Function to run both motors backward
void motorBackward() {
  // Serial.println(" Forward");
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