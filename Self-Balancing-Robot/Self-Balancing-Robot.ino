#include <Wire.h>
#include <MPU6050.h>
#include <avr/wdt.h>

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
double kp = 4;  //Set this first
double ki = 0;  //Finally set this
double kd = 0;  //Set this secound
/******End of values setting*********/


float setpoint = 19;  //set the value when the bot is perpendicular to ground using serial monitor.
float angle = 0;
float error, prev_error = 0;
float integral = 0;
float derivative = 0;
float pidOutput = 0;
int margin = 1;


void setup() {
  //start hardware
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // mpu.setXGyroOffset(0);
  // mpu.setYGyroOffset(0);
  // mpu.setZGyroOffset(0);
  // mpu.setXAccelOffset(-1583);
  // mpu.setYAccelOffset(-2605);
  // mpu.setZAccelOffset(2210);

  // Set the motor control pins as output
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //stop motors by default
  motorStop();

  //start watchdog
  wdt_enable(WDTO_120MS);
}

void loop() {
  wdt_reset();
  // Read sensor data
  angle = mpu.getAccelerationZ() / 100;

  //find how off we are from the setpoint
  error = angle - setpoint;
  // integral += error;
  // derivative = error - prev_error;

  error = error < 0 ? -error : error;

  //generate a motor response relative to the amount of error
  pidOutput = (kp * error) + (ki * integral) + (kd * derivative);

  //print values for testing
  Serial.print("angle: ");
  Serial.print(angle);
  Serial.print(" error: ");
  Serial.print(error);
  // Serial.print(" integral: ");
  // Serial.print(integral);
  // Serial.print(" Pid output: ");
  // Serial.println(pidOutput);

  //send power to the motors
  powerMotors(pidOutput);

  // prev_error = error;
}

void powerMotors(double output) {
  //constrain motorspeed between min and max speed
  int motorSpeed = constrain(abs(pidOutput), MIN_SPEED, MAX_SPEED);

  Serial.print(" motorspeed: ");
  Serial.println(motorSpeed);
  analogWrite(enA, motorSpeed);  // Set speed for Motor A
  analogWrite(enB, motorSpeed);  // Set speed for Motor B

  if (angle > setpoint) {
    //falling forward
    motorForward();
  } else if (angle < setpoint) {
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