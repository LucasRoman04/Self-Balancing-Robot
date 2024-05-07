#include <Wire.h>
#include <MPU6050.h>
#include <avr/wdt.h> 

// Define the motor control pins
const int enA = 2;  // Enable for Motor A
const int in1 = 3;  // Input 1 for Motor A
const int in2 = 4;  // Input 2 for Motor A
const int enB = 5;  // Enable for Motor B
const int in3 = 6;  // Input 1 for Motor B
const int in4 = 7;  // Input 2 for Motor B


// MPU control/status vars
MPU6050 mpu;

/*********Tune these values for your BOT*********/
double kp = 20;  //Set this first
double ki = 0;  //Finally set this
double kd = 0;  //Set this secound
/******End of values setting*********/


float setpoint = 16320; 
float angle = 0;
float error, prev_error = 0;
float integral = 0;
float derivative = 0;
float pidOutput = 0;

float accelY = 0;
float accelX = 0;
float accelZ = 0;
float accel_angle = 0;
float gyroX  = 0;
float gyroRate = 0;
float gyroAngle = 0;
float sampleTime = 0.01;
float filterCoEfficient = 0.98;
float margin = 0;


void setup() {

  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  // mpu offsets
  mpu.setXGyroOffset(134);
  mpu.setYGyroOffset(-1);
  mpu.setZGyroOffset(20);
  mpu.setXAccelOffset(-1583);
  mpu.setYAccelOffset(-2605);
  mpu.setZAccelOffset(2210);

  // Set the motor control pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //stop motors by default
  motorStop();
  wdt_enable(WDTO_1S);
}

void loop() {
  wdt_reset();

  //grab loop time
  
  // Read sensor data
  accelZ = mpu.getAccelerationZ();

  //filter results
  // Serial.print("accel angle : ");
  // Serial.print(accelZ);
  // Serial.print(" accel Y ");
  // Serial.print(accelY);

  error = accelZ - setpoint;
  error = error < 0 ? -error : error;


  pidOutput = (kp * error) ;

  Serial.print(" error: ");
  Serial.print(error);
  // Serial.print(" integral: ");
  // Serial.print(integral);
  Serial.print(" pid output: ");
  Serial.println(pidOutput);
  // Serial.println(" ");

  //send power to the motors
  //powerMotors(pidOutput);


  prev_error = error;
}

void powerMotors(double output) {
  int motorSpeed = constrain(abs(pidOutput), 0, 255);
  // Serial.print("motor speed: ");
  // Serial.println(motorSpeed);
  int leftSpeed = motorSpeed;
  int rightSpeed = leftSpeed;

  if (pidOutput > 0 ) {
    //falling forward
   motorBackward();
  } else if (pidOutput < 0) {
    //falling backward
    motorForward();
  } else {
    //balanced
    motorStop();
  }


  analogWrite(enA, motorSpeed);  // Set speed for Motor A
  analogWrite(enB, motorSpeed);  // Set speed for Motor B
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
