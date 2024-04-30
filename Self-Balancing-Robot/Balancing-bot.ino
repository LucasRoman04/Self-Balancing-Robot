#include <AFMotor.h>
#include "Arduino.h"
#include "AFMotor.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>


Adafruit_MPU6050 mpu;
AF_DCMotor motor1(3);
AF_DCMotor motor2(4);

sensors_event_t a, g, temp;

double error = 0;
float lastError = 0;
float currentTime = 0;
float elapsedTime = 0;
float lastTime = 0;
float frontMargin = 2;
float backMargin = 2;


double setpoint = 88.75;
double angle;
double motorSpeed = 0;
float minSpeed = 103;
float maxSpeed = 255;

double kp = 30;
double ki = 0;
double kd = 0;

PID pid(&angle, &motorSpeed, &setpoint, kp, ki, kd, DIRECT);



void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU 6050 chip");
    while (true) {
      delay(10);
    }
  }
  Serial.println("MPU 6050 found");

  //set modes
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(103, 255);  //min and max speed of motors
  pid.SetSampleTime(10);          //10ms between samples
}

void loop() {
  // put your main code here, to run repeatedly:

  //grab reading
  mpu.getEvent(&a, &g, &temp);

  //find angle of tilt
  angle = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;

  //Find error
  error = angle - setpoint;
  error = error < 0 ? -error : error;

  //compute output using PID controller
  pid.Compute();
  // motorSpeed = kp * error;
  motorSpeed = constrain(motorSpeed, minSpeed, maxSpeed);

  // motor1.setSpeed(motorSpeed);
  // motor2.setSpeed(motorSpeed);
  if (angle > setpoint && error > frontMargin) {
    //falling forward
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
  } else if (angle < setpoint && error > backMargin) {
    //falling backward

    motor1.run(FORWARD);
    motor2.run(FORWARD);
  }
  // else{
  //   //at setpoint
  //   motor1.run(RELEASE);
  //   motor2.run(RELEASE);
  // }


  //do something with output.


  //print error

  Serial.print("Angle: ");
  Serial.println(angle);
  Serial.print("Speed: ");
  Serial.println(motorSpeed);
}
