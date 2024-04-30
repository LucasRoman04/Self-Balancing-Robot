#include <ArduPID.h>

#include <AFMotor.h>
#include "Arduino.h"
#include "AFMotor.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
AF_DCMotor motor1(3);
AF_DCMotor motor2(4);

sensors_event_t a, g, temp;

double error       = 0;
double errorSum    = 0;
double currentTime = 0;
double elapsedTime = 0;
double prevTime    = 0;
double frontMargin = 1.5;
double backMargin  = 1.5;


double setpoint = 87.75;
double angle;
double motorSpeed = 0;
double integral = 0;
double minSpeed = 103;
double maxSpeed = 255;

double kp = 23;
double ki = 200;
double kd = 0;

ArduPID pid;

void setup() {
  Serial.begin(9600);

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

  pid.begin(&angle, &motorSpeed, &setpoint, kp, ki, kd);
}

void loop() {
  //grab reading
  mpu.getEvent(&a, &g, &temp);

  currentTime = millis();
  elapsedTime = currentTime - prevTime;
  prevTime    = currentTime;

  //find angle of tilt
  angle = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;

  //Find error
  error = angle - setpoint;
  error = error < 0 ? -error : error;
  errorSum += error;
  errorSum = constrain(errorSum, 0, 300);

  //compute output using PID controller
  // pid.compute();

  motorSpeed = (kp * error) ;//proportional
  integral = ki * errorSum; //integral
  motorSpeed = constrain(motorSpeed, minSpeed, maxSpeed);

  // Serial.print("Angle: ");
  // Serial.println(angle);
  Serial.print("Speed: ");
  Serial.println(motorSpeed);

  motor1.setSpeed(motorSpeed);
  motor2.setSpeed(motorSpeed);
  if (angle > setpoint && error > frontMargin) {
    //falling forward
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
  } else if (angle < setpoint && error > backMargin) {
    //falling backward
    motor1.run(FORWARD);
    motor2.run(FORWARD);
  } else if(angle == setpoint){
    //at setpoint
    motor1.run(RELEASE);
    motor2.run(RELEASE);
  }
}
