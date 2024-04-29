#ifndef Robot_H
#define Robot_H

#include "Arduino.h"
#include "AFMotor.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <AFMotor.h>
#include <PID_v1.h>

class Robot {

private:
  const float SET_POINT = 85;
  const float MIN_SPEED = 103;
  const float MAX_SPEED = 255;

  const float FORWARD_MARGIN = 0.001;
  const float BACK_MARGIN = 0.001;
  const float SAMPLE_TIME = 0.005; //Time in ms between readings
  const float KP = 25;
  const float KI = 200;
  const float KD = 0;

  float motorSpeed;
  float proportional;
  float integral = 0;
  float derivative = 0;

  float difference;
  float lastDifference = 0;

  long currentTime = 0;
  long previousTime = 0;
  long elapsedTime = 0;

  Adafruit_MPU6050 mpu;
  AF_DCMotor motor1;
  AF_DCMotor motor2;
  sensors_event_t a, g, temp;

public:
  Robot();
  void init();
  void run();
};

#endif