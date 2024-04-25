#ifndef Robot_H
#define Robot_H

#include "Arduino.h"
#include "AFMotor.h"
#include <Wire.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


class Robot {

private:
  const float SET_POINT = 81.6;
  const float MIN_SPEED = 103;
  const float MAX_SPEED = 255;
  const float PROPORTIONAL = 19;
  const float FORWARD_MARGIN = .2;
  const float BACK_MARGIN = .2;

  Adafruit_MPU6050 mpu;
  AF_DCMotor motor1;
  AF_DCMotor motor2;
  sensors_event_t a, g, temp;

  void get_reading();
  void movement();

public:
  Robot();
  void init();
  void run();
};

#endif