#ifndef Robot_H
#define Robot_H

#include "Arduino.h"
#include "AFMotor.h"
#include <Wire.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


class Robot{

private:
  const float SET_POINT      = 83;
  const float MIN_DIFFERENCE = 0;
  const float MAX_DIFFERENCE = 20;
  const float MIN_SPEED      = 100;
  const float MAX_SPEED      = 255;
  const float PROPORTIONAL   = 12;
  const float FORWARD_MARGIN = .5;
  const float BACK_MARGIN    = 1.5;

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