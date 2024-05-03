#include <AFMotor.h>
#include "Arduino.h"
#include "AFMotor.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <avr/wdt.h>

Adafruit_MPU6050 mpu;
AF_DCMotor motor1(3);
AF_DCMotor motor2(4);

sensors_event_t a, g, temp;

double error = 0;
double gyroRate = 0;
double frontMargin = 0.75;
double backMargin = 0.75;

double setpoint = 86.35;
double angle;
double motorSpeed = 0;
double minSpeed = 0;
double maxSpeed = 255;
long currentTime = 0;
long elapsedTime = 0;
long prevTime = 0;
double sampleTime = 0;

double kp = 23;

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
  wdt_disable();
  wdt_enable(WDTO_15MS);


  //  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  //reset watchdog
  wdt_reset();
  //grab reading
  if (mpu.getEvent(&a, &g, &temp)) {
    //find angle of tilt
    angle = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
    // gyroRate = g.gyro.y;

    //get the time
    currentTime = millis();
    elapsedTime = currentTime - prevTime;

    if (elapsedTime > sampleTime) {
      prevTime = currentTime;
      //Find error
      error = angle - setpoint;
      // error = abs(error);

      //proportional
      motorSpeed = minSpeed + (kp * error);
      motorSpeed = constrain(motorSpeed, minSpeed, maxSpeed);

      //print data
      // Serial.print("Error: ");
      // Serial.println(error);
      // Serial.print("Speed: ");
      // Serial.println(motorSpeed);
      Serial.print("GyroRate Y: ");
      Serial.println(gyroRate);

      // motor1.setSpeed(motorSpeed);
      // motor2.setSpeed(motorSpeed);
      if (angle > setpoint && error > frontMargin) {
        //falling forward
        motor1.run(FORWARD);
        motor2.run(FORWARD);
      } else if (angle < setpoint && error > backMargin) {
        //falling backward
        motor1.run(BACKWARD);
        motor2.run(BACKWARD);
      } else {
        //at setpoint
        motor1.run(RELEASE);
        motor2.run(RELEASE);
      }
    } else {
      Serial.println("not enough time passed");
    }

      error = 0;
  }

}
