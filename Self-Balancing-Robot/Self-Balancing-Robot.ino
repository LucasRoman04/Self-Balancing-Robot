
#include <PID_v1_bc.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "math.h"
 
#define MIN_ABS_SPEED 20

#define sampleTime 0.005
#define baseAngle 1.22
 
//motor / sensor objects
AF_DCMotor motor1(3);
AF_DCMotor motor2(4);
Adafruit_MPU6050 mpu;
 
 
void setup() {
  //start serial clock
  Serial.begin(9600);
  Serial.println("Motor test!");
  //set max speed for motors
  Serial.println("Mpu 6050 test!");

  //if mpu fails to start, prompt
  if(!mpu.begin()){
    Serial.println("Failed to find MPU 6050 chip");
    while(1){
      delay(10);
    }
  }

  Serial.println("MPU6050 found");
 
  //set hardware modes
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

}
 
void loop() {

  //set up event variables and collect a reading

  sensors_event_t a, g, temp;

  int accelY, accelZ, gyroX, gyroRate, motorPower;

  float accelAngle, gyroAngle = 0, prevAngle = 0;

  volatile float currentAngle;

  unsigned long currTime, prevTime = 0, loopTime;



  mpu.getEvent(&a, &g, &temp);
 
 
   
currTime = millis();
loopTime = currTime - prevTime;
prevTime = currTime;

accelZ = a.acceleration.z;
accelY = a.acceleration.y;
//use accel readings to figure out angle of inclination
accelAngle = atan2(accelY, accelZ) * RAD_TO_DEG;

gyroX = g.gyro.x;

gyroRate = map(gyroX, -32768, 32767, 0, 255);

// gyroAngle = gyroAngle + (float)gyroRate * loopTime/1000;

// Serial.print("GyroAngle: ");
Serial.println(gyroAngle);

gyroAngle = (float) gyroRate * sampleTime;

currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accelAngle);

prevAngle = currentAngle;

Serial.print("current angle: ");
Serial.println(currentAngle);

//basic movement logic
 if(isnan(accelAngle));
  else
     motor1.setSpeed(115);
     motor2.setSpeed(115);
    // Serial.print("Accel Angle: ");
    // Serial.println(accelAngle);
    if(currentAngle > (baseAngle + 0.005)){
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
    } 
    else if(currentAngle < (baseAngle - 0.005)){
      motor1.run(FORWARD);
      motor2.run(FORWARD);
    }
    else{
      motor1.run(RELEASE);
      motor2.run(RELEASE);
    }
 



}