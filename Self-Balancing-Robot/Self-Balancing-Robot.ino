
#include <PID_v1_bc.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "math.h"
 
#define MIN_ABS_SPEED 20
 
//motor / sensor objects
AF_DCMotor motor1(3, MOTOR12_64KHZ);
AF_DCMotor motor2(4, MOTOR12_64KHZ);
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

  float accelAngle, gyroAngle = 0;

  unsigned long currTime, prevTime = 0, loopTime;

  mpu.getEvent(&a, &g, &temp);
 
 
   
currTime = millis();
loopTime = currTime - prevTime;
prevTime = currTime;

accelZ = a.acceleration.z;
accelY = a.acceleration.y;
//use accel readings to figure out angle of inclination
accelAngle = atan2(accelY, accelZ) * RAD_TO_DEG;

//basic movement logic
 if(isnan(accelAngle));
  else
    motor1.setSpeed(0);
    motor2.setSpeed(0);

    if(accelAngle > 90){
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
    }
    else if(accelAngle < 90){
      motor1.run(FORWARD);
      motor2.run(FORWARD);
    }
    else{
      motor1.run(RELEASE);
      motor2.run(RELEASE);
    }
 
gyroX = g.gyro.x;

gyroRate = map(gyroX, -32768, 32767, -250, 250);

gyroAngle = gyroAngle + (float)gyroRate * loopTime/1000;

Serial.println(gyroAngle);
 
//do something with readings (TO-DO)
  float middlePoint = 0.16;
  float difference = gyroX - middlePoint;

 

}