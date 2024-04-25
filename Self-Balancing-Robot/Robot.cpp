#include "Robot.h"

void Robot::run(){


  mpu.getEvent(&this->a, &this->g, &this->temp);

  float accelAngle = atan2(this->a.acceleration.y, this->a.acceleration.z) * RAD_TO_DEG;
  float difference = accelAngle - this->SET_POINT;
  difference = difference < 0 ? -difference : difference;

  // Serial.print("Angle: ");
  // Serial.println(accelAngle, 5);
  // Serial.print("Difference: ");
  // Serial.println(difference, 5);

  // Motor speed using PROPORTIONAL
  float motorSpeed = (this->PROPORTIONAL * difference);

  // Motor speed using map
  // float motorSpeed = map(difference, MIN_DIFFERENCE, MAX_DIFFERENCE, 100, 255);

  motorSpeed = constrain(motorSpeed, MIN_SPEED, MAX_SPEED);
   this->motor1.setSpeed(motorSpeed);
   this->motor2.setSpeed(motorSpeed);

  if (accelAngle > this->SET_POINT && difference > this->FORWARD_MARGIN) {

    // motorSpeed += 45;
    motorSpeed += map(difference, 0, 20, 0, 135);
    motorSpeed = constrain(motorSpeed, this->MIN_SPEED, this->MAX_SPEED);
    this->motor1.setSpeed(motorSpeed);
    this->motor2.setSpeed(motorSpeed);

    //falling backward
    this->motor1.run(BACKWARD);
    this->motor2.run(BACKWARD);

  } else if (accelAngle < this->SET_POINT && difference > this->BACK_MARGIN) {

    this->motor1.run(FORWARD);
    this->motor2.run(FORWARD);
  } else {
    this->motor1.run(RELEASE);
    this->motor2.run(RELEASE);
  }
  Serial.print("Speed: ");
  Serial.println(motorSpeed);
}

Robot::Robot() : motor1(3), motor2(4){
  
 
}

void Robot::init(){
   //check that the gyro is working
  if(!mpu.begin()){
    Serial.println("Failed to find MPU 6050 chip");
    while(true){
      delay(10);
    }
  }
  Serial.println("MPU 6050 found");

  //set mpu modes
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

}