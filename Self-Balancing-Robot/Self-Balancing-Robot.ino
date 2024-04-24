#include <Wire.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define SET_POINT 80
#define MAX_SPEED 0
 
Adafruit_MPU6050 mpu;
AF_DCMotor motor1(4);
AF_DCMotor motor2(3);
 
void setup() {
  Serial.begin(9600);        

  if(!mpu.begin()){
    Serial.println("Failed to find MPU 6050 chip");
    while(1){
      delay(10);
     }
  }
 
  Serial.println("MPU6050 found");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}
 
void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  /* Print out the values */
  // Serial.print("Acceleration x: ");
  // Serial.println(a.acceleration.x, 5);
  float accelAngle = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
  float difference = accelAngle - SET_POINT;
  difference = angleError < 0 ? -angleError : angleError;
   
  Serial.print("Angle: ");
  Serial.println(accelAngle, 5);
  // Serial.print("Error: ");
  // Serial.println(angleError, 5);


  motor1.setSpeed(MAX_SPEED);
  motor2.setSpeed(MAX_SPEED);

  if(accelAngle ){
    //falling backward
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
  }
  else if (a.acceleration.x < .30) {
    //falling forward
    motor1.run(FORWARD);
    motor2.run(FORWARD);
  }
  else{
    //balanced
    motor1.run(RELEASE);
    motor2.run(RELEASE);    
  }


}
