#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

AF_DCMotor motor1(3, MOTOR12_64KHZ);
AF_DCMotor motor2(4, MOTOR12_64KHZ);

Adafruit_MPU6050 mpu;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Motor test!");

  //set max speed for motors
  motor1.setSpeed(200);
  motor2.setSpeed(200);

  Serial.println("Mpu 6050 test!");
  //if mpu fails to start, prompt
  if(!mpu.begin()){
    Serial.println("Failed to find MPU 6050 chip");
    while(1){
      delay(10);
    }
  }
  Serial.println("MPU6050 found");
 
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void loop() {
  // put your main code here, to run repeatedly:

 
  // Serial.print("tick");
  
  // motor1.run(FORWARD);
  // motor2.run(FORWARD);            // turn it on going forward
  // delay(1000);

  // Serial.print("tock");
  // motor1.run(BACKWARD);
  // motor2.run(BACKWARD);           // the other way
  // delay(1000);
  
  // Serial.print("tack");
  // motor1.run(RELEASE);
  // motor2.run(RELEASE);      
  // delay(1000);

    /* Print out the values */

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("accel X: ");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print("accel y: ");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print("accel z: ");
  Serial.print(a.acceleration.z);
  Serial.print(", ");
  Serial.print("gyro x: ");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print("gyro y: ");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print("gyro z: ");
  Serial.print(g.gyro.z);
  Serial.println("");

  delay(100);

  motor1.run(FORWARD);
  motor2.run(FORWARD);

//check if robot is leaning forward
  if(g.gyro.x > .16){
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
  }
  //check if robot is leaning backward
  if(g.gyro.x < .16){
    motor1.run(FORWARD);
    motor2.run(FORWARD);
  }

  


}
