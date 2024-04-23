#include <PID_v1.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "math.h"

#define KP 40
#define KI 40
#define KD 0.05
#define sampleTime 0.005
#define targetAngle -2.5;
#define MIN_ABS_SPEED 20


//motor / sensor objects
AF_DCMotor motor1(3, MOTOR12_64KHZ);
AF_DCMotor motor2(4, MOTOR12_64KHZ);
Adafruit_MPU6050 mpu;

int16_t accelY, accelZ, gyroX;
volatile int gyroRate, motorPower;
volatile float accelAngle, gyroAngle = 0, currentAngle, prevAngle = 0, error, prevError=0, errorSum=0;
volatile byte count = 0;

void setup() {
  //start serial clock
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
 
  //set hardware modes
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void loop() {
  //motor test
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

  //set up event variables and collect a reading
  sensors_event_t a, g, temp;
  unsigned long currTime, prevTime = 0, loopTime;
  mpu.getEvent(&a, &g, &temp);


  // /* Print out the values */
  // Serial.print("accel X: ");
  // Serial.print(a.acceleration.x);
  // Serial.print(",");
  // Serial.print("accel y: ");
  // Serial.print(a.acceleration.y);
  // Serial.print(",");
  // Serial.print("accel z: ");
  // Serial.print(a.acceleration.z);
  // Serial.print(", ");
  // Serial.print("gyro x: ");
  // Serial.print(g.gyro.x);
  // Serial.print(",");
  // Serial.print("gyro y: ");
  // Serial.print(g.gyro.y);
  // Serial.print(",");
  // Serial.print("gyro z: ");
  // Serial.print(g.gyro.z);
  // Serial.println("");

currTime = millis();
loopTime = currTime - prevTime;
prevTime = currTime;

accelZ = a.acceleration.z;
accelY = a.acceleration.y;
gyroX = g.gyro.x;





  delay(100);
}

//to be called every 5 milliseconds 
ISR(TIMER1_COMPA_vect){
  //calculate inclination angle
  accelAngle = atan2(accelY, accelZ) * RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;
  //put reading through complimentary filter
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accelAngle);

  //figure out error
  error = currentAngle - targetAngle;
  errorSum += error;
  errorSum = constrain(errorSum, -300, 300);

  //calculate output from PID
  motorPower = KP *(error) + KI *(errorSum)*sampleTime - KD * (currentAngle - prevAngle) / sampleTime;
  prevAngle = currentAngle;



}
