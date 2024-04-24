#include <Wire.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define SET_POINT 83
#define MIN_DIFFERENCE 0
#define MAX_DIFFERENCE 20
#define MIN_SPEED 100
#define MAX_SPEED 255
#define proportional 10

Adafruit_MPU6050 mpu;
AF_DCMotor motor1(4);
AF_DCMotor motor2(3);

void setup() {
  Serial.begin(9600);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU 6050 chip");
    while (1) {
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
  difference = difference < 0 ? -difference : difference;

  // Serial.print("Angle: ");
  // Serial.println(accelAngle, 5);
  // Serial.print("Difference: ");
  // Serial.println(difference, 5);

  // Motor speed using proportional
  float motorSpeed = proportional * difference;

  // Motor speed using map
  // float motorSpeed = map(difference, MIN_DIFFERENCE, MAX_DIFFERENCE, 100, 255);

  motorSpeed = constrain(motorSpeed, MIN_SPEED, MAX_SPEED);
  motor1.setSpeed(motorSpeed);
  motor2.setSpeed(motorSpeed);

  Serial.print("Speed: ");
  Serial.println(motorSpeed);

  if (accelAngle > SET_POINT && difference > 1.5) {
    //falling backward
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);

  } else if (accelAngle < SET_POINT && difference > 1.5) {
    // falling forward
    motor1.run(FORWARD);
    motor2.run(FORWARD);
  } else {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
  }
}
