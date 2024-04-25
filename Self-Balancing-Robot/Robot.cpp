

// Robot::Robot(){
//   motor1 = AF_DCMotor(4);
//   motor2 = AF_DCMotor(3);

//   if (!mpu.begin()) {
//     Serial.println("Failed to find MPU 6050 chip");
//     while (1) {
//       delay(10);
//     }
//   }

//   Serial.println("MPU6050 found");
//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//   mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
// }