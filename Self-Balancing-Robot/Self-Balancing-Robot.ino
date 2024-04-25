#include "./Robot.h"

Robot robot;

void setup() {
  Serial.begin(9600);
  robot.init();
}

void loop() {
  robot.run();
}
