#include "./Robot.h"

Robot robot;

void setup() {
  Serial.begin(9600); 
  Serial.println("Setup done");
  robot.init();
}

void loop() {
  robot.run();
}
