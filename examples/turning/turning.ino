#include "steering.h"

// This program would turn the robot left and right by 90 degrees
// It works by getting the current heading using the gyroscope module (the blue module on the robot labled MPU6050)
// And then using that to calculate how much power to apply to each of the motors to turn the robot to the programmed target angle
// Mathematically this is done using a system called a PID controller
// Move counter-clockwise: Positive displacement
// Use Visual Studio Code for the best programming experience
// This program is licensed under the GNU GPL (v3) license, see license.txt for terms and conditions

void setup() {
  // This activates the systems needed to steer the robot to a set angle
  engageSteering(6,7,8,9);
}
void loop() {
  yawSetpoint = 90;    // Tell the robot that it needs to turn 90 degrees counter-clockwise (left) from the heading it was when it was powered on
  turnRobot(6,7,8,9);  // Tell the robot to start turning
  delay(1000);
  yawSetpoint = 0;     // Tell the robot that it needs to return to the heading it was when it was powered on
  turnRobot(6,7,8,9);  // Tell the robot to start turning
  delay(1000);
}
