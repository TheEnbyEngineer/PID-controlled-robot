#include "motion.h"

// This program would move the robot back and forth by 100 encoder ticks on each side
// You can feel the encoder click when turning the encoder by hand, where on "click" you feel is one encoder tick
// The encoder is the two small devices attached to small wheels mounted above the large wheels, its purpose is to tell the robot how much the wheel has rotated
// Then the robot would rotate the main wheel(s) until it has reached the number of encoder ticks you programmed it to 
// Mathematically this is done using a system called a PID controller
// Use Visual Studio Code for the best programming experience
// This example program is licensed under the GNU GPL (v3) license, see license.txt for terms and conditions

void setup() {
  // This activates the systems needed to move the robot back and forth (The numbers are the pins)
  engageMotionControl(14,15,20,21,6,7,8,9);
}
void loop() {
  motorLeftSetpoint = 100;  // Tell the robot the left motor needs to be rotated 100 encoder ticks from where it is now
  motorRightSetpoint = 100; // Tell the robot the right motor needs to be rotated 100 encoder ticks from where it is now
  moveRobot(6,7,8,9);     // Tell the robot to start moving the motors
  motorLeftSetpoint = 0;    // Tell the robot the left motor needs to be rotated to the position it was when the robot is first powered on
  motorRightSetpoint = 0;   // Tell the robot the right motor needs to be rotated to the position it was when the robot is first powered on
  moveRobot(6,7,8,9);     // Tell the robot to start moving the motors
}
