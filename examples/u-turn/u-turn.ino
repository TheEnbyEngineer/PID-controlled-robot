#include "motion.h"
#include "steering.h"

// This program will get the robot to drive forward, turn around, and drive back
// Use Visual Studio Code for the best programming experience
// This program is licensed under the GNU GPL (v3) license, see license.txt for terms and conditions

void setup() {
  // Activate all relavent systems
  engageSteering(6,7,8,9);
  engageMotionControl(16,17,20,21,6,7,8,9); // The left encoder port stopped working, therefore the left encoder is connected to the SPI port
  pinMode(25,OUTPUT);

  // Turn on the LED
  digitalWrite(25,1);

  // Drive the robot forward
  motorLeftSetpoint = 500;    // Tell the robot the left motor needs to be rotated 100 encoder ticks from where it is now
  motorRightSetpoint = 500;   // Tell the robot the right motor needs to be rotated 100 encoder ticks from where it is now
  moveRobot(6,7,8,9);         // Tell the robot to start moving the motors

  // Turn the robot 180 degrees
  yawSetpoint = 180;          // Tell the robot that it needs to turn 180 degrees from the heading it was when it was powered on
  turnRobot(8,9,6,7);         // Tell the robot to start turning

  delay(500);

  // Drive the robot forward
  motorLeftSetpoint = 1000;   // Tell the robot the left motor needs to be rotated 100 encoder ticks from where it is now
  motorRightSetpoint = 1000;  // Tell the robot the right motor needs to be rotated 100 encoder ticks from where it is now
  moveRobot(6,7,8,9);         // Tell the robot to start moving the motors

  // Turn off the LED
  digitalWrite(25,0);
}
void loop() {
}
