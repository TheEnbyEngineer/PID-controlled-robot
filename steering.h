// Robot differential steering control
// This library can turn the robot to a set angle with reasonable accuracy, by using the feedback provided MPU6050, and calculating the "throttle" of each of the motors
// Raspberry Pi PICO I2C Connections GP4: SDA   GP5: SCL
// Move counter-clockwise: Positive displacement
// This library is licensed under the GNU GPL (v3) license, see license.txt for terms and conditions
// External libraries used
// https://www.arduino.cc/reference/en/libraries/pid/
// https://www.arduino.cc/reference/en/libraries/rotaryencoder/

#include <Wire.h>
#include <MPU6050_light.h>
#include <PID_v1.h>

// Variables that does not need to be tuned
MPU6050 mpu(Wire);
unsigned long yawTimer = 0;
double yawSetpoint = 0, yawInput = 0, yawOutput = 0;

// Specify the links and initial tuning parameters (Yaw)
double yawProportional = 5, yawIntegral = 0, yawDerivative = 0.05, yawTolerance = 5;
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawProportional, yawIntegral, yawDerivative, DIRECT);

// Activate all the relavent systems need to steer the robot based on angle
void engageSteering(int motorLeftFront, int motorLeftBack, int motorRightFront, int motorRightBack) {
  // Starting the hardware
  Wire.begin();
  // Left motor
  pinMode(motorLeftFront, OUTPUT); // Forward
  pinMode(motorLeftBack, OUTPUT); // Back
  // Right motor
  pinMode(motorRightFront, OUTPUT); // Forward
  pinMode(motorRightBack, OUTPUT); // Back

  // Initalizing PID
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-255, 255);

  // Debug
  // Start serial
  /*
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Press enter to start");
  while (!Serial.available()) {}
  */

  // Initalizing the MPU6050
  byte status = mpu.begin();
  //Serial.print(F("MPU6050 status: "));
  //Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050

  // Calibrating
  //Serial.println(F("Calculating offsets, do not move the robot"));
  delay(5000);
  mpu.calcOffsets(); // gyro and accelero
  //Serial.println("Done!\n");
}

void turnRobot(int motorLeftFront, int motorLeftBack, int motorRightFront, int motorRightBack) {

  delay(1000);

  while (1==1) {
    if ((millis()-yawTimer)>10) { // Main control loop

      // Feedback
      mpu.update();
      yawInput = mpu.getAngleZ();

      // Compute PID
      yawPID.Compute();

      // Output
      if (yawOutput > 1) {
        analogWrite(motorLeftFront, map(yawOutput, 0, 255, 100, 255));
        analogWrite(motorLeftBack,   0);
        analogWrite(motorRightFront, 0);
        analogWrite(motorRightBack, map(yawOutput, 0, 255, 100, 255));
      }
      else if (yawOutput < -1) {
        analogWrite(motorLeftFront, 0);
        analogWrite(motorLeftBack,   map(yawOutput*-1, 0, 255, 100, 255));
        analogWrite(motorRightFront, map(yawOutput*-1, 0, 255, 100, 255));
        analogWrite(motorRightBack, 0);
      }
      else {
        analogWrite(motorLeftFront,  0);
        analogWrite(motorLeftBack,   0);
        analogWrite(motorRightFront, 0);
        analogWrite(motorRightBack,  0);
      }

      // Shutdown when target is reached
      if (((yawInput - yawSetpoint) < yawTolerance) and ((yawInput - yawSetpoint) > yawTolerance*-1)) {
        yawSetpoint = yawInput; // Once the position is within yawTolerance, the setpoint would be set to the same value as the feedback position, even if it is not the same, to effective "shutdown" the controller
        analogWrite(motorLeftFront,  0);
        analogWrite(motorLeftBack,   0);
        analogWrite(motorRightFront, 0);
        analogWrite(motorRightBack,  0);
        break;
      }

      // PID output debug
      /*
      Serial.print("Target yaw: ");
      Serial.print(yawSetpoint);
      Serial.print(" Current Yaw: ");
      Serial.print(mpu.getAngleZ());
      Serial.print(" Error: ");
      Serial.print(mpu.getAngleZ() - yawSetpoint);
      Serial.print(" Steering power: ");
      Serial.println(yawOutput);
      */
      yawTimer = millis();
    }
  }
}
