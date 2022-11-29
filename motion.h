// Robot PID motion control
// This library facilitates the robot to travel back and forth to a set distance with reasonable accuracy, by using the feedback provided by the encoders, and calculating the "throttle" of each of the motors
// This library is licensed under the GNU GPL (v3) license, see license.txt for terms and conditions
// External libraries used
// https://www.arduino.cc/reference/en/libraries/pid/
// https://www.arduino.cc/reference/en/libraries/rotaryencoder/

#include <Arduino.h>
#include <RotaryEncoder.h>
#include <PID_v1.h>

// Variables that does not need to be tuned
double motorLeftSetpoint = 0, motorLeftFeedback = 0, motorLeftOutput = 0;
double motorRightSetpoint = 0, motorRightFeedback = 0, motorRightOutput = 0;
double tolerance = 50;
unsigned long timer = 0;
RotaryEncoder *encoderLeft = nullptr;
RotaryEncoder *encoderRight = nullptr;

// Specify the PID object and initial tuning parameters (motorLeft)
double motorLeftProportional = 7, motorLeftIntegral = 0.1, motorLeftDerivative = 0.1;
PID motorLeftPID(&motorLeftFeedback, &motorLeftOutput, &motorLeftSetpoint, motorLeftProportional, motorLeftIntegral, motorLeftDerivative, DIRECT);


// Specify the PID object and initial tuning parameters (motorRight)
double motorRightProportional = 10, motorRightIntegral = 0.1, motorRightDerivative = 0.1;
PID motorRightPID(&motorRightFeedback, &motorRightOutput, &motorRightSetpoint, motorRightProportional, motorRightIntegral, motorRightDerivative, DIRECT);

// -----------------------------------------------------------------------------------------------------------------------------------

// Check the encoder state
void checkPositionLeft() {
  encoderLeft->tick();
}
void checkPositionRight() {
  encoderRight->tick();
}

// Setup the motion controller
void engageMotionControl(int encoderLeftPin1, int encoderLeftPin2, int encoderRightPin1, int encoderRightPin2, int motorLeftFront, int motorLeftBack, int motorRightFront, int motorRightBack) {
  // Start PID
  motorLeftPID.SetMode(AUTOMATIC);
  motorLeftPID.SetOutputLimits(-255, 255);
  motorRightPID.SetMode(AUTOMATIC);
  motorRightPID.SetOutputLimits(-255, 255);  

  // Start encoder
  encoderLeft = new RotaryEncoder(encoderLeftPin1, encoderLeftPin2, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(encoderLeftPin1), checkPositionLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLeftPin2), checkPositionLeft, CHANGE);
  encoderRight = new RotaryEncoder(encoderRightPin1, encoderRightPin2, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(encoderRightPin1), checkPositionRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRightPin2), checkPositionRight, CHANGE);


  // Start motor
  pinMode(motorLeftFront, OUTPUT);  // Left front
  pinMode(motorLeftBack, OUTPUT);   // Left back
  pinMode(motorRightFront, OUTPUT); // Right front
  pinMode(motorRightBack, OUTPUT);  // Right back

  // Debug
  // Start serial
  /*
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Press enter to start");
  while (!Serial.available()) {}
  */
}

// Move robot back and forth
void moveRobot(int motorLeftFront, int motorLeftBack, int motorRightFront, int motorRightBack) {

  while (1==1) {
    // Feedback
    motorLeftFeedback = encoderLeft->getPosition();
    motorRightFeedback = encoderRight->getPosition();

    // Compute PID
    motorLeftPID.Compute();
    motorRightPID.Compute();

    // Output
    if (motorLeftOutput > 1) {
      analogWrite(motorLeftFront, map(motorLeftOutput, 0, 255, 100, 255));
      analogWrite(motorLeftBack, 0);
    }
    else if (motorLeftOutput < -1) {
      analogWrite(motorLeftFront, 0);
      analogWrite(motorLeftBack, map(motorLeftOutput*-1, 0, 255, 100, 255));
    }
    else {
      analogWrite(motorLeftFront, 0);
      analogWrite(motorLeftBack, 0);
    }
    if (motorRightOutput > 1) {
      analogWrite(motorRightFront, map(motorRightOutput, 0, 255, 100, 255));
      analogWrite(motorRightBack, 0);
    }
    else if (motorRightOutput < -1) {
      analogWrite(motorRightFront, 0);
      analogWrite(motorRightBack, map(motorRightOutput*-1, 0, 255, 100, 255));
    }
    else {
      analogWrite(motorRightFront, 0);
      analogWrite(motorRightBack, 0);
    }

    // Shutdown when target is reached
    if (((((motorLeftFeedback - motorLeftSetpoint) + (motorRightFeedback - motorRightSetpoint)) / 2) < tolerance) and ((((motorLeftFeedback - motorLeftSetpoint) + (motorRightFeedback - motorRightSetpoint)) / 2) > tolerance*-1)) {
      motorLeftSetpoint = motorLeftFeedback; // Once the position is within tolerance, the setpoint would be set to the same value as the feedback position, even if it is not the same, to effective "shutdown" the controller
      motorRightSetpoint = motorRightFeedback; // Once the position is within tolerance, the setpoint would be set to the same value as the feedback position, even if it is not the same, to effective "shutdown" the controller
      analogWrite(motorLeftFront, 0);
      analogWrite(motorLeftBack, 0);
      analogWrite(motorRightFront, 0);
      analogWrite(motorRightBack, 0);
      break;
    }

    // Debug
    /*
    Serial.print("Setpoint: ");
    Serial.print(motorLeftSetpoint);
    Serial.print(" ");
    Serial.print(motorRightSetpoint);
    Serial.print("  Feedback: ");
    Serial.print(motorLeftFeedback);
    Serial.print(" ");
    Serial.print(motorRightFeedback);
    Serial.print("  Throttle: ");
    Serial.print(motorLeftOutput);
    Serial.print(" ");
    Serial.print(motorRightOutput);
    Serial.print("  Error: ");
    Serial.print(motorLeftFeedback - motorLeftSetpoint);
    Serial.print(" ");
    Serial.println(motorRightFeedback - motorRightSetpoint);
    */
  }
}
