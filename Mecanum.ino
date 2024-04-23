/**
* Developer: Seyi R. Afolayan 
* Work: Mecanum Wheels Logic
**/
/***************************************************************************BEGIN CODE***********************************************************************/

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Instantiate the motor shield object
Adafruit_MotorShield DekeShotMotors = Adafruit_MotorShield();
Adafruit_DCMotor *frontRight;
Adafruit_DCMotor *frontLeft;
Adafruit_DCMotor *backRight;
Adafruit_DCMotor *backLeft;

void setup() {
  initializeSerial();
  initializeMotors();
  setAllMotorSpeeds(50);
}

void loop() {
  moveForward();
  delay(2000); // Stop for 2 seconds

  moveBackward();
  delay(2000); // Stop for 2 seconds

  strafeRight();
  delay(2000); // Stop for 2 seconds

  strafeLeft();
  delay(2000); // Stop for 2 seconds

  turnRight90();
  delay(2000); // Stop for 2 seconds

  turnLeft90();
  delay(2000); // Stop for 2 seconds

  performUTurn();
  delay(2000); // Stop for 2 seconds
  
  stopAllMotors(); // Ensures motors are stopped at the end of the sequence

  while(true) {
    delay(10); // Keeps the Arduino program from looping
  }
}

/**********************************************************************CONTROL FUNCTION********************************************************************/
void initializeSerial() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v3 - DC Motor Mecanum Test");
}

void initializeMotors() {
  DekeShotMotors.begin();
  frontRight = DekeShotMotors.getMotor(1);
  frontLeft = DekeShotMotors.getMotor(2);
  backRight = DekeShotMotors.getMotor(3);
  backLeft = DekeShotMotors.getMotor(4);
}

void setAllMotorSpeeds(int speed) {
  frontLeft->setSpeed(speed);   
  frontRight->setSpeed(speed);    
  backLeft->setSpeed(speed);      
  backRight->setSpeed(speed);     
}

void turnRight90() {
  Serial.println("Turning Right 90 Degrees");
  // Assuming turning right involves these wheel directions for 90 degrees
  frontLeft->run(FORWARD);
  frontRight->run(RELEASE);
  backLeft->run(FORWARD);
  backRight->run(RELEASE);
  delay(500); // Adjust timing based on actual turning speed
}

void turnLeft90() {
  Serial.println("Turning Left 90 Degrees");
  frontLeft->run(RELEASE);
  frontRight->run(FORWARD);
  backLeft->run(RELEASE);
  backRight->run(FORWARD);
  delay(500);
}

void moveDiagonalForwardRight() {
  Serial.println("Moving Diagonal Forward Right");
  frontLeft->run(FORWARD);
  frontRight->run(RELEASE);
  backLeft->run(RELEASE);
  backRight->run(FORWARD);
  delay(1000);
}

void moveDiagonalForwardLeft() {
  Serial.println("Moving Diagonal Forward Left");
  frontLeft->run(RELEASE);
  frontRight->run(FORWARD);
  backLeft->run(FORWARD);
  backRight->run(RELEASE);
  delay(1000);
}

void moveForward() {
  Serial.println("Moving Forward!");
  frontLeft->run(FORWARD);
  frontRight->run(FORWARD);
  backLeft->run(FORWARD);
  backRight->run(FORWARD);
  delay(1000);
}

void moveBackward() {
  Serial.println("Reverse!)");
  frontLeft->run(BACKWARD);
  frontRight->run(BACKWARD);
  backLeft->run(BACKWARD);
  backRight->run(BACKWARD);
  delay(1000);
}


void strafeRight() {
  Serial.println("Strafing Right");
  frontLeft->run(FORWARD);
  frontRight->run(BACKWARD);
  backLeft->run(BACKWARD);
  backRight->run(FORWARD);
  delay(1000);
}

void strafeLeft() {
  Serial.println("Strafing Left");
  frontLeft->run(BACKWARD);
  frontRight->run(FORWARD);
  backLeft->run(FORWARD);
  backRight->run(BACKWARD);
  delay(1000);
}


void performUTurn() {
  Serial.println("Performing U-Turn");
  frontLeft->run(FORWARD);
  frontRight->run(BACKWARD);
  backLeft->run(FORWARD);
  backRight->run(BACKWARD);
  delay(1000);
}

void stopAllMotors() {
  Serial.println("Stopping All Motors");
  frontLeft->run(RELEASE);
  frontRight->run(RELEASE);
  backLeft->run(RELEASE);
  backRight->run(RELEASE);
  delay(1000);
}
