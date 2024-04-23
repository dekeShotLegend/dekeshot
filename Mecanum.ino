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
  setAllMotorSpeeds(200);
}

void loop() {
  moveDiagonalForwardRight();
  moveDiagonalForwardLeft();
  moveForward();
  moveBackward();
  strafeLeft();
  strafeRight();
  performUTurn();
  stopAllMotors();
}

/**********************************************************************CONTROL FUNCTION********************************************************************/
void initializeSerial() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - DC Motor Mecanum Test");
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

void moveDiagonalForwardRight() {
  Serial.println("Moving Diagonal Forward Right");
  frontLeft->run(FORWARD);
  frontRight->run(BACKWARD);
  backLeft->run(BACKWARD);
  backRight->run(FORWARD);
  delay(1000);
}

void moveDiagonalForwardLeft() {
  Serial.println("Moving Diagonal Forward Left");
  frontLeft->run(BACKWARD);
  frontRight->run(FORWARD);
  backLeft->run(FORWARD);
  backRight->run(BACKWARD);
  delay(1000);
}

void moveForward() {
  Serial.println("Moving Forward");
  frontLeft->run(FORWARD);
  frontRight->run(FORWARD);
  backLeft->run(FORWARD);
  backRight->run(FORWARD);
  delay(1000);
}

void moveBackward() {
  Serial.println("Moving Backward");
  frontLeft->run(BACKWARD);
  frontRight->run(BACKWARD);
  backLeft->run(BACKWARD);
  backRight->run(BACKWARD);
  delay(1000);
}

void strafeLeft() {
  Serial.println("Moving Sideways Left");
  frontLeft->run(BACKWARD);
  frontRight->run(FORWARD);
  backLeft->run(BACKWARD);
  backRight->run(FORWARD);
  delay(1000);
}

void strafeRight() {
  Serial.println("Moving Sideways Right");
  frontLeft->run(FORWARD);
  frontRight->run(BACKWARD);
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
