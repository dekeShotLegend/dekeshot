// Developer: Seyi R. Afolayan
#ifndef MECANUM_ROBOT_H
#define MECANUM_ROBOT_H

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Arduino.h>

class MecanumRobot {
public:
    MecanumRobot() : motorShield(Adafruit_MotorShield()), speed(50) {
        initializeMotors();
    }

    void initializeMotors() {
        motorShield.begin();
        frontRight = motorShield.getMotor(1);
        frontLeft = motorShield.getMotor(2);
        backRight = motorShield.getMotor(3);
        backLeft = motorShield.getMotor(4);
    }

    void setAllMotorSpeeds(int newSpeed) {
        speed = newSpeed;
        frontLeft->setSpeed(speed);
        frontRight->setSpeed(speed);
        backLeft->setSpeed(speed);
        backRight->setSpeed(speed);
    }
        // Set individual motor speeds for advanced maneuvers
    void setMotorSpeeds(int leftSpeed, int rightSpeed) {
        frontLeft->setSpeed(leftSpeed);
        backLeft->setSpeed(leftSpeed);
        frontRight->setSpeed(rightSpeed);
        backRight->setSpeed(rightSpeed);
    }

    void moveForward() {
        Serial.println("Moving Forward!");
        runAllMotors(FORWARD);
    }

    void moveBackward() {
        Serial.println("Reverse!");
        runAllMotors(BACKWARD);
    }

    void strafeRight() {
        Serial.println("Strafing Right");
        frontLeft->setSpeed(speed);
        frontRight->setSpeed(speed);
        backLeft->setSpeed(speed);
        backRight->setSpeed(speed);
        frontLeft->run(FORWARD);
        frontRight->run(BACKWARD);
        backLeft->run(BACKWARD);
        backRight->run(FORWARD);
    }

    void strafeLeft() {
        Serial.println("Strafing Left");
        frontLeft->setSpeed(speed);
        frontRight->setSpeed(speed);
        backLeft->setSpeed(speed);
        backRight->setSpeed(speed);
        frontLeft->run(BACKWARD);
        frontRight->run(FORWARD);
        backLeft->run(FORWARD);
        backRight->run(BACKWARD);
    }

    void turnRight90() {
        Serial.println("Turning Right 90 Degrees");
        frontLeft->run(FORWARD);
        frontRight->run(RELEASE);
        backLeft->run(FORWARD);
        backRight->run(RELEASE);
    }

    void turnLeft90() {
        Serial.println("Turning Left 90 Degrees");
        frontLeft->run(RELEASE);
        frontRight->run(FORWARD);
        backLeft->run(RELEASE);
        backRight->run(FORWARD);
    }

    void performUTurn() {
        Serial.println("Performing U-Turn");
        frontLeft->run(FORWARD);
        frontRight->run(BACKWARD);
        backLeft->run(FORWARD);
        backRight->run(BACKWARD);
    }

    void stopAllMotors() {
        Serial.println("Stopping All Motors");
        frontLeft->run(RELEASE);
        frontRight->run(RELEASE);
        backLeft->run(RELEASE);
        backRight->run(RELEASE);
    }

private:
    Adafruit_MotorShield motorShield;
    Adafruit_DCMotor *frontRight, *frontLeft, *backRight, *backLeft;
    int speed;

    void runAllMotors(uint8_t direction) {
        frontLeft->setSpeed(speed);
        frontRight->setSpeed(speed);
        backLeft->setSpeed(speed);
        backRight->setSpeed(speed);
        frontLeft->run(direction);
        frontRight->run(direction);
        backLeft->run(direction);
        backRight->run(direction);
    }
};

#endif // MECANUM_ROBOT_H
