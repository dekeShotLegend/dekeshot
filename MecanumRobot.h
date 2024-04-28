// Seyi R. Afolayan
#ifndef MECANUM_ROBOT_H
#define MECANUM_ROBOT_H

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Arduino.h>
#include "XbeeCommunicator.h"

class MecanumRobot {
public:
    static constexpr int DEFAULT_SPEED = 0;  // Default speed for all motors
    bool trackingPuck = false;

    MecanumRobot() : speed(DEFAULT_SPEED) {
        motorShield = Adafruit_MotorShield(); // Ensure the motor shield is initialized here
    }

    void begin() {
        if (!motorShield.begin()) { // Check if the motor shield successfully initializes
            Serial.println("Failed to start the motor shield");
            return;
        }
        initializeMotors();
    }

    void initializeMotors() {
        // Initialize each motor and check if they are correctly set
        frontLeft = motorShield.getMotor(1); // Motor 1 is the FRONT left motor
        frontRight = motorShield.getMotor(2); // Motor 2 is the FRONT right motor
        backLeft = motorShield.getMotor(3); // Motor 3 is the REAR left motor
        backRight = motorShield.getMotor(4); // Motor 4 is the REAR right motor 
    }

    void setAllMotorSpeeds(int newSpeed) {
        speed = newSpeed;
        updateMotorSpeeds();
    }

    void setChaseStatus(bool status) {
        trackingPuck = status;
    }

    void setMotorSpeeds(int leftSpeed, int rightSpeed) {
        setSpeedAndDirection(frontLeft, leftSpeed);
        setSpeedAndDirection(backLeft, leftSpeed);
        setSpeedAndDirection(frontRight, rightSpeed);
        setSpeedAndDirection(backRight, rightSpeed);
    }

    void moveForward() {
        Serial.println("Moving Forward!");
        runAllMotors(FORWARD, FORWARD, FORWARD, FORWARD);
    }

    void moveBackward() {
        Serial.println("Reverse!");
        runAllMotors(BACKWARD, BACKWARD, BACKWARD, BACKWARD);
    }

    void strafeRight() {
        Serial.println("Strafing Right");
        runAllMotors(FORWARD, BACKWARD, BACKWARD, FORWARD);
    }

    void strafeLeft() {
        Serial.println("Strafing Left");
        runAllMotors(BACKWARD, FORWARD, FORWARD, BACKWARD);
    }

    void turnRight90() {
        Serial.println("Turning Right 90 Degrees");
        setSpeedAndDirection(frontLeft, speed, FORWARD);
        setSpeedAndDirection(frontRight, speed, RELEASE);
        setSpeedAndDirection(backLeft, speed, FORWARD);
        setSpeedAndDirection(backRight, speed, RELEASE);
    }

    void turnLeft90() {
        Serial.println("Turning Left 90 Degrees");
        setSpeedAndDirection(frontLeft, speed, RELEASE);
        setSpeedAndDirection(frontRight, speed, FORWARD);
        setSpeedAndDirection(backLeft, speed, RELEASE);
        setSpeedAndDirection(backRight, speed, FORWARD);
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
        runAllMotors(RELEASE, RELEASE, RELEASE, RELEASE);
    }

    void searchForPuck() {
        setMotorSpeeds(0, 0);
        delay(100);
        setMotorSpeeds(160, -160);
        delay(200);
        setMotorSpeeds(0, 0);
    }

private:
    Adafruit_MotorShield motorShield;
    Adafruit_DCMotor *frontRight, *frontLeft, *backRight, *backLeft;
    int speed;

    void updateMotorSpeeds() {
        frontLeft->setSpeed(speed);
        frontRight->setSpeed(speed);
        backLeft->setSpeed(speed);
        backRight->setSpeed(speed);
    }

    void setSpeedAndDirection(Adafruit_DCMotor *motor, int speed, uint8_t direction = FORWARD) {
        motor->setSpeed(abs(speed));
        motor->run(direction);
    }

    void runAllMotors(uint8_t directionFR, uint8_t directionFL, uint8_t directionBR, uint8_t directionBL) {
        setSpeedAndDirection(frontLeft, speed, directionFL);
        setSpeedAndDirection(frontRight, speed, directionFR);
        setSpeedAndDirection(backLeft, speed, directionBL);
        setSpeedAndDirection(backRight, speed, directionBR);
    }
};

#endif // MECANUM_ROBOT_H
