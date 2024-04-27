// Check if the header file is already included
#ifndef MECANUM_ROBOT_H
#define MECANUM_ROBOT_H

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Arduino.h>

class MecanumRobot {
public:
    MecanumRobot() : motorShield(Adafruit_MotorShield()), speed(50), rotateSpeed(30) {  // Initialize it here
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
    void MecanumRobot::setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Set the speed and direction for the left motors
    frontLeft->setSpeed(abs(leftSpeed));
    backLeft->setSpeed(abs(leftSpeed));
    frontLeft->run((leftSpeed >= 0) ? FORWARD : BACKWARD);
    backLeft->run((leftSpeed >= 0) ? FORWARD : BACKWARD);

    // Set the speed and direction for the right motors
    frontRight->setSpeed(abs(rightSpeed));
    backRight->setSpeed(abs(rightSpeed));
    frontRight->run((rightSpeed >= 0) ? FORWARD : BACKWARD);
    backRight->run((rightSpeed >= 0) ? FORWARD : BACKWARD);
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
    void MecanumRobot::searchForPuck() {
    static size_t lastUpdate{0}, currentTime{millis()};
    static float angle{0.0}; // Continuous angle increment

    if (currentTime - lastUpdate > 100) {  // Update every 100 ms for smoother operation
        lastUpdate = currentTime;
        angle += 0.05;  // Increase the angle to create spiral motion

        // Calculate dynamic speeds based on the angle
        int leftSpeed = speed + sin(angle) * speed; // Modulate left speed with sine wave
        int rightSpeed = speed + cos(angle) * speed; // Modulate right speed with cosine wave

        // Set speeds for left and right sides
        setMotorSpeeds(leftSpeed, rightSpeed);
    }
}

private:
    Adafruit_MotorShield motorShield;
    Adafruit_DCMotor *frontRight, *frontLeft, *backRight, *backLeft;
    int speed, rotateSpeed;

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
