// Seyi R. Afolayan
#ifndef MECANUM_ROBOT_H
#define MECANUM_ROBOT_H

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Arduino.h>

class MecanumRobot {
public:
    static constexpr int DEFAULT_SPEED{50}, DEFAULT_ROTATE_SPEED{0};

    MecanumRobot() : speed(DEFAULT_SPEED), rotateSpeed(DEFAULT_ROTATE_SPEED) {}

    void begin() {
        motorShield.begin(); 
        initializeMotors();
    }

    void initializeMotors() {
        frontLeft = motorShield.getMotor(1); // Motor 1 is the FRONT left motor
        frontRight = motorShield.getMotor(2); // Motor 2 is the FRONT right motor
        backLeft = motorShield.getMotor(3); // Motor 3 is the  REAR left motor
        backRight = motorShield.getMotor(4); // Motor 4 is the REAR right motor 
    }

    void setAllMotorSpeeds(int newSpeed) {
        speed = newSpeed;
        frontLeft->setSpeed(speed);
        frontRight->setSpeed(speed);
        backLeft->setSpeed(speed);
        backRight->setSpeed(speed);
    }

    void setMotorSpeeds(int leftSpeed, int rightSpeed) {
        frontLeft->setSpeed(abs(leftSpeed));
        backLeft->setSpeed(abs(leftSpeed));
        frontLeft->run((leftSpeed >= 0) ? FORWARD : BACKWARD);
        backLeft->run((leftSpeed >= 0) ? FORWARD : BACKWARD);

        frontRight->setSpeed(abs(rightSpeed));
        backRight->setSpeed(abs(rightSpeed));
        frontRight->run((rightSpeed >= 0) ? FORWARD : BACKWARD);
        backRight->run((rightSpeed >= 0) ? FORWARD : BACKWARD);
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

    void searchForPuck() {
        static size_t lastUpdate = 0;
        size_t currentTime = millis();
        static float angle = 0.0; // Continuous angle increment

        if (currentTime - lastUpdate > 100) {  // Update every 100 ms for smoother operation
            lastUpdate = currentTime;
            angle += 0.05;  // Increase the angle to create spiral motion

            int leftSpeed = speed + sin(angle) * speed; // Modulate left speed with sine wave
            int rightSpeed = speed + cos(angle) * speed; // Modulate right speed with cosine wave

            setMotorSpeeds(leftSpeed, rightSpeed);
        }
    }

private:
    Adafruit_MotorShield motorShield;
    Adafruit_DCMotor *frontRight, *frontLeft, *backRight, *backLeft;
    int speed, rotateSpeed;

    void runAllMotors(uint8_t directionFR, uint8_t directionFL, uint8_t directionBR, uint8_t directionBL) {
        frontLeft->setSpeed(speed);
        frontRight->setSpeed(speed);
        backLeft->setSpeed(speed);
        backRight->setSpeed(speed);
        frontLeft->run(directionFL);
        frontRight->run(directionFR);
        backLeft->run(directionBL);
        backRight->run(directionBR);
    }
};

#endif // MECANUM_ROBOT_H
