/**
* Developer: Seyi R. Afolayan
* Work: Controller Class 
**/
/********************************************************BEGIN CODE******************************************************************/

#ifndef COMMUNICATION_CONTROLLER_H
#define COMMUNICATION_CONTROLLER_H
#include <Arduino.h>
#include <Pixy2.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "MecanumRobot.h"

// Puck tracking structure
struct Puck {
    int x; // x_cord of the cam frame
    int y; // y_cord of the cam frame
    int width; // Width of the detected object
    int height; // Height of the detected object
};
Puck puck;

#define PUCK_SIGNATURE 1 // PLEASE change this if you calibrated the pixy using another signature.

/********************************************************CONTROLLER CLASS************************************************************/
// The legendary Robot controller class (some of the codes are from lab4 so maybe correct the readroll stuff or what not)
class Controller {
public:
    Controller(Pixy2 *pixy, int pinPing, Adafruit_BNO055 *bno, MecanumRobot *robot)
        : pixy(pixy), PIN_PING(pinPing), robot(robot), bno(bno) {
        // Initialize PID constants
        kp_turn = 2.0;
        ki_turn = 0.1;
        kd_turn = 0.05;
        kp_forward = 0.4;
        ki_forward = 0.1;
        kd_forward = 0.01;
        refreshRate = 50; // PID refresh rate in milliseconds
    }

    void init() {
        Serial.begin(115200);
        if (!bno->begin()) {
            Serial.println("No BNO055 detected. Check your wiring.");
            while (1); // Hang if sensor is not detected
        }
        bno->setExtCrystalUse(true);
        robot->initializeMotors();
        robot->setAllMotorSpeeds(50); // Default speed for all motors
        readInitialRollOffset();
    }

    void Controller::run() {
        updatePING();
        updatePixy();
        processMovements();
    }

private:
    Pixy2* pixy;
    MecanumRobot* robot;
    Adafruit_BNO055* bno;
    int PIN_PING;
    double kp_turn, ki_turn, kd_turn, kp_forward, ki_forward, kd_forward;
    double rollOffset, roll;
    float frontDis;
    int turnFlag;
    unsigned int refreshRate;
    volatile double integral{0}, previous_error{0};

    void updatePixy() {
    int blockCount = pixy->ccc.getBlocks();
    if (blockCount > 0) {
        // Process all blocks to find the one with PUCK_SIGNATURE
        int index = -1;
        for (int i = 0; i < blockCount; ++i) {
            if (pixy->ccc.blocks[i].m_signature == PUCK_SIGNATURE) {
                if (index == -1 || pixy->ccc.blocks[i].m_width * pixy->ccc.blocks[i].m_height > puck.width * puck.height) {
                    index = i;
                }
            }
        }
        if (index != -1) {
            puck.x = pixy->ccc.blocks[index].m_x;
            puck.y = pixy->ccc.blocks[index].m_y;
            puck.width = pixy->ccc.blocks[index].m_width;
            puck.height = pixy->ccc.blocks[index].m_height;
            Serial.print("Puck detected at: ");
            Serial.print(puck.x);
            Serial.print(", ");
            Serial.println(puck.y);
            driveTowardsPuck();
        }
    } else {
        Serial.println("No puck detected");
        robot->searchForPuck();  // Implement this method in MecanumRobot to handle searching
    }
}

    void driveTowardsPuck() {
    int cameraCenterX = 160; // Pixy stuff is 319 so I am using 320 here
    int errorX = puck.x - cameraCenterX;  // Calculate error from the center
    int proportionalSpeedAdjustment = kp_turn * errorX;  // Calculate proportional speed adjustment

    // Adjust motor speeds based on the horizontal error to smoothly align with the puck
    int baseSpeed = 50;  // Base speed can be dynamically adjusted based on other factors if needed
    int leftMotorSpeed = baseSpeed - proportionalSpeedAdjustment;
    int rightMotorSpeed = baseSpeed + proportionalSpeedAdjustment;
    robot->setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);

    // Dynamically adjust speed as the puck approaches the center
    if (abs(errorX) < 10) {  // '10' can be adjusted based on what is considered 'centered'
        robot->setMotorSpeeds(0, 0);  // Stop the robot or adjust to a very slow movement
        robot->moveForward();  // This could be modified to only trigger under certain conditions
    }
}


    void updatePING() {
        pinMode(PIN_PING, OUTPUT);
        digitalWrite(PIN_PING, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_PING, HIGH);
        delayMicroseconds(5);
        digitalWrite(PIN_PING, LOW);
        pinMode(PIN_PING, INPUT);
        unsigned long pulseDuration = pulseIn(PIN_PING, HIGH);
        frontDis = (pulseDuration / 2) * 0.0343; // Speed of sound in cm/us
        Serial.print("Front distance: ");
        Serial.println(frontDis);
    }

    void processMovements() {
        if (frontDis <= 11) {
            robot->stopAllMotors();
            if (turnFlag != 0) {
                performTurnBasedOnFlag();
            } else {
                performEmergencyManeuver();
            }
        } else if (frontDis > 11 && frontDis <= 30) {
            adjustSpeedForSafety();
        } else {
            forward(80, kp_forward, ki_forward, kd_forward);
        }
    }

    void performTurnBasedOnFlag() {
        switch (turnFlag) {
            case 1: turn(-90, kp_turn, ki_turn, kd_turn); break;
            case 2: turn(90, kp_turn, ki_turn, kd_turn); break;
            default: performEmergencyManeuver(); break;
        }
    }

    void performEmergencyManeuver() {
        // Custom logic for emergency maneuvers
        turn(180, kp_turn, ki_turn, kd_turn); // Example: 180-degree turn
    }

    void adjustSpeedForSafety() {
        robot->setAllMotorSpeeds(25); // Reduce speed in potentially risky areas
    }

    void forward(int speed, double kp, double ki, double kd) {
        readRoll(roll);
        double output = calculatePID(roll - rollOffset, kp, ki, kd);
        robot->setMotorSpeeds(speed + output, speed - output);
    }

    void turn(double degrees, double kp, double ki, double kd) {
        double target = rollOffset + degrees;
        double currentRoll;
        do {
            readRoll(currentRoll);
            double output = calculatePID(target - currentRoll, kp, ki, kd);
            robot->setMotorSpeeds(output, -output);
        } while (!isTurnComplete(target, currentRoll));
        rollOffset = target;
    }

    bool isTurnComplete(double target, double current) {
        return abs(target - current) < 5; // 5 degrees tolerance
    }

    double calculatePID(double error, double kp, double ki, double kd) {
        integral += error;
        double derivative = error - previous_error;
        previous_error = error;
        return kp * error + ki * integral + kd * derivative;
    }

    void readInitialRollOffset() {
        readRoll(rollOffset);
        while (rollOffset >= 360) readRoll(rollOffset);
    }

    void readRoll(double& val) {
        sensors_event_t event;
        bno->getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
        val = event.orientation.x;
        if (val >= 180) val -= 360;
    }
};

#endif // COMMUNICATION_CONTROLLER_H

