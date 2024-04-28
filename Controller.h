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

#define PUCK_SIGNATURE 7 // PLEASE change this if you calibrated the pixy using another signature.

/********************************************************CONTROLLER CLASS************************************************************/
// The legendary Robot controller class (some of the codes are from lab4 so maybe correct the readroll stuff or what not)
class Controller {
public:
    Controller(Pixy2 *pixy, int pinPing, Adafruit_BNO055 *bno, MecanumRobot *robot)
        : pixy(pixy), PIN_PING(pinPing), robot(robot), bno(bno) {
        // Initialize PID constants
        kp_turn = 1.15;
        ki_turn = 0.07;
        kd_turn = 0.05;
        kp_forward = 0.3;
        ki_forward = 0.1;
        kd_forward = 0.01;
        refreshRate = 50; // PID refresh rate in milliseconds
    }
    float goal_x{25.0}, goal_y{60.0};
    bool straightToGoal{false};
    void init() {
        if (!bno->begin()) {
            Serial.println("No BNO055 detected. Check your wiring.");
            while (1); // Hang if sensor is not detected
        }
        bno->setExtCrystalUse(true);
        robot->initializeMotors();
        robot->setAllMotorSpeeds(0); // Default speed for all motors
        readInitialRollOffset();
    }
    float distanceToGoal () {
      if(robotData.posX == NULL) {
        return -1;
      }
      if(robotData.posY == NULL) {
        return -2;
      }
      return sqrt(pow(goal_x - robotData.posX,2) + pow(goal_y - robotData.posY,2));
    }
    void Controller::run(float heading) {
        updatePING();
        updatePixy();
        processMovements(heading);
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
                if (index == -1 || pixy->ccc.blocks[i].m_width * pixy->ccc.blocks[i].m_height > 20) {
                    index = i;
                }
            }
        }
        if (index != -1) {
            robot->setChaseStatus(true);
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
        if(!(robot->trackingPuck)){
        robot->searchForPuck();  // Implement this method in MecanumRobot to handle searching
        }
    }
}

    void driveTowardsPuck() {
    int cameraCenterX = 160; // Pixy stuff is 319 so I am using 320 here
    int errorX = puck.x - cameraCenterX;  // Calculate error from the center
    int proportionalSpeedAdjustment = kp_turn * errorX / 5.0;  // Calculate proportional speed adjustment
    Serial.print(proportionalSpeedAdjustment); Serial.println(" Proportional Speed Adjustment.");


    // Dynamically adjust speed as the puck approaches the center
    if (abs(errorX) < 20) {  // '10' can be adjusted based on what is considered 'centered'
        robot->setMotorSpeeds(0, 0);  // Stop the robot or adjust to a very slow movement
        delay(100);
        robot->setAllMotorSpeeds((frontDis * 2 + 15> 100 ? 100 : frontDis * 2 + 15)); // Chandler: This has to either be continuous or slow down quicker
        delay(300);
        robot->setMotorSpeeds(0, 0);  // Stop the robot or adjust to a very slow movement
    } else {
          // Adjust motor speeds based on the horizontal error to smoothly align with the puck
    int baseSpeed = 100;  // Base speed can be dynamically adjusted based on other factors if needed
    int leftMotorSpeed = baseSpeed + proportionalSpeedAdjustment;
    int rightMotorSpeed = baseSpeed - proportionalSpeedAdjustment;
    robot->setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
    delay(200);
    }
}

  float getGoalAngle(){
    float horizontal = (robotData.posX - goal_x);
    float vertical = (robotData.posY - goal_y);
    float angle = atan2(vertical,horizontal)*180/(M_PI); //Chandler: This is probably wrong; IMU goes from -180 to 180, I thought 0 to 360 at firs
    //Chandler: use ReadRoll function here
    return angle;
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

    void processMovements(float heading) {
        if (frontDis > 2 && frontDis < 7) {
            Serial.println("Acquired Pixy .....");
            robot->setAllMotorSpeeds(0);
            delay(10000);
            /*if (robot->trackingPuck) {
              robot->setChaseStatus(false);
              turn(getGoalAngle() - heading,kp_turn,ki_turn,kd_turn); //Chandler: This is where I tried to implement a turn
              straightToGoal = true;
            }
            else {
              robot->setMotorSpeeds(-100, -100);
              delay(300);
            }*/
        } else if (frontDis > 30) {
           robot->setChaseStatus(false);
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
        target = (target < -180 ? target + 360 : target); //Chandler: yeah I messed up the turns
        double currentRoll;
        do {
            Serial.print("Target: ");
            Serial.println(target);
            Serial.print("currentRoll: ");
            Serial.println(currentRoll);
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
        //if (val >= 180) val -= 360; Chandler: if maping from -pi to pi, this isn't necessary
    }
};

#endif // COMMUNICATION_CONTROLLER_H

