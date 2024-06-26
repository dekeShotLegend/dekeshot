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
    int x; // x-coordinate of the camera frame
    int y; // y-coordinate of the camera frame
    int width; // Width of the detected object
    int height; // Height of the detected object
};
Puck puck;

#define PUCK_SIGNATURE 7 // Change this if you calibrated the Pixy using another signature.
// Addition initializaitons 
float goal_x_far{225.0}, goal_y{60.0},goal_x_near; // We got this from Marvin Gao (ASBR guy)
bool goal_switch{false};
double targetToGoal{0.0}; // This is for when we seize the puck and we are trying to drive it towards goal.


/********************************************************CONTROLLER CLASS************************************************************/
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
        refreshRate = 50; 
    }

    // Public initializations
    const int cameraCenterX = 160;
    const int errorThreshold = 20;
    const int maxSpeed = 100;
    const int baseSpeed = 100;
    int turnFlag;
    double integral = 0.0;
    double previous_error = 0.0;
    size_t reactionDelay = 200; 
    size_t actionDelay = 10000; 
    size_t lastUpdateTime = 0;
    size_t lastActionTime = 0;
    bool actionInProgress = false;
    bool straightToGoal = false;

    float frontDis;

    // Goal angle 
    float getGoalAngle() {
    float horizontal = (robotData.posX - (goal_switch ? goal_x_far : goal_x_near));
    float vertical = (robotData.posY - goal_y);
    float angle = 180 + atan2(vertical, horizontal) * 180 / M_PI; //0 to 360
    return angle;
}
    // Distance to Goal
    float distanceToGoal() {
    // Sanity checks
    if (robotData.posX == NULL) {
        return -1; // posX is not set or invalid
    }
    if (robotData.posY == NULL) {
        return -2; // posY is not set or invalid
    }
    // Euclidean Norm
    float goal_x = goal_switch ? goal_x_far : goal_x_near;
    targetToGoal = sqrt(pow(goal_x - robotData.posX, 2) + pow(goal_y - robotData.posY, 2));
    return targetToGoal;
}
  float robotX() {
    if (robotData.posX == NULL) {
        return -1; // posX is not set or invalid
    }
    return robotData.posX;
  }

  float robotY() {
    if (robotData.posY == NULL) {
        return -1; // posY is not set or invalid
    }
    return robotData.posY;
  }
   float initialYawOffset; 
}


    // Adjusts the robot's speed for safety purposes
    void adjustSpeedForSafety() {
        robot->setAllMotorSpeeds(25); // Reduce speed in potentially risky areas
    }

    // Executes a turn by a specified number of degrees using PID control
void turn(double degrees, double kp, double ki, double kd) {
    double targetYaw = degrees;
    double currentYaw = readHeading();
    double output, error;

    do {
        currentYaw = readHeading();
        error = targetYaw - currentYaw;

        // It is important to normalize the error here folks from -180 to 180 as per ASBR. 
        if (error > 180) {
            error -= 360; 
        } else if (error < -180) {
            error += 360;
        }

        // Calculate the PID output based on the error
        output = calculatePID(error, kp, ki, kd);
        robot->setMotorSpeeds(output, -output); // Apply the output to motor speeds

        Serial.print("Target Yaw: ");
        Serial.println(targetYaw);
        Serial.print("Current Yaw: ");
        Serial.println(currentYaw);
        delay(10); 

    } while (!isTurnComplete(targetYaw, currentYaw)); // Continue until the turn is complete
}

//left side: true, right side: false. Inspired by the video shown to us by TAs.
void turnOneSide(double degrees, double kp, double ki, double kd, bool side) {
    double targetYaw = degrees;
    double currentYaw = readHeading();
    double output, error;

    do {
        currentYaw = readHeading();
        error = targetYaw - currentYaw;

        // It is important to normalize the error here folks from -180 to 180 as per ASBR. 
        if (error > 180) {
            error -= 360; 
        } else if (error < -180) {
            error += 360;
        }

        // Calculate the PID output based on the error
        output = calculatePID(error, kp, ki, kd);
        robot->setMotorSpeeds(output*side, output*!side); // Apply the output to motor speeds

        Serial.print("Target Yaw: ");
        Serial.println(targetYaw);
        Serial.print("Current Yaw: ");
        Serial.println(currentYaw);
        delay(10); 

    } while (!isTurnComplete(targetYaw, currentYaw)); // Continue until the turn is complete
}

// This function checks if the turn has been completed within a specified tolerance
bool isTurnComplete(double target, double current, double tolerance = 5.0) {

    double diff = fabs(fmod(fabs(target - current), 360.0));
    if (diff > 180) diff = 360 - diff;
    return diff <= tolerance;
}

// Calculate PID logic
double calculatePID(double error, double kp, double ki, double kd) {
    static double integral = 0.0, previous_error = 0.0;
    integral += error; // Integrate the error
    double derivative = error - previous_error; 
    previous_error = error; 

    // Return the PID output
    return kp * error + ki * integral + kd * derivative;
}


    void init() {
        if (!bno->begin()) {
            Serial.println("No BNO055 detected. Check your wiring.");
            while (1); 
        }
        bno->setExtCrystalUse(true);
        robot->initializeMotors();
        robot->setAllMotorSpeeds(0); 
    }

    void run() {
        updatePING();
        updatePixy();
        processMovements();
    }

    // Normalizes any angle to the range of 0 to 360 degrees
    float normalizeAngle(float angle) {
        while (angle >= 360) angle -= 360;
        while (angle < 0) angle += 360;
        return angle;
    }

    float normalizeAnglePI(float angle) {
      if (angle > 180) angle = angle - 360;
      return angle;
    }

    // Reads the current heading in degrees from the BNO055
    double readHeading() {
        sensors_event_t event;
        bno->getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
        float heading = event.orientation.x; // Reading heading in degrees from 0 to 360
        return (360 - heading); //coordinate plane is flipped between ZigBee field and IMU, I'm picking to flip this, robot must face pink goal on initialization
    }
    void updatePING() {
        pinMode(PIN_PING, OUTPUT);
        digitalWrite(PIN_PING, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_PING, HIGH);
        delayMicroseconds(5);
        digitalWrite(PIN_PING, LOW);
        pinMode(PIN_PING, INPUT);
        frontDis = (pulseIn(PIN_PING, HIGH) / 2) * 0.0343; 
    }

private:
    Pixy2* pixy;
    MecanumRobot* robot;
    Adafruit_BNO055* bno;
    int PIN_PING;
    double kp_turn, ki_turn, kd_turn, kp_forward, ki_forward, kd_forward;
    unsigned int refreshRate;

    void updatePixy() {
        int blockCount = pixy->ccc.getBlocks();
        if (blockCount > 0) {
            int index = -1;
            for (int i = 0; i < blockCount; ++i) {
                if (pixy->ccc.blocks[i].m_signature == PUCK_SIGNATURE) {
                    if (index == -1 || pixy->ccc.blocks[i].m_width * pixy->ccc.blocks[i].m_height > puck.width * puck.height) {
                        index = i;
                    }
                }
            }
            if (index != -1) {
                robot->setChaseStatus(true);
                puck = {pixy->ccc.blocks[index].m_x, pixy->ccc.blocks[index].m_y, pixy->ccc.blocks[index].m_width, pixy->ccc.blocks[index].m_height};
                driveTowardsPuck();
            }
        } else {
            robot->searchForPuck();
        }
    }

    void driveTowardsPuck() {
    int errorX = puck.x - cameraCenterX;
    int proportionalSpeedAdjustment = static_cast<int>(kp_turn * errorX / 5.0);
    Serial.print(proportionalSpeedAdjustment);
    Serial.println(" Proportional Speed Adjustment.");

    // Check time to update without blocking
    if (millis() - lastUpdateTime > reactionDelay) {
        if (abs(errorX) < errorThreshold) {
            robot->setMotorSpeeds(0, 0);
            int dynamicSpeed = min(maxSpeed, static_cast<int>(frontDis * 2 + 15)); 
            robot->setAllMotorSpeeds(dynamicSpeed);
        } else {
            // Adjust motor speeds based on the horizontal error to smoothly align with the puck
            int leftMotorSpeed = baseSpeed + proportionalSpeedAdjustment;
            int rightMotorSpeed = baseSpeed - proportionalSpeedAdjustment;
            robot->setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
        }
    }
}


    void processMovements() {
        // Check distance to puck and adjust behavior
        if (frontDis > 2 && frontDis < 10) {
            if (!actionInProgress) {
                Serial.println("Acquired Pixy ..... Positioning.");
                robot->setAllMotorSpeeds(0);
                actionInProgress = true;
                lastActionTime = millis();
            }
            
            // Continue after a delay without blocking
            if (millis() - lastActionTime > actionDelay && actionInProgress) {
                actionInProgress = false; // Reset the action flag
                if (robot->trackingPuck) {
                    robot->setChaseStatus(false);
                    float angleDifference = getGoalAngle() - readHeading();
                    turn(angleDifference, kp_turn, ki_turn, kd_turn);
                    straightToGoal = true;
                } else {
                    robot->setMotorSpeeds(-100, -100); // Reverse or perform other action
                }
            }
        } else if (frontDis > 30) {
            if (robot->trackingPuck) {
                Serial.println("Puck too far, stopping chase.");
                robot->setChaseStatus(false);
            }
        }
    }
};

#endif // COMMUNICATION_CONTROLLER_H
