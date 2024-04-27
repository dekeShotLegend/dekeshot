/**
* Developer: Seyi R. Afolayan
* Work: Controller Class with PID control and Mecanum wheel integration
**/

#include <Arduino.h>
#include <Pixy2.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "MecanumRobot.h"  // Include the MecanumRobot class header

class Controller {
public:
    Controller(Pixy2* pixy, int pinPing, Adafruit_BNO055* bno, MecanumRobot* robot)
        : pixy(pixy), PIN_PING(pinPing), bno(bno), robot(robot) {}  // Ensure robot is passed as a pointer

    void init() {
        Serial.begin(9600);
        if (!bno->begin()) {
            Serial.println("No BNO055 detected. Check your wiring.");
            while (1);  // Hang if sensor is not detected
        }
        bno->setExtCrystalUse(true);

        robot->initializeMotors(); // Use the robot instance passed to the controller
        robot->setAllMotorSpeeds(50);  // Default speed for all motors

        rollOffset = readInitialRollOffset();
        Serial.print("rollOffset: ");
        Serial.println(rollOffset);
        Serial.print("roll: ");
        Serial.println(roll);
    }

    void run() {
        updatePING();
        updatePixy();
        if (frontDis <= 30) {
            if (frontDis <= 11) {
                Serial.println("Obstacle too close, stopping and turning.");
                robot->stopAllMotors();
                decideTurnDirection();
            } else {
                Serial.println("Obstacle detected, reducing speed.");
                robot->setAllMotorSpeeds(25);
            }
        } else {
            Serial.println("Path clear, moving forward.");
            robot->moveForward();
        }
    }

private:
    Pixy2* pixy;
    Adafruit_BNO055* bno;
    MecanumRobot* robot;  // Pointer to the MecanumRobot instance
    int PIN_PING;
    float frontDis = 0.0;
    double roll, rollOffset = 400;

    void updatePixy() {
        pixy->ccc.getBlocks();
        if (pixy->ccc.numBlocks > 0) {
            Serial.println("Detected object with Pixy2.");
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
        frontDis = (pulseDuration / 2) * 0.0343;  // cm per microsecond
        Serial.print("Front distance: ");
        Serial.println(frontDis);
    }

    double readInitialRollOffset() {
        double initialRoll;
        readRoll(initialRoll);
        while (initialRoll >= 360) {
            readRoll(initialRoll);
        }
        return initialRoll;
    }

    void readRoll(double& val) {
        sensors_event_t event;
        bno->getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
        val = event.orientation.x;
        if (val >= 180) val -= 360;
    }

    void decideTurnDirection() {
        // Placeholder for decision logic to turn based on sensor inputs
        robot->turnRight90(); // Example function to turn right
    }
};
