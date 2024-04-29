// Include necessary library headers
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Pixy2.h>
#include "Controller.h"

// Initialize sensor and controller objects
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Pixy2 pixy; 
MecanumRobot robot;  // Proper instantiation of MecanumRobot

const int PIN_PING{13}; // Ping sensor
const int PIN_RELAY{20}; // Ping sensor  

// Instantiate the Controller and XbeeCommunicator
Controller controller(&pixy, PIN_PING, &bno, &robot);  // Pass the robot object to the controller
XbeeCommunicator xbeeComms(Serial1);  // Assuming Serial1 is the correct serial port for XBee

void setupIMU(){
    while (!Serial) {
        delay(10);  // Wait for the serial port to open
    }
    Serial.println("Orientation Sensor Test");
    Serial.println("");

    // Initialize the BNO055 sensor
    if (!bno.begin()) {
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1); // Endless loop on failure
    }
}

void setup() {
    pinMode(PIN_RELAY,OUTPUT);
    digitalWrite(PIN_RELAY,LOW);
    Serial.begin(115200);
    setupIMU();
    pixy.init();
    robot.begin(); // initialize the robot motors
    controller.init();
    xbeeComms.setupXbeeComm();  // Setup XBee communication
    xbeeComms.handleXbeeComm(); // single check, don't run code until game starts
    while(robotData.matchStatus != "1") {
      xbeeComms.handleXbeeComm();
    }
    Serial.println(robotData.matchStatus);
}

void loop() {
    float currentHeading = controller.readHeading();
    controller.run();
    Serial.println("I get here");
    xbeeComms.handleXbeeComm();  // Handle XBee communication in the main loop
    //Chandler: this is just charging forward once the puck is captured
    while (controller.straightToGoal && controller.distanceToGoal() > 80) { //experimentally 10 is too close
      xbeeComms.handleXbeeComm();
    }
    if (controller.straightToGoal) {
    robot.setAllMotorSpeeds(0);
    digitalWrite(PIN_RELAY,HIGH);
    delay(500);
    digitalWrite(PIN_RELAY,LOW);
    controller.straightToGoal = false;
    }
    // I will implement this later
    // navigateToPosition(robotData.posX, robotData.posY);

    // Other Codes 
}

