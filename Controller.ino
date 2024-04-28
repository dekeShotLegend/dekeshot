// Include necessary library headers
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Pixy2.h>
#include "Controller.h"
#include "MecanumRobot.h"

// Initialize sensor and controller objects
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Pixy2 pixy; 
MecanumRobot robot;  // Proper instantiation of MecanumRobot

const int PIN_PING{49}; // Ping sensor 
const int PIN_RELAY{44};

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
    digitalWrite(PIN_RELAY, LOW);
    Serial.begin(115200);
    setupIMU();
    pixy.init();
    xbeeComms.setupXbeeComm();  // Setup XBee communication
    robot.begin(); // initialize the robot motors
    controller.init();
}

void loop() {
    sensors_event_t event; 
    bno.getEvent(&event);
    controller.run(event.orientation.x);
    xbeeComms.handleXbeeComm();  // Handle XBee communication in the main loop
    if (robotData.matchStatus == "START") {
      //robot.setAllMotorSpeeds(50);
    } else if (robotData.matchStatus == "STOP") {
      //robot.stopAllMotors();
    }

    // I will implement this later
    // navigateToPosition(robotData.posX, robotData.posY);

    // Other Codes 
}
