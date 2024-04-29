#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Pixy2.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "XbeeCommunicator.h"

// Initialize sensor and controller objects

Pixy2 pixy; 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_MotorShield DekeShotMotors = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor;
Adafruit_DCMotor *rightMotor;

const int PIN_PING{35}; // Ping sensor

int index;

bool goal = true;

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
    Serial.begin(115200);
    setupIMU();
    initializeMotors();
    pixy.init();
    xbeeComms.setupXbeeComm();  // Setup XBee communication
    xbeeComms.handleXbeeComm(); // single check
    while(robotData.matchStatus != "1") {
      xbeeComms.handleXbeeComm();
    }
    Serial.println(robotData.matchStatus);
}

void loop() {
    int logic = validSignature();
    if (logic <= -1) {
      Serial.println("I get here too");
      setAllMotorSpeeds(90);
      leftMotor->run(FORWARD);
      rightMotor->run(BACKWARD);
      delay(250);
      setAllMotorSpeeds(0);
      delay(50);
    }
    else {
      chaseForward();
    }
}

void initializeMotors() {
  DekeShotMotors.begin();
  leftMotor = DekeShotMotors.getMotor(4);
  rightMotor = DekeShotMotors.getMotor(2);
}

void setAllMotorSpeeds(int speed) {
  leftMotor->setSpeed(speed);   
  rightMotor->setSpeed(speed); 
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftMotor->setSpeed(leftSpeed);   
  rightMotor->setSpeed(rightSpeed);   
}

void chaseForward() {
  Serial.println("chaseForward");
  int searchLogic = validSignature();
  bool height100 = false;
  int newestHeight = 25;
  while (searchLogic >= 0 && !((newestHeight < 30) && pixy.ccc.blocks[index].m_y > 150)) {
    newestHeight = pixy.ccc.blocks[index].m_height;
    if (newestHeight >= 100) {
      height100 = true;
    }
    int errorX = pixy.ccc.blocks[index].m_x - 160;
    int proportionalSpeedAdjustment (0.3 * errorX);
    int baseSpeed;
    if (!height100) {
      baseSpeed = 100;
    }
    else if (newestHeight >= 100){
      baseSpeed = 30;
    }
    else {
      baseSpeed = 40 - (100 - newestHeight)/4;
    }
    setMotorSpeeds(100 + proportionalSpeedAdjustment, 100 - proportionalSpeedAdjustment);
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    searchLogic = validSignature();  
  }
  swivelTurnGoal(goal);
  moveForwardTowardGoal(goal);
}

float updatePING() {
    pinMode(PIN_PING, OUTPUT);
    digitalWrite(PIN_PING, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_PING, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_PING, LOW);
    pinMode(PIN_PING, INPUT);
    return (pulseIn(PIN_PING, HIGH) / 2) * 0.0343; 
}

int validSignature() {
    Serial.println("validSignature");
    int blockCount = pixy.ccc.getBlocks();
    if (blockCount > 0) {
        index = 9999;
        for (int i = 0; i < blockCount; ++i) {
            if (pixy.ccc.blocks[i].m_signature == 7) { // puck is at 7
                if (index == -1 || pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height > 10) {
                    if (i < index) {
                      index = i;
                    }
                }
            }
        }
    }
    else {
      return -1; //no blocks found
    }
    if (index == 9999) {
      return -2; //no puck blocks found
    }
    return index;
}


//true = red goal, false = pink goal
void swivelTurnGoal(bool goal) {
  xbeeComms.handleXbeeComm();
  float goal_y = 60.0;
  float goal_x = (goal ? 25.0 : 225.0);
  setAllMotorSpeeds(0);
  bool turnWay = (robotData.posY > 60 ? true : false);
  //turnWay = (goal ? turnWay : !turnWay);
  wideTurnDumb(getGoalAngle(goal_x,goal_y),turnWay);
}

// This function checks if the turn has been completed within a specified tolerance
bool isTurnComplete(double target, double current, double tolerance = 5.0) {

    double diff = fabs(fmod(fabs(target - current), 360.0));
    if (diff > 180) diff = 360 - diff;
    return diff <= tolerance;
}

void wideTurn(double degrees, double kp, double ki, double kd, bool further_side) {
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
        setMotorSpeeds(output*further_side, output*(!further_side)); // Apply the output to motor speeds

        Serial.print("Target Yaw: ");
        Serial.println(targetYaw);
        Serial.print("Current Yaw: ");
        Serial.println(currentYaw);
        delay(10); 

    } while (!isTurnComplete(targetYaw, currentYaw)); // Continue until the turn is complete
}

void wideTurnDumb(double degrees, bool further_side) {
  while (abs(degrees - readHeading()) > 3) {
    setMotorSpeeds((further_side ? 75 : 15), (further_side ? 15 : 75));
  }
  setAllMotorSpeeds(75);
}


float getGoalAngle(float goal_x, float goal_y) {
    float horizontal = (robotData.posX - goal_x);
    float vertical = (robotData.posY - goal_y);
    float angle = 180 + atan2(vertical, horizontal) * 180 / M_PI; //0 to 360
    return angle;
}

double readHeading() {
  sensors_event_t event; 
  bno.getEvent(&event);
  return event.orientation.x;
}

double calculatePID(double error, double kp, double ki, double kd) {
    static double integral = 0.0, previous_error = 0.0;
    integral += error; // Integrate the error
    double derivative = error - previous_error; 
    previous_error = error; 

    // Return the PID output
    return kp * error + ki * integral + kd * derivative;
}

void moveForwardTowardGoal(bool goal) {
  float goal_y = 60.0;
  float goal_x = (goal ? 25.0 : 225.0);
  xbeeComms.handleXbeeComm();
  bool one_reallignment = false;
  while (distanceToGoal(goal_x, goal_y) > 80) {
    setAllMotorSpeeds(100);
    xbeeComms.handleXbeeComm();
    if(distanceToGoal(goal_x,goal_y) < 150 && !one_reallignment) {
      wideTurnDumb(getGoalAngle(goal_x,goal_y),goal);
      one_reallignment = true;
    }
  }
  setAllMotorSpeeds(255);
  delay(500);
  Serial.println("STRIKE!");
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
  delay(1000);
  setAllMotorSpeeds(0);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}


float distanceToGoal(float goal_x, float goal_y) {
    // Sanity checks
if (robotData.posX == NULL) {
    return -1; // posX is not set or invalid
}
if (robotData.posY == NULL) {
    return -2; // posY is not set or invalid
}
return sqrt(pow(goal_x - robotData.posX, 2) + pow(goal_y - robotData.posY, 2));
}
