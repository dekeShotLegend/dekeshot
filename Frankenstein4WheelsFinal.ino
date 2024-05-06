#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Pixy2.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>`
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "XbeeCommunicator.h"

// Initialize sensor and controller objects

Pixy2 pixy; 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_MotorShield DekeShotMotors = Adafruit_MotorShield();
Adafruit_DCMotor *frontLeft;
Adafruit_DCMotor *frontRight;
Adafruit_DCMotor *backLeft;
Adafruit_DCMotor *backRight; 

const int PIN_PING{52},PIN_LEFT_PING{46},PIN_RIGHT_PING{48},PIN_RELAY{33}; // Ping sensor

int index;
int logic;
int lostCount = 0;
int lastAdjustment = 0;
bool sent = false;

bool goal = false; //true = score on red goal, false = pink goal
float goal_y = 60.0;
float goal_x = (goal ? 25.0 : 225.0);
float distance = 999;

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
enum RobotState {
  SEARCH_FOR_PUCK,
  CHASE_FORWARD_PIXY,
  CHASE_FORWARD_PING,
  TURN_TO_GOAL,
  MOVE_TOWARD_GOAL,
  STRIKE
};

RobotState currentState = SEARCH_FOR_PUCK;

void setup() {
    Serial.begin(115200);
    pinMode(PIN_PING,OUTPUT);
    pinMode(PIN_LEFT_PING,OUTPUT);
    pinMode(PIN_RIGHT_PING,OUTPUT);
    pinMode(PIN_RELAY,OUTPUT);
    digitalWrite(PIN_RELAY,LOW); 
    setupIMU();
    pixy.init();
    initializeMotors();
    xbeeComms.setupXbeeComm();  // Setup XBee communication
    xbeeComms.sendQuery();
    sent = true;
    while (robotData.matchStatus == NULL || robotData.matchStatus == 0) {
      if (xbeeComms.available()) {
      Serial.println("data recieved");
      xbeeComms.receiveData();
      sent = false;
      }
      else if (!sent) {
      Serial.println("? sent");
      xbeeComms.sendQuery();
      sent = true;
      }      
    };
}

void loop() {
  if (!sent) {
    xbeeComms.sendQuery();
    sent = true;
  }
  switch (currentState) {
    Serial.println("CHASE_FORWARD_PIXY");
    case CHASE_FORWARD_PIXY:
      chaseForward();
      if (lostCount >= 5) {
        lostCount = 0;
        currentState = CHASE_FORWARD_PING;
      }
      break;

    case CHASE_FORWARD_PING: {
      Serial.println("CHASE_FORWARD_PING");
      float pingDistance = updatePING(PIN_PING);
      if (pingDistance > 80) {
        currentState = SEARCH_FOR_PUCK;
        break;
      }
      setAllMotorSpeeds(30);
      if (pingDistance < 10) {
        currentState = TURN_TO_GOAL;
        setAllMotorSpeeds(0);
      }
      break;
    }

    case TURN_TO_GOAL: {
      Serial.println("TURN_TO_GOAL");
      swivelTurnGoal();
      currentState = MOVE_TOWARD_GOAL;
      break;
    }

    case MOVE_TOWARD_GOAL: {
      Serial.println("MOVE_TOWARD_GOAL");
      moveForwardTowardGoal(goal);
      if (distance < 40 && distance > 1) {
        distance = 41;
        currentState = STRIKE;
      }
      break;
    }

    case STRIKE: {
      Serial.println("STRIKE!");
      setAllMotorSpeeds(0);
      digitalWrite(PIN_RELAY,HIGH);
      delay(150);
      digitalWrite(PIN_RELAY,LOW);
      frontLeft->run(BACKWARD);
      frontRight->run(BACKWARD);
      backLeft->run(BACKWARD);
      backRight->run(BACKWARD);
      setAllMotorSpeeds(500);
      delay(500);
      currentState = SEARCH_FOR_PUCK;
      break;
    }

    case SEARCH_FOR_PUCK: {
      Serial.println("SEARCH_FOR_PUCK");
      logic = validSignature();
      if (logic <= -1) {
        frontLeft->run(FORWARD);
        backLeft->run(FORWARD);
        frontRight->run(BACKWARD);
        backRight->run(BACKWARD);
        setAllMotorSpeeds(60);
        delay(200);
        setAllMotorSpeeds(0);
        delay(150);
      }
      else {
        currentState = CHASE_FORWARD_PIXY;
      }
      break;
    }
  }
  if (xbeeComms.available()) {
    xbeeComms.receiveData();
    sent = false;
    while (robotData.matchStatus == NULL || robotData.matchStatus == 0) {
      setAllMotorSpeeds(0);
      if (xbeeComms.available()) {
      xbeeComms.receiveData();
      sent = false;
      }
      else if (!sent) {
      xbeeComms.sendQuery();
      sent = true;
      }      
    };
  }
}

void initializeMotors() {
  DekeShotMotors.begin();
  frontLeft = DekeShotMotors.getMotor(1);
  frontRight = DekeShotMotors.getMotor(2);  
  backLeft = DekeShotMotors.getMotor(3);  
  backRight = DekeShotMotors.getMotor(4);
}

void setAllMotorSpeeds(int speed) {
  frontLeft->setSpeed(speed);
  frontRight->setSpeed(speed);
  backLeft->setSpeed(speed);
  backRight->setSpeed(speed);
}
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  frontLeft->setSpeed(leftSpeed);   
  frontRight->setSpeed(rightSpeed);
  backLeft->setSpeed(leftSpeed);
  backRight->setSpeed(rightSpeed);
}

void chaseForward() {
  setAllMotorSpeeds(40);
  Serial.println("chaseForward");
  frontLeft->run(FORWARD);
	frontRight->run(FORWARD);
	backLeft->run(FORWARD);
	backRight->run(FORWARD);
  delay(50); 
  int errorX = pixy.ccc.blocks[index].m_x - 160;
  int proportionalSpeedAdjustment = (0.5 * errorX);
  if (lostCount == 0) {
    lastAdjustment = proportionalSpeedAdjustment;
  }
  int baseSpeed = 70; //change back to 70
  setMotorSpeeds(baseSpeed - proportionalSpeedAdjustment, baseSpeed + proportionalSpeedAdjustment);
  logic = validSignature();  
  if(logic < 0) {
    lostCount++;
  }
  else {
    lostCount = 0;
  }
}

float updatePING(int inputPin) {
    pinMode(inputPin, OUTPUT);
    digitalWrite(inputPin, LOW);
    delayMicroseconds(2);
    digitalWrite(inputPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(inputPin, LOW);
    pinMode(inputPin, INPUT);
    return (pulseIn(inputPin, HIGH) / 2) * 0.0343; 
}

int validSignature() {
    int blockCount = pixy.ccc.getBlocks();
    if (blockCount > 0) {
        index = 9999;
        for (int i = 0; i < blockCount; ++i) {
            if (pixy.ccc.blocks[i].m_signature == 7) { // puck is at 7
                if (pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height > 10 && pixy.ccc.blocks[i].m_y > 80) {
                    if (i < index) {
                      index = i;
                      return i;
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
}


//true = red goal, false = pink goal
void swivelTurnGoal() {
  setAllMotorSpeeds(0);
  xbeeComms.handleXbeeComm();
  bool turnWay = (robotData.posY > 60 ? true : false);
  delay(100);
  setAllMotorSpeeds(0);
  delay(100);
  wideTurnDumb(getGoalAngle(goal_x,goal_y),turnWay);
}

void wideTurnDumb(double degrees, bool further_side) {
  while (abs(degrees - readHeading()) > 3) {
    setMotorSpeeds((further_side ? 55 : 10), (further_side ? 10 : 55));
  }
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
  return (360 - event.orientation.x);
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
  float distance = distanceToGoal(goal_x, goal_y);
  float targetGoalAngle = getGoalAngle(goal_x,goal_y);
  setMotorSpeeds(80,90);
  distance = distanceToGoal(goal_x, goal_y);
}

float distanceToGoal(float goal_x, float goal_y) {
    // Sanity checks
if (robotData.posX == NULL) return -1; // posX is not set or invalid
if (robotData.posY == NULL) return -2; // posY is not set or invalid
return sqrt(pow(goal_x - robotData.posX, 2) + pow(goal_y - robotData.posY, 2));
}
