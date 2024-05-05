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

bool goal = false;

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
    pinMode(PIN_PING,OUTPUT);
    pinMode(PIN_LEFT_PING,OUTPUT);
    pinMode(PIN_RIGHT_PING,OUTPUT);
    pinMode(PIN_RELAY,OUTPUT);
    digitalWrite(PIN_RELAY,LOW); 
    setupIMU();
    pixy.init();
    initializeMotors();
    xbeeComms.setupXbeeComm();  // Setup XBee communication
    xbeeComms.handleXbeeComm(); // single check
    while(robotData.matchStatus != "1") {
      xbeeComms.handleXbeeComm();
    }
}

void loop() {
    logic = validSignature();
    if (logic <= -1) {
      setAllMotorSpeeds(40);
      frontLeft->run(FORWARD);
      backLeft->run(FORWARD);
      frontRight->run(BACKWARD);
      backRight->run(BACKWARD);
      delay(200);
      setAllMotorSpeeds(0);
      delay(150);
    }
    else {
      chaseForward();
    }
    Serial.println(logic);
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
      frontLeft->run(FORWARD);
      backLeft->run(FORWARD);
      frontRight->run(BACKWARD);
      backRight->run(BACKWARD);
      delay(200);
      setAllMotorSpeeds(0);
  Serial.println("chaseForward");
  int lostCount = 0;
  frontLeft->run(FORWARD);
	frontRight->run(FORWARD);
	backLeft->run(FORWARD);
	backRight->run(FORWARD);
  delay(1000);
  int lastAdjustment = 0;
  while (lostCount < 5){
    int proportionalSpeedAdjustment = lastAdjustment;
    if (lostCount == 0) {
    int errorX = pixy.ccc.blocks[index].m_x - 160;
    proportionalSpeedAdjustment = (0.5 * errorX);
    lastAdjustment= proportionalSpeedAdjustment;
    }
    int baseSpeed = 40; //change back to 70
    setMotorSpeeds(baseSpeed - proportionalSpeedAdjustment, baseSpeed + proportionalSpeedAdjustment);
    logic = validSignature();  
    if(logic < 0) {
      lostCount++;
    }
    else {
      lostCount = 0;
    }
  }
  float pingDistance = updatePING(PIN_PING);
  if (pingDistance > 80) {
    setAllMotorSpeeds(0);
    return;
  }
  setAllMotorSpeeds(30);
  while(pingDistance > 10) {
    pingDistance = updatePING(PIN_PING);
    if (pingDistance < 30) {
      setAllMotorSpeeds(pingDistance);
    }
  }
  setAllMotorSpeeds(0);
  swivelTurnGoal(goal);
  moveForwardTowardGoal(goal);
  setAllMotorSpeeds(0);
  delay(500);
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
void swivelTurnGoal(bool goal) {
  setAllMotorSpeeds(0);
  xbeeComms.handleXbeeComm();
  float goal_y = 60.0;
  float goal_x = (goal ? 25.0 : 225.0);
  bool turnWay = (robotData.posY > 60 ? true : false);
  setAllMotorSpeeds(40);
  delay(100);
  setAllMotorSpeeds(0);
  delay(100);
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
  float goal_y = 60.0;
  float goal_x = (goal ? 25.0 : 225.0);
  float distance = distanceToGoal(goal_x, goal_y);
  float targetGoalAngle = getGoalAngle(goal_x,goal_y);
  int baseSlowSpeed = 40;
  do {
    setMotorSpeeds(40,45);
    xbeeComms.handleXbeeComm();
    distance = distanceToGoal(goal_x, goal_y);
  } while (distance > 40 || distance < 1);
  setAllMotorSpeeds(0);
  Serial.println("STRIKE!");
  setAllMotorSpeeds(255);
  delay(150);
  /*digitalWrite(PIN_RELAY,HIGH);
  delay(150);
  digitalWrite(PIN_RELAY,LOW);*/
  frontLeft->run(BACKWARD);
  frontRight->run(BACKWARD);
  backLeft->run(BACKWARD);
  backRight->run(BACKWARD);
  delay(500);
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