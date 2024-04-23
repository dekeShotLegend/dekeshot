/**
* Developer: Seyi R. Afolayan
* Work: Controller Class with PID control and Mecanum wheel integration
**/

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

class Controller {
public:
    Controller(Pixy2 *pixy, int pinPing, Adafruit_MotorShield *shield, Adafruit_BNO055 *bno)
    : pixy(pixy), shield(shield), PIN_PING(pinPing), bno(bno), rollOffset(0), integral(0), previous_error(0), lastTime(0), SPEED_OF_SOUND(0.0343),
      kp_turn(2.0), ki_turn(0.1), kd_turn(0.05), kp_forward(0.4), ki_forward(0.1), kd_forward(0.01) {
        motors[0] = shield->getMotor(1);
        motors[1] = shield->getMotor(2);
        motors[2] = shield->getMotor(3);
        motors[3] = shield->getMotor(4);
    }

    void init() {
        shield->begin();
        Serial.begin(9600);
        Serial.println("Initialization Complete - Mecanum Wheel Robot");
        if (!bno->begin()) {
            Serial.print("No BNO055 detected");
        }
        bno->setExtCrystalUse(true);
        readRoll(rollOffset); // Initial reading to set baseline roll offset
    }

    void run() {
        updatePING();
        if (frontDis <= 11) {
            stopAllMotors();
            performUTurn();
        } else {
            moveForward(100);  // Add conditional control logic as needed
        }
    }

    void stopAllMotors() {
        for (int i = 0; i < 4; i++) {
            motors[i]->run(RELEASE);
        }
    }

    void moveForward(int speed) {
        applyMotorCommands(FORWARD, FORWARD, FORWARD, FORWARD, speed);
    }

    void strafeRight(int speed) {
        applyMotorCommands(FORWARD, BACKWARD, BACKWARD, FORWARD, speed);
    }

    void performUTurn() {
        turn(180, kp_turn, ki_turn, kd_turn);
    }

    void applyMotorCommands(uint8_t fl, uint8_t fr, uint8_t bl, uint8_t br, int speed) {
        motors[0]->setSpeed(speed);
        motors[0]->run(fl);
        motors[1]->setSpeed(speed);
        motors[1]->run(fr);
        motors[2]->setSpeed(speed);
        motors[2]->run(bl);
        motors[3]->setSpeed(speed);
        motors[3]->run(br);
    }

    void turn(double degrees, double kp, double ki, double kd) {
        double outputRoll = 0;
        double dev = roll - rollOffset;
        normalizeAngle(dev);
        while (PID(kp, ki, kd, degrees, dev, outputRoll)) {
            delay(10);
            readRoll(roll);
            dev = roll - rollOffset;
            normalizeAngle(dev);
        }
        stopAllMotors();
        rollOffset += degrees;
        normalizeAngle(rollOffset);
    }

    bool PID(double Kp, double Ki, double Kd, double target, double input, double &output) {
        unsigned long now = millis();
        if (now - lastTime < refreshRate) {
            return true;
        }
        lastTime = now;
        double error = target - input;
        integral += error * (refreshRate / 1000.0);
        double derivative = (error - previous_error) / (refreshRate / 1000.0);
        output = Kp * error + Ki * integral + Kd * derivative;

        if (output > 150) output = 150;
        if (output < -150) output = -150;

        previous_error = error;
        return fabs(error) > 1.0;
    }

    void normalizeAngle(double &angle) {
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
    }

    void readRoll(double &angle) {
        sensors_event_t event;
        bno->getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
        angle = event.orientation.x; // Assuming roll is around the x-axis
    }

private:
    Pixy2 *const pixy;
    Adafruit_MotorShield *const shield;
    Adafruit_DCMotor *motors[4];
    Adafruit_BNO055 *const bno;
    const int PIN_PING;
    const float SPEED_OF_SOUND;

    double roll, rollOffset;
    volatile double integral, previous_error;
    volatile unsigned long lastTime;
    const unsigned int refreshRate = 50;

    volatile double frontDis = 0;
    const double kp_turn, ki_turn, kd_turn;
    const double kp_forward, ki_forward, kd_forward;
};
