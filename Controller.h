/**
* Developer: Seyi R. Afolayan 
* Work: Controller Class with PID control (From Lab 4)
**/
/***************************************************************************BEGIN CODE****************************************************************/
class Controller {
public:
    Controller(Pixy2 *const pixy, const int PIN_PING, DualMAX14870MotorShield *const motors, Adafruit_BNO055 *const bno)
    : pixy(pixy), motors(motors), PIN_PING(PIN_PING), bno(bno), rollOffset(400), integral(0), previous_error(0), lastTime(0) {
        kp_turn = 2.0;
        ki_turn = 0.1;
        kd_turn = 0.05;
        kp_forward = 0.4;
        ki_forward = 0.1;
        kd_forward = 0.01;
        SPEED_OF_SOUND = 0.0343;  
    }

    void init() {
        while (rollOffset >= 360) {
            readRoll(rollOffset);
        }
        readRoll(roll);
        Serial.print("rollOffset: ");
        Serial.println(rollOffset);
        Serial.print("roll: ");
        Serial.println(roll);
    }

    void run() {
        updatePING();
        if (frontDis <= 11) {
            stopMotors();
            performUTurn();
        } else {
            moveForward(100); 
        }
    }

    void updatePING() {
        pinMode(PIN_PING, OUTPUT);
        digitalWrite(PIN_PING, LOW);
        delayMicroseconds(5);
        digitalWrite(PIN_PING, HIGH);
        delayMicroseconds(5);
        digitalWrite(PIN_PING, LOW);
        pinMode(PIN_PING, INPUT);
        unsigned long pulseDuration = pulseIn(PIN_PING, HIGH);
        frontDis = (pulseDuration / 2) * SPEED_OF_SOUND;
    }

    void stopMotors() {
        motors->setM1Speed(0);
        motors->setM2Speed(0);
        motors->setM3Speed(0);
        motors->setM4Speed(0);
    }

    void moveForward(int speed) {
        // Example of using PID for forward motion
        double outputRoll = 0;
        readRoll(roll);
        double dev = roll - rollOffset;
        normalizeAngle(dev);
        PID(kp_forward, ki_forward, kd_forward, 0, dev, outputRoll);
        motors->setM1Speed(speed + outputRoll);
        motors->setM2Speed(speed - outputRoll);
        motors->setM3Speed(speed + outputRoll);
        motors->setM4Speed(speed - outputRoll);
    }

    void performUTurn() {
        turn(180, kp_turn, ki_turn, kd_turn);
    }

    void turn(double degrees, double kp, double ki, double kd) {
        readRoll(roll);
        double outputRoll = 0;
        double dev = roll - rollOffset;
        normalizeAngle(dev);
        while (PID(kp, ki, kd, degrees, dev, outputRoll)) {
            delay(10);
            readRoll(roll);
            dev = roll - rollOffset;
            normalizeAngle(dev);
        }
        stopMotors();
        rollOffset += degrees;
        normalizeAngle(rollOffset);
        turnFlag = 0;
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

        if(output > 150) output = 150;
        if(output < -150) output = -150;

        previous_error = error;
        return fabs(error) > 1.0;  
    }

    void normalizeAngle(double &angle) {
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
    }

private:
    Pixy2 *const pixy;
    DualMAX14870MotorShield *const motors;
    Adafruit_BNO055 *const bno;
    const int PIN_PING;
    float SPEED_OF_SOUND;

    sensors_event_t orientationData;
    double roll, rollOffset;
    volatile double integral, previous_error;
    volatile unsigned long lastTime;
    const unsigned int refreshRate = 50; 

    volatile double frontDis = 0;
    volatile int turnFlag = 0;

    double kp_turn, ki_turn, kd_turn;
    double kp_forward, ki_forward, kd_forward;
};
