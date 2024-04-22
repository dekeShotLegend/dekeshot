/**
* Developer: Seyi R. Afolayan 
* Work: Controller Class with PID control (From Lab 4)
**/
/***************************************************************************BEGIN CODE****************************************************************/
class Controller {
public:
    Controller(Pixy2 *pixy, int pinPing, DualMAX14870MotorShield *motors, Adafruit_BNO055 *bno)
    : pixy(pixy), motors(motors), PIN_PING(pinPing), bno(bno), rollOffset(400), integral(0), previous_error(0), lastTime(0), SPEED_OF_SOUND(0.0343),
      kp_turn(2.0), ki_turn(0.1), kd_turn(0.05), kp_forward(0.4), ki_forward(0.1), kd_forward(0.01) {}

    void init() {
        while (rollOffset >= 360) {
            readRoll(rollOffset);
        }
        readRoll(roll);
        Serial.println("Initialization Complete");
    }

    void run() {
        updatePING();
        if (frontDis <= 11) {
            stopMotors();
            performUTurn();
        } else {
            switch (currentDirection) {
                case FORWARD:
                    moveForward(100);
                    break;
                case BACKWARD:
                    moveBackward(100);
                    break;
                case STRAFE_LEFT:
                    strafeLeft(100);
                    break;
                case STRAFE_RIGHT:
                    strafeRight(100);
                    break;
                default:
                    stopMotors();
                    break;
            }
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

private:
    Pixy2 *const pixy;
    DualMAX14870MotorShield *const motors;
    Adafruit_BNO055 *const bno;
    const int PIN_PING;
    const float SPEED_OF_SOUND;

    sensors_event_t orientationData;
    double roll, rollOffset;
    volatile double integral, previous_error;
    volatile unsigned long lastTime;
    const unsigned int refreshRate = 50;

    volatile double frontDis = 0;
    volatile int turnFlag = 0;

    const double kp_turn, ki_turn, kd_turn;
    const double kp_forward, ki_forward, kd_forward;
};
