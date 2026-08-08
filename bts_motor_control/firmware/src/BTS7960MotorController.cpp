#include "BTS7960MotorController.h"

BTS7960MotorController::BTS7960MotorController(uint8_t l_rpwm, uint8_t l_lpwm,
                                               uint8_t r_rpwm, uint8_t r_lpwm,
                                               uint8_t l_en, uint8_t r_en)
    : pinL_RPWM(l_rpwm), pinL_LPWM(l_lpwm), pinL_EN(l_en),
      pinR_RPWM(r_rpwm), pinR_LPWM(r_lpwm), pinR_EN(r_en),
      currentSpeed(200), currentDirection(MOTOR_STOP),
      lastCommandTime(0), safetyTimeoutMs(1500) {}

void BTS7960MotorController::begin() {
    ledcSetup(L_RPWM_CHAN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(pinL_RPWM, L_RPWM_CHAN);

    ledcSetup(L_LPWM_CHAN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(pinL_LPWM, L_LPWM_CHAN);

    ledcSetup(R_RPWM_CHAN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(pinR_RPWM, R_RPWM_CHAN);

    ledcSetup(R_LPWM_CHAN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(pinR_LPWM, R_LPWM_CHAN);

    pinMode(pinL_EN, OUTPUT);
    pinMode(pinR_EN, OUTPUT);
    digitalWrite(pinL_EN, HIGH);
    digitalWrite(pinR_EN, HIGH);

    stopRobot();
}

void BTS7960MotorController::setSpeed(uint8_t speed) {
    currentSpeed = speed;
    switch (currentDirection) {
        case MOTOR_FORWARD: forward(); break;
        case MOTOR_BACKWARD: backward(); break;
        case MOTOR_LEFT: left(); break;
        case MOTOR_RIGHT: right(); break;
        case MOTOR_FORWARD_LEFT: forwardLeft(); break;
        case MOTOR_FORWARD_RIGHT: forwardRight(); break;
        case MOTOR_BACKWARD_LEFT: backwardLeft(); break;
        case MOTOR_BACKWARD_RIGHT: backwardRight(); break;
        case MOTOR_STOP: default: stopRobot(); break;
    }
}

void BTS7960MotorController::setOutputs(uint8_t l_rpwm, uint8_t l_lpwm, uint8_t r_rpwm, uint8_t r_lpwm) {
    ledcWrite(L_RPWM_CHAN, l_rpwm);
    ledcWrite(L_LPWM_CHAN, l_lpwm);
    ledcWrite(R_RPWM_CHAN, r_rpwm);
    ledcWrite(R_LPWM_CHAN, r_lpwm);
}

void BTS7960MotorController::forward() {
    currentDirection = MOTOR_FORWARD;
    lastCommandTime = millis();
    setOutputs(currentSpeed, 0, currentSpeed, 0);
}

void BTS7960MotorController::backward() {
    currentDirection = MOTOR_BACKWARD;
    lastCommandTime = millis();
    setOutputs(0, currentSpeed, 0, currentSpeed);
}

void BTS7960MotorController::left() {
    currentDirection = MOTOR_LEFT;
    lastCommandTime = millis();
    setOutputs(0, currentSpeed, currentSpeed, 0);
}

void BTS7960MotorController::right() {
    currentDirection = MOTOR_RIGHT;
    lastCommandTime = millis();
    setOutputs(currentSpeed, 0, 0, currentSpeed);
}

void BTS7960MotorController::forwardLeft() {
    currentDirection = MOTOR_FORWARD_LEFT;
    lastCommandTime = millis();
    setOutputs(currentSpeed / 2, 0, currentSpeed, 0);
}

void BTS7960MotorController::forwardRight() {
    currentDirection = MOTOR_FORWARD_RIGHT;
    lastCommandTime = millis();
    setOutputs(currentSpeed, 0, currentSpeed / 2, 0);
}

void BTS7960MotorController::backwardLeft() {
    currentDirection = MOTOR_BACKWARD_LEFT;
    lastCommandTime = millis();
    setOutputs(0, currentSpeed / 2, 0, currentSpeed);
}

void BTS7960MotorController::backwardRight() {
    currentDirection = MOTOR_BACKWARD_RIGHT;
    lastCommandTime = millis();
    setOutputs(0, currentSpeed, 0, currentSpeed / 2);
}

void BTS7960MotorController::stopRobot() {
    currentDirection = MOTOR_STOP;
    lastCommandTime = millis();
    setOutputs(0, 0, 0, 0);
}

void BTS7960MotorController::update() {
    if (safetyTimeoutMs > 0 && currentDirection != MOTOR_STOP) {
        if (millis() - lastCommandTime > safetyTimeoutMs) {
            stopRobot();
            Serial.println("[SAFETY] Bluetooth command timeout. Motors stopped.");
        }
    }
}

void BTS7960MotorController::processCommand(char cmd) {
    switch (cmd) {
        case 'F': case 'f': forward(); break;
        case 'B': case 'b': backward(); break;
        case 'L': case 'l': left(); break;
        case 'R': case 'r': right(); break;
        case 'G': case 'g': forwardLeft(); break;
        case 'I': case 'i': forwardRight(); break;
        case 'H': case 'h': backwardLeft(); break;
        case 'J': case 'j': backwardRight(); break;
        case 'S': case 's': case 'X': case 'x': stopRobot(); break;

        case '0': setSpeed(0); break;
        case '1': setSpeed(25); break;
        case '2': setSpeed(50); break;
        case '3': setSpeed(75); break;
        case '4': setSpeed(100); break;
        case '5': setSpeed(125); break;
        case '6': setSpeed(150); break;
        case '7': setSpeed(175); break;
        case '8': setSpeed(200); break;
        case '9': setSpeed(225); break;
        case 'q': setSpeed(255); break;
        default: break;
    }
}
