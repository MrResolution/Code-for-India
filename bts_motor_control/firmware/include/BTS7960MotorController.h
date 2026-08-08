#ifndef BTS7960_MOTOR_CONTROLLER_H
#define BTS7960_MOTOR_CONTROLLER_H

#include <Arduino.h>

/**
 * @brief ESP32 Pin Definitions for BTS7960 High-Power H-Bridge Motor Drivers
 */
#define DEFAULT_L_RPWM 25
#define DEFAULT_L_LPWM 26
#define DEFAULT_L_EN   32  // Left Driver Enable Pin (R_EN + L_EN tied)

#define DEFAULT_R_RPWM 27
#define DEFAULT_R_LPWM 14
#define DEFAULT_R_EN   33  // Right Driver Enable Pin (R_EN + L_EN tied)

// PWM Frequency & Resolution for ESP32 LEDC
#define PWM_FREQ       20000 // 20 kHz (eliminates audible motor whine)
#define PWM_RESOLUTION 8     // 8-bit resolution (0 - 255)

#define L_RPWM_CHAN 0
#define L_LPWM_CHAN 1
#define R_RPWM_CHAN 2
#define R_LPWM_CHAN 3

enum MotorDirection {
    MOTOR_STOP,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_LEFT,
    MOTOR_RIGHT,
    MOTOR_FORWARD_LEFT,
    MOTOR_FORWARD_RIGHT,
    MOTOR_BACKWARD_LEFT,
    MOTOR_BACKWARD_RIGHT
};

class BTS7960MotorController {
private:
    uint8_t pinL_RPWM, pinL_LPWM, pinL_EN;
    uint8_t pinR_RPWM, pinR_LPWM, pinR_EN;

    uint8_t currentSpeed;
    MotorDirection currentDirection;
    unsigned long lastCommandTime;
    uint32_t safetyTimeoutMs;

    void setOutputs(uint8_t l_rpwm, uint8_t l_lpwm, uint8_t r_rpwm, uint8_t r_lpwm);

public:
    BTS7960MotorController(uint8_t l_rpwm = DEFAULT_L_RPWM, uint8_t l_lpwm = DEFAULT_L_LPWM,
                           uint8_t r_rpwm = DEFAULT_R_RPWM, uint8_t r_lpwm = DEFAULT_R_LPWM,
                           uint8_t l_en = DEFAULT_L_EN, uint8_t r_en = DEFAULT_R_EN);

    void begin();
    void setSpeed(uint8_t speed);
    uint8_t getSpeed() const { return currentSpeed; }
    void setSafetyTimeout(uint32_t timeoutMs) { safetyTimeoutMs = timeoutMs; }

    void forward();
    void backward();
    void left();
    void right();
    void forwardLeft();
    void forwardRight();
    void backwardLeft();
    void backwardRight();
    void stopRobot();

    void update();
    void processCommand(char cmd);
};

#endif // BTS7960_MOTOR_CONTROLLER_H
