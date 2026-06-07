#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <Arduino.h>

#define MAX_MOTORS 4

class MotorController {
public:
    MotorController(
        uint8_t motorID,
        uint8_t pwmPin,
        uint8_t dirPin,
        uint8_t encoderPin
    );

    void begin();

    void setPID(float kp, float ki, float kd);
    void setTargetRPM(float targetRPM);
    void update();

    void moveForward(uint8_t speed);
    void moveBackward(uint8_t speed);
    void stop();

    long getEncoderCount();
    long getAndResetEncoderCount();
    float getCurrentRPM();

private:
    uint8_t _motorID;
    uint8_t _pwmPin;
    uint8_t _dirPin;
    uint8_t _encoderPin;

    float _targetRPM = 0.0;
    float _currentRPM = 0.0;

    float _kp = 0.5;
    float _ki = 0.35;
    float _kd = 0.0;

    float _integral = 0.0;
    float _previousError = 0.0;

    uint8_t _pwmOutput = 0;

    unsigned long _lastUpdateTime = 0;
    const unsigned long _sampleTimeMs = 200;

    static constexpr float _countsPerRev = 555.0;

    volatile long _encoderCount = 0;

    static MotorController* _instances[MAX_MOTORS];

    static void IRAM_ATTR encoderISR0();
    static void IRAM_ATTR encoderISR1();
    static void IRAM_ATTR encoderISR2();
    static void IRAM_ATTR encoderISR3();

    void IRAM_ATTR handleEncoderInterrupt();
};

#endif