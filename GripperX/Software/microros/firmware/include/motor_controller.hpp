#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <Arduino.h>

enum class DriveMode {
    PwmDir,   // PWM + direction pin (motors 1 and 2)
    In1In2    // DBH-1A: IN1=PWM forward, IN2=PWM reverse (motors 3 and 4)
};

class MotorController {
public:
    MotorController(uint8_t pinA, uint8_t pinB, DriveMode mode = DriveMode::PwmDir);

    void begin();
    void setTargetRPM(float rpm);
    void setFeedForward(float pwmPerRPM);

    float getTargetRPM() const;
    float getRPM() const;
    int getPWM() const;

private:
    DriveMode mode_;
    uint8_t pinA_;
    uint8_t pinB_;

    float targetRPM_ = 0.0;

    float pwmPerRPM_ = 0.90;

    int pwmOutput_ = 0;
    int direction_ = 1;

    void stopMotor();
    void applyPwmDir(float rpm);
    void applyIn1In2(float rpm);
};

#endif
