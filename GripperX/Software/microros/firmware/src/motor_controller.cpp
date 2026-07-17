#include "motor_controller.hpp"

MotorController::MotorController(uint8_t pinA, uint8_t pinB, DriveMode mode)
    : mode_(mode),
      pinA_(pinA),
      pinB_(pinB)
{
}

void MotorController::begin() {
    pinMode(pinA_, OUTPUT);
    pinMode(pinB_, OUTPUT);
    stopMotor();
}

void MotorController::stopMotor() {
    pwmOutput_ = 0;

    if (mode_ == DriveMode::In1In2) {
        analogWrite(pinA_, 0);
        analogWrite(pinB_, 0);
        return;
    }

    digitalWrite(pinB_, LOW);
    analogWrite(pinA_, 0);
}

void MotorController::setTargetRPM(float rpm) {
    targetRPM_ = rpm;

    if (targetRPM_ == 0.0) {
        stopMotor();
        return;
    }

    if (mode_ == DriveMode::PwmDir) {
        applyPwmDir(rpm);
    } else {
        applyIn1In2(rpm);
    }
}

void MotorController::applyPwmDir(float rpm) {
    if (rpm > 0.0) {
        direction_ = 1;
        digitalWrite(pinB_, LOW);
    } else {
        direction_ = -1;
        digitalWrite(pinB_, HIGH);
    }

    pwmOutput_ = constrain((int)(fabs(rpm) * pwmPerRPM_), 0, 255);
    analogWrite(pinA_, pwmOutput_);
}

void MotorController::applyIn1In2(float rpm) {
    pwmOutput_ = constrain((int)(fabs(rpm) * pwmPerRPM_), 0, 255);

    // Always turn off both with PWM before changing direction.
    // Mixing digitalWrite + analogWrite on ESP32 leaves the DBH with IN1 and IN2 active.
    analogWrite(pinA_, 0);
    analogWrite(pinB_, 0);

    if (rpm > 0.0) {
        direction_ = 1;
        if (pwmOutput_ > 0) {
            analogWrite(pinA_, pwmOutput_);
        }
    } else {
        direction_ = -1;
        if (pwmOutput_ > 0) {
            analogWrite(pinB_, pwmOutput_);
        }
    }
}

void MotorController::setFeedForward(float pwmPerRPM) {
    pwmPerRPM_ = pwmPerRPM;
}

float MotorController::getTargetRPM() const {
    return targetRPM_;
}

float MotorController::getRPM() const {
    return targetRPM_;
}

int MotorController::getPWM() const {
    return pwmOutput_;
}
