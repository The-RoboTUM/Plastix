#include "motor_controller.hpp"

MotorController* MotorController::_instances[MAX_MOTORS] = {
    nullptr, nullptr, nullptr, nullptr
};

MotorController::MotorController(
    uint8_t motorID,
    uint8_t pwmPin,
    uint8_t dirPin,
    uint8_t encoderPin
)
    : _motorID(motorID),
      _pwmPin(pwmPin),
      _dirPin(dirPin),
      _encoderPin(encoderPin)
{
}

void MotorController::begin() {
    pinMode(_dirPin, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
    pinMode(_encoderPin, INPUT_PULLUP);

    if (_motorID < MAX_MOTORS) {
        _instances[_motorID] = this;
    }

    switch (_motorID) {
        case 0:
            attachInterrupt(digitalPinToInterrupt(_encoderPin), encoderISR0, RISING);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(_encoderPin), encoderISR1, RISING);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(_encoderPin), encoderISR2, RISING);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(_encoderPin), encoderISR3, RISING);
            break;
    }

    _lastUpdateTime = millis();

    stop();
}

void MotorController::setPID(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void MotorController::setTargetRPM(float targetRPM) {
    _targetRPM = targetRPM;
}

void MotorController::update() {
    unsigned long now = millis();
    unsigned long dtMs = now - _lastUpdateTime;

    if (dtMs < _sampleTimeMs) {
        return;
    }

    _lastUpdateTime = now;

    float dt = dtMs / 1000.0;

    long pulses = getAndResetEncoderCount();

    float revolutions = (float)pulses / _countsPerRev;
    float measuredRPM = (revolutions / dt) * 60.0;

    _currentRPM = 0.7 * _currentRPM + 0.3 * measuredRPM;

    if (_targetRPM == 0) {
        stop();
        _integral = 0.0;
        _previousError = 0.0;
        _pwmOutput = 0;
        return;
    }

    float targetAbsRPM = abs(_targetRPM);
    float error = targetAbsRPM - _currentRPM;

    _integral += error * dt;

    if (_integral > 200) _integral = 200;
    if (_integral < -200) _integral = -200;

    float derivative = (error - _previousError) / dt;
    _previousError = error;

    float output = (_kp * error) + (_ki * _integral) + (_kd * derivative);

    float minPWM = 25;
    float desiredPWM = minPWM + output;

    if (desiredPWM > 255) desiredPWM = 255;
    if (desiredPWM < 0) desiredPWM = 0;

    int maxStep = 5;

    if (desiredPWM > _pwmOutput + maxStep) {
        _pwmOutput += maxStep;
    } else if (desiredPWM < _pwmOutput - maxStep) {
        _pwmOutput -= maxStep;
    } else {
        _pwmOutput = desiredPWM;
    }

    if (_targetRPM > 0) {
        moveForward(_pwmOutput);
    } else {
        moveBackward(_pwmOutput);
    }

    Serial.print("Motor ");
    Serial.print(_motorID);
    Serial.print(" | Target RPM: ");
    Serial.print(_targetRPM);
    Serial.print(" | Measured RPM: ");
    Serial.print(measuredRPM);
    Serial.print(" | Filtered RPM: ");
    Serial.print(_currentRPM);
    Serial.print(" | PWM: ");
    Serial.println(_pwmOutput);
}

void MotorController::moveForward(uint8_t speed) {
    digitalWrite(_dirPin, LOW);
    analogWrite(_pwmPin, speed);
}

void MotorController::moveBackward(uint8_t speed) {
    digitalWrite(_dirPin, HIGH);
    analogWrite(_pwmPin, speed);
}

void MotorController::stop() {
    analogWrite(_pwmPin, 0);
}

long MotorController::getEncoderCount() {
    noInterrupts();
    long count = _encoderCount;
    interrupts();
    return count;
}

long MotorController::getAndResetEncoderCount() {
    noInterrupts();
    long count = _encoderCount;
    _encoderCount = 0;
    interrupts();
    return count;
}

float MotorController::getCurrentRPM() {
    return _currentRPM;
}

void IRAM_ATTR MotorController::handleEncoderInterrupt() {
    _encoderCount++;
}

void IRAM_ATTR MotorController::encoderISR0() {
    if (_instances[0] != nullptr) {
        _instances[0]->handleEncoderInterrupt();
    }
}

void IRAM_ATTR MotorController::encoderISR1() {
    if (_instances[1] != nullptr) {
        _instances[1]->handleEncoderInterrupt();
    }
}

void IRAM_ATTR MotorController::encoderISR2() {
    if (_instances[2] != nullptr) {
        _instances[2]->handleEncoderInterrupt();
    }
}

void IRAM_ATTR MotorController::encoderISR3() {
    if (_instances[3] != nullptr) {
        _instances[3]->handleEncoderInterrupt();
    }
}