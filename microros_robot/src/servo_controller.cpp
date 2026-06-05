#include "servo_controller.hpp"

void ServoController::begin() {
  servo_serial_.begin(SERVO_UART_BAUD, SERIAL_8N1, SERVO_UART_RX_PIN, SERVO_UART_TX_PIN);
  sts_.pSerial = &servo_serial_;
  delay(300);

  for (uint8_t i = 0; i < NUM_STEER_SERVOS; ++i) {
    sts_.WheelMode(SERVO_IDS[i]);
    sts_.WriteSpe(SERVO_IDS[i], 0, SERVO_ACCELERATION);
  }
}

void ServoController::setVelocity(uint8_t servo_index, float velocity) {
  if (servo_index >= NUM_STEER_SERVOS) {
    return;
  }

  if (velocity > 1.0f) {
    velocity = 1.0f;
  } else if (velocity < -1.0f) {
    velocity = -1.0f;
  }

  const int16_t speed = static_cast<int16_t>(velocity * static_cast<float>(SERVO_MAX_SPEED));
  sts_.WriteSpe(SERVO_IDS[servo_index], speed, SERVO_ACCELERATION);
}

void ServoController::applyAll(const float *velocities, uint8_t count) {
  const uint8_t n = (count < NUM_STEER_SERVOS) ? count : NUM_STEER_SERVOS;
  for (uint8_t i = 0; i < n; ++i) {
    setVelocity(i, velocities[i]);
  }
}

void ServoController::stopAll() {
  for (uint8_t i = 0; i < NUM_STEER_SERVOS; ++i) {
    sts_.WriteSpe(SERVO_IDS[i], 0, SERVO_ACCELERATION);
  }
}
