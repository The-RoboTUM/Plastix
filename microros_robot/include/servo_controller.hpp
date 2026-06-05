#pragma once

#include <Arduino.h>
#include <SMS_STS.h>

// Feetech STS3215 on Waveshare Serial Bus Servo Driver (TTL @ 1 Mbps).
// Velocities are normalized [-1.0, 1.0] from ROS2.

static constexpr uint8_t NUM_STEER_SERVOS = 4;

// UART to Waveshare board (change pins if your ESP32-EVB wiring differs)
static constexpr int SERVO_UART_RX_PIN = 16;
static constexpr int SERVO_UART_TX_PIN = 17;
static constexpr uint32_t SERVO_UART_BAUD = 1000000;

// STS3215 speed scale: full command at |velocity| == 1.0
static constexpr int16_t SERVO_MAX_SPEED = 3400;
static constexpr uint8_t SERVO_ACCELERATION = 50;

// Default Feetech IDs (set during servo configuration)
static constexpr uint8_t SERVO_IDS[NUM_STEER_SERVOS] = {1, 2, 3, 4};

class ServoController {
public:
  void begin();
  void setVelocity(uint8_t servo_index, float velocity);
  void applyAll(const float *velocities, uint8_t count);
  void stopAll();

private:
  SMS_STS sts_;
  HardwareSerial servo_serial_{1};
};
