#pragma once

#include <Arduino.h>

// Swerve drive: 4 DC motors on two MDD10A boards (PWM + DIR, sign-magnitude).
// ROS2 envía velocidad normalizada [-1, 1]; el MCU cierra el lazo con PID + encoders.

static constexpr uint8_t NUM_DRIVE_MOTORS = 4;

// true = PID con encoders; false = solo feedforward (sin lazo cerrado)
static constexpr bool MOTOR_USE_ENCODERS = true;

// Calibración encoder: ticks por segundo cuando velocidad normalizada == 1.0
static constexpr float MAX_TICKS_PER_SEC = 2000.0f;

// Pulsos por revolución del encoder en el eje del motor (CPR * reducción)
static constexpr float ENCODER_TICKS_PER_REV = 1440.0f;

struct PidGains {
  float kp;
  float ki;
  float kd;
  float i_clamp;   // anti-windup
  float out_max;   // salida PID normalizada
};

// Solo pines físicos. El PWM usa el periférico LEDC del ESP32 por software
// (canal interno = índice del motor 0..3), no consume GPIO extra.
struct MotorChannel {
  uint8_t pwm_pin;
  uint8_t dir_pin;
  uint8_t enc_pin_a;  // señal A (interrupción)
  uint8_t enc_pin_b;  // señal B (dirección)
};

class MotorController {
public:
  void begin();
  void setSetpoint(uint8_t motor_index, float velocity);
  void applyAll(const float *velocities, uint8_t count);
  void update(uint32_t now_ms);
  void stopAll();

  float getSetpoint(uint8_t motor_index) const;
  float getMeasuredVelocity(uint8_t motor_index) const;
  float getOutput(uint8_t motor_index) const;

private:
  struct MotorState {
    float setpoint = 0.0f;
    float measured = 0.0f;
    float output = 0.0f;
    float integral = 0.0f;
    float last_error = 0.0f;
    int32_t encoder_ticks = 0;
    int32_t last_encoder_ticks = 0;
    uint32_t last_update_ms = 0;
  };

  static void encoderIsr0();
  static void encoderIsr1();
  static void encoderIsr2();
  static void encoderIsr3();

  void attachEncoders();
  float computePid(uint8_t index, float error, float dt_sec);
  void writeMotor(uint8_t motor_index, float velocity);

  static constexpr uint8_t PWM_RESOLUTION_BITS = 8;
  static constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
  static constexpr float CMD_DEADBAND = 0.05f;
  static constexpr float KFF = 0.35f;  // feedforward sobre el setpoint

  static constexpr PidGains DEFAULT_GAINS = {
      .kp = 0.8f,
      .ki = 1.2f,
      .kd = 0.02f,
      .i_clamp = 0.4f,
      .out_max = 1.0f,
  };

  const MotorChannel channels_[NUM_DRIVE_MOTORS] = {
      {19, 18, 34, 35},  // FL — pwm, dir, enc_a, enc_b
      {14, 27, 36, 39},  // FR
      {25, 33, 21, 22},  // RL
      {32, 26, 23, 17},  // RR
  };

  PidGains gains_ = DEFAULT_GAINS;
  MotorState states_[NUM_DRIVE_MOTORS];

  static MotorController *instance_;
};
