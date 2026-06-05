#include "motor_controller.hpp"

MotorController *MotorController::instance_ = nullptr;

void MotorController::encoderIsr0() {
  if (!instance_) {
    return;
  }
  const MotorChannel &ch = instance_->channels_[0];
  if (digitalRead(ch.enc_pin_b) != digitalRead(ch.enc_pin_a)) {
    instance_->states_[0].encoder_ticks++;
  } else {
    instance_->states_[0].encoder_ticks--;
  }
}

void MotorController::encoderIsr1() {
  if (!instance_) {
    return;
  }
  const MotorChannel &ch = instance_->channels_[1];
  if (digitalRead(ch.enc_pin_b) != digitalRead(ch.enc_pin_a)) {
    instance_->states_[1].encoder_ticks++;
  } else {
    instance_->states_[1].encoder_ticks--;
  }
}

void MotorController::encoderIsr2() {
  if (!instance_) {
    return;
  }
  const MotorChannel &ch = instance_->channels_[2];
  if (digitalRead(ch.enc_pin_b) != digitalRead(ch.enc_pin_a)) {
    instance_->states_[2].encoder_ticks++;
  } else {
    instance_->states_[2].encoder_ticks--;
  }
}

void MotorController::encoderIsr3() {
  if (!instance_) {
    return;
  }
  const MotorChannel &ch = instance_->channels_[3];
  if (digitalRead(ch.enc_pin_b) != digitalRead(ch.enc_pin_a)) {
    instance_->states_[3].encoder_ticks++;
  } else {
    instance_->states_[3].encoder_ticks--;
  }
}

void MotorController::begin() {
  instance_ = this;

  for (uint8_t i = 0; i < NUM_DRIVE_MOTORS; ++i) {
    pinMode(channels_[i].dir_pin, OUTPUT);
    ledcSetup(i, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(channels_[i].pwm_pin, i);

    states_[i] = MotorState{};
    states_[i].last_update_ms = millis();
    writeMotor(i, 0.0f);
  }

  if (MOTOR_USE_ENCODERS) {
    attachEncoders();
  }
}

void MotorController::attachEncoders() {
  using IsrFn = void (*)();
  const IsrFn isrs[NUM_DRIVE_MOTORS] = {encoderIsr0, encoderIsr1, encoderIsr2,
                                        encoderIsr3};

  for (uint8_t i = 0; i < NUM_DRIVE_MOTORS; ++i) {
    pinMode(channels_[i].enc_pin_a, INPUT_PULLUP);
    pinMode(channels_[i].enc_pin_b, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(channels_[i].enc_pin_a), isrs[i],
                    CHANGE);
  }
}

void MotorController::setSetpoint(uint8_t motor_index, float velocity) {
  if (motor_index >= NUM_DRIVE_MOTORS) {
    return;
  }
  if (velocity > 1.0f) {
    velocity = 1.0f;
  } else if (velocity < -1.0f) {
    velocity = -1.0f;
  }
  if (fabsf(velocity) < CMD_DEADBAND) {
    velocity = 0.0f;
  }
  states_[motor_index].setpoint = velocity;
}

void MotorController::applyAll(const float *velocities, uint8_t count) {
  const uint8_t n = (count < NUM_DRIVE_MOTORS) ? count : NUM_DRIVE_MOTORS;
  for (uint8_t i = 0; i < n; ++i) {
    setSetpoint(i, velocities[i]);
  }
}

void MotorController::update(uint32_t now_ms) {
  for (uint8_t i = 0; i < NUM_DRIVE_MOTORS; ++i) {
    MotorState &st = states_[i];
    const uint32_t dt_ms = now_ms - st.last_update_ms;
    if (dt_ms < 5) {
      continue;
    }

    const float dt_sec = static_cast<float>(dt_ms) * 0.001f;
    st.last_update_ms = now_ms;

    if (MOTOR_USE_ENCODERS) {
      noInterrupts();
      const int32_t ticks = st.encoder_ticks;
      interrupts();

      const int32_t delta_ticks = ticks - st.last_encoder_ticks;
      st.last_encoder_ticks = ticks;

      const float ticks_per_sec = static_cast<float>(delta_ticks) / dt_sec;
      st.measured = ticks_per_sec / MAX_TICKS_PER_SEC;
      if (st.measured > 1.0f) {
        st.measured = 1.0f;
      } else if (st.measured < -1.0f) {
        st.measured = -1.0f;
      }
    } else {
      // Sin encoder: solo feedforward (no hay medición real de velocidad)
      st.measured = 0.0f;
    }

    const float error = st.setpoint - st.measured;
    float command = 0.0f;
    if (MOTOR_USE_ENCODERS) {
      const float pid_out = computePid(i, error, dt_sec);
      command = (KFF * st.setpoint) + pid_out;
    } else {
      command = st.setpoint;
    }
    if (command > 1.0f) {
      command = 1.0f;
    } else if (command < -1.0f) {
      command = -1.0f;
    }

    st.output = command;
    writeMotor(i, command);
  }
}

float MotorController::computePid(uint8_t index, float error, float dt_sec) {
  if (dt_sec <= 0.0f) {
    return 0.0f;
  }

  MotorState &st = states_[index];

  st.integral += error * dt_sec;
  if (st.integral > gains_.i_clamp) {
    st.integral = gains_.i_clamp;
  } else if (st.integral < -gains_.i_clamp) {
    st.integral = -gains_.i_clamp;
  }

  const float derivative = (error - st.last_error) / dt_sec;
  st.last_error = error;

  float out = (gains_.kp * error) + (gains_.ki * st.integral) +
              (gains_.kd * derivative);

  if (out > gains_.out_max) {
    out = gains_.out_max;
  } else if (out < -gains_.out_max) {
    out = -gains_.out_max;
  }

  return out;
}

void MotorController::stopAll() {
  for (uint8_t i = 0; i < NUM_DRIVE_MOTORS; ++i) {
    states_[i].setpoint = 0.0f;
    states_[i].output = 0.0f;
    states_[i].integral = 0.0f;
    states_[i].last_error = 0.0f;
    writeMotor(i, 0.0f);
  }
}

float MotorController::getSetpoint(uint8_t motor_index) const {
  return (motor_index < NUM_DRIVE_MOTORS) ? states_[motor_index].setpoint : 0.0f;
}

float MotorController::getMeasuredVelocity(uint8_t motor_index) const {
  return (motor_index < NUM_DRIVE_MOTORS) ? states_[motor_index].measured : 0.0f;
}

float MotorController::getOutput(uint8_t motor_index) const {
  return (motor_index < NUM_DRIVE_MOTORS) ? states_[motor_index].output : 0.0f;
}

void MotorController::writeMotor(uint8_t motor_index, float velocity) {
  if (motor_index >= NUM_DRIVE_MOTORS) {
    return;
  }

  const MotorChannel &ch = channels_[motor_index];

  if (velocity > 1.0f) {
    velocity = 1.0f;
  } else if (velocity < -1.0f) {
    velocity = -1.0f;
  }

  const bool forward = velocity >= 0.0f;
  digitalWrite(ch.dir_pin, forward ? HIGH : LOW);

  const uint8_t duty = static_cast<uint8_t>(fabsf(velocity) * 255.0f);
  ledcWrite(motor_index, duty);
}
