#include <Arduino.h>
#include <driver/ledc.h>

const int PWM_PIN = 25;
const int PWM_FREQ = 50;       // 50 Hz (20 ms)
const int PWM_RES_BITS = 16;   // 0..65535

static inline uint32_t usToDuty(uint32_t pulse_us) {
  // duty = (pulse_us / period_us) * (2^res - 1)
  const uint32_t maxDuty = (1UL << PWM_RES_BITS) - 1;
  const uint32_t period_us = 1000000UL / PWM_FREQ; // 20000 us
  return (pulse_us * maxDuty) / period_us;
}

void setup() {
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES_BITS);

  // Neutro 1500 us
  ledcWrite(PWM_PIN, usToDuty(1500));
}

void loop() {
  ledcWrite(PWM_PIN, usToDuty(1000)); // min
  delay(2000);

  ledcWrite(PWM_PIN, usToDuty(1500)); // neutral
  delay(2000);

  ledcWrite(PWM_PIN, usToDuty(2000)); // max
  delay(2000);
}
