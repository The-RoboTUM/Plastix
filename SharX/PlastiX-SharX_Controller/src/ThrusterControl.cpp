#include "ThrusterControl.h"
#include "config.h" // We need this for the ESC timing constants

ThrusterControl::ThrusterControl(uint8_t pin, uint8_t channel) {
    pwmPin = pin;
    pwmChannel = channel;
}

void ThrusterControl::begin() {
    // Configure the LEDC timer for this specific channel
    ledcSetup(pwmChannel, ESC_PWM_FREQ, ESC_PWM_RESOLUTION);
    
    // Attach the timer to our specific GPIO pin
    ledcAttachPin(pwmPin, pwmChannel);
    
    // Send the neutral signal immediately so the ESC arms properly
    setSpeed(0); 
}

void ThrusterControl::setSpeed(int speedPercent) {
    // 1. Safety first: Constrain input to valid bounds
    speedPercent = constrain(speedPercent, -100, 100);
    
    // 2. Map the percentage to the 16-bit timer resolution (0 to 65535)
    // At 50Hz, the period is 20ms. 
    // 1000us (reverse) = 5% of 20ms   = 0.05 * 65535 = 3276
    // 1500us (neutral) = 7.5% of 20ms = 0.075 * 65535 = 4915
    // 2000us (forward) = 10% of 20ms  = 0.10 * 65535 = 6553
    
    uint32_t dutyCycle = map(speedPercent, -100, 100, 3276, 6553);
    
    // 3. Write to hardware
    ledcWrite(pwmChannel, dutyCycle);
}